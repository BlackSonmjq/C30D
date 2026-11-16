#ifndef WHEEL_OUTPUT_H
#define WHEEL_OUTPUT_H

/* 重载麦轮：输出 = 速度 PI ×倍率 + 目标轮速前馈 + 前轮侧移额外补偿。
 * 前馈跟随各轮目标速度，先给基础输出，再由 PI 修正实际速度偏差。
 * 参数是试调初值，尚未经过载荷标定；此处不增加硬件最大占空比。
 * 前轮 B/C 的侧移额外补偿仍由 lateral_friction.h 控制。 */
#define WHEEL_OUTPUT_BOOST_ENABLE       1      // PI 输出倍率开关
#define WHEEL_OUTPUT_GAIN_PERCENT       100    // 先保持原 PI 倍率，避免和新前馈同时大幅增益
#define WHEEL_OUTPUT_PWM_LIMIT          16700  // 保留原硬件输出上限

#define WHEEL_FEEDFORWARD_ENABLE        1      // 1=启用四轮速度前馈，0=关闭前馈
#define WHEEL_FEEDFORWARD_KS_PWM        1000.0f // 静摩擦基础前馈，PWM 计数，四轮共用初值
#define WHEEL_FEEDFORWARD_KV_PWM        4000.0f // 速度前馈系数，单位 PWM/(m/s)
#define WHEEL_FEEDFORWARD_MIN_MPS       0.005f  // 目标轮速不超过此值时前馈为零
#define WHEEL_FEEDFORWARD_MAX_PWM       6000.0f // 单轮前馈限幅，保留 PI 调节余量

#if WHEEL_OUTPUT_GAIN_PERCENT < 100 || WHEEL_OUTPUT_GAIN_PERCENT > 200
#error Wheel_output_gain_must_be_between_100_and_200_percent
#endif

/* target 是硬件电机极性变换前的目标轮速，单位 m/s。 */
static float WheelOutput_Feedforward(float target, int mecanum)
{
    float speed, output;
    if (!WHEEL_FEEDFORWARD_ENABLE || !mecanum) return 0.0f;
    speed = target < 0.0f ? -target : target;
    if (speed <= WHEEL_FEEDFORWARD_MIN_MPS) return 0.0f;
    output = WHEEL_FEEDFORWARD_KS_PWM + WHEEL_FEEDFORWARD_KV_PWM * speed;
    if (output > WHEEL_FEEDFORWARD_MAX_PWM) output = WHEEL_FEEDFORWARD_MAX_PWM;
    return target > 0.0f ? output : -output;
}

/* 按当前前馈和倍率反算 PI 上下限，避免总输出已饱和而 PI 仍继续积累。
 * 不同方向的前馈会形成不同 PI 余量，不能只限制 PI 本身到 ±16700。 */
static float WheelOutput_ClampPi(float pwm, float target, int mecanum)
{
    float ff = WheelOutput_Feedforward(target, mecanum);
    float gain = WHEEL_OUTPUT_BOOST_ENABLE && mecanum ? WHEEL_OUTPUT_GAIN_PERCENT / 100.0f : 1.0f;
    float upper = (WHEEL_OUTPUT_PWM_LIMIT - ff) / gain;
    float lower = (-WHEEL_OUTPUT_PWM_LIMIT - ff) / gain;
    if (pwm > upper) pwm = upper;
    if (pwm < lower) pwm = lower;
    return pwm;
}

static int WheelOutput_Apply(int pwm, float target, int mecanum)
{
    long output = pwm;
    if (WHEEL_OUTPUT_BOOST_ENABLE && mecanum)
        output = output * WHEEL_OUTPUT_GAIN_PERCENT / 100;
    output += (long)WheelOutput_Feedforward(target, mecanum);
    if (output > WHEEL_OUTPUT_PWM_LIMIT) output = WHEEL_OUTPUT_PWM_LIMIT;
    if (output < -WHEEL_OUTPUT_PWM_LIMIT) output = -WHEEL_OUTPUT_PWM_LIMIT;
    return (int)output;
}

#endif
