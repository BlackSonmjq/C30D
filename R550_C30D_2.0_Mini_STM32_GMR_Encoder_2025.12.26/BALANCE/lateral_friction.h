#ifndef LATERAL_FRICTION_H
#define LATERAL_FRICTION_H

#include <math.h>

/* 麦轮侧移低速静摩擦补偿。PWM 量程约为 ±16700，以下不是实测死区值。
 * 补偿叠加在速度 PI 输出之后，不累加到 PI 内部积分。
 * 车轮转起来后按实际轮速逐渐减小补偿；停车或反向制动时不加助力。 */
#define LATERAL_FRICTION_ENABLE           1       // 1=启用，0=关闭
#define LATERAL_FRICTION_FRONT_MASK       6U      // 用户确认前左 B、前右 C；掩码 B=2 加 C=4
#define LATERAL_FRICTION_FRONT_LEFT_POS   500     // 前左轮正转补偿，PWM 计数；约满量程的 3%
#define LATERAL_FRICTION_FRONT_LEFT_NEG   500     // 前左轮反转补偿，填写正数
#define LATERAL_FRICTION_FRONT_RIGHT_POS  500     // 前右轮正转补偿，PWM 计数
#define LATERAL_FRICTION_FRONT_RIGHT_NEG  500     // 前右轮反转补偿，填写正数
#define LATERAL_FRICTION_LEFT_MOTOR       1U      // 前左轮接 B；编号 A=0、B=1、C=2、D=3
#define LATERAL_FRICTION_RIGHT_MOTOR      2U      // 前右轮接 C；与当前车头方向一致
#define LATERAL_FRICTION_MIN_Y_MPS        0.015f  // 侧向指令至少达到此速度才启用，单位 m/s
#define LATERAL_FRICTION_MAX_X_MPS        0.06f   // 允许少量纵向纠偏；明显前后行驶时不补偿
#define LATERAL_FRICTION_FADE_MPS         0.06f   // 实际轮速达到此值时补偿降为 0，单位 m/s
#define LATERAL_FRICTION_MIN_TARGET_MPS   0.005f  // 目标轮速接近 0 时不加补偿
#define LATERAL_FRICTION_PWM_LIMIT        16700   // 与原车轮 PWM 限幅一致

#if LATERAL_FRICTION_LEFT_MOTOR > 3U || LATERAL_FRICTION_RIGHT_MOTOR > 3U || \
    LATERAL_FRICTION_LEFT_MOTOR == LATERAL_FRICTION_RIGHT_MOTOR || \
    LATERAL_FRICTION_FRONT_MASK != ((1U << LATERAL_FRICTION_LEFT_MOTOR) | (1U << LATERAL_FRICTION_RIGHT_MOTOR))
#error Lateral_friction_front_wheel_configuration_invalid
#endif

/* wheel 为 A/B/C/D 的 0/1/2/3；target/feedback 是硬件极性变换前的轮速。 */
static int LateralFriction_Apply(unsigned wheel, int pwm, float target, float feedback,
                                 float requested_x, float requested_y)
{
    float fade, result;
    int amount;
    if (!LATERAL_FRICTION_ENABLE || wheel > 3U ||
        !(LATERAL_FRICTION_FRONT_MASK & (1U << wheel)) ||
        fabsf(requested_y) < LATERAL_FRICTION_MIN_Y_MPS ||
        fabsf(requested_x) > LATERAL_FRICTION_MAX_X_MPS ||
        fabsf(target) < LATERAL_FRICTION_MIN_TARGET_MPS ||
        LATERAL_FRICTION_FADE_MPS <= 0.0f ||
        fabsf(feedback) >= LATERAL_FRICTION_FADE_MPS)
        return pwm;
    /* PI 已在反向制动，或轮速已经达到目标时，不叠加驱动方向的补偿。 */
    if ((target > 0.0f && (pwm < 0 || feedback >= target)) ||
        (target < 0.0f && (pwm > 0 || feedback <= target)))
        return pwm;
    if (wheel == LATERAL_FRICTION_LEFT_MOTOR)
        amount = target > 0.0f ? LATERAL_FRICTION_FRONT_LEFT_POS : LATERAL_FRICTION_FRONT_LEFT_NEG;
    else if (wheel == LATERAL_FRICTION_RIGHT_MOTOR)
        amount = target > 0.0f ? LATERAL_FRICTION_FRONT_RIGHT_POS : LATERAL_FRICTION_FRONT_RIGHT_NEG;
    else return pwm;
    if (amount <= 0) return pwm;
    fade = 1.0f - fabsf(feedback) / LATERAL_FRICTION_FADE_MPS;
    result = pwm + (target > 0.0f ? amount * fade : -amount * fade);
    if (result > LATERAL_FRICTION_PWM_LIMIT) result = LATERAL_FRICTION_PWM_LIMIT;
    if (result < -LATERAL_FRICTION_PWM_LIMIT) result = -LATERAL_FRICTION_PWM_LIMIT;
    return (int)result;
}

#endif
