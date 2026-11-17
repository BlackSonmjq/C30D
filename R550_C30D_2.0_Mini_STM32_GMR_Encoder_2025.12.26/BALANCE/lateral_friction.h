#ifndef LATERAL_FRICTION_H
#define LATERAL_FRICTION_H

#include <math.h>
/* Legacy standalone helper. Production uses wheel_output.h for all motions;
 * keep this disabled to avoid stacking old lateral-only compensation. */

/* 地图前方重载两轮的麦轮侧移低速静摩擦补偿。
 * 当前车体相对地图旋转 180 度：地图左前 D、右前 A。
 * PWM 量程约为 ±16700，以下不是实测死区值。
 * 补偿叠加在速度 PI 输出之后，不累加到 PI 内部积分。
 * 车轮转起来后按实际轮速逐渐减小补偿；停车或反向制动时不加助力。 */
#define LATERAL_FRICTION_ENABLE           0       // 1=启用，0=关闭
#define LATERAL_FRICTION_FRONT_MASK       9U      // 地图前方 A/D；掩码 A=1 加 D=8
#define LATERAL_FRICTION_FRONT_LEFT_POS   2500    // 地图左前 D 正转补偿；PE 防滑垫试调，约满量程的 15%
#define LATERAL_FRICTION_FRONT_LEFT_NEG   2500    // 地图左前 D 反转补偿，填写正数
#define LATERAL_FRICTION_FRONT_RIGHT_POS  2500    // 地图右前 A 正转补偿，PWM 计数
#define LATERAL_FRICTION_FRONT_RIGHT_NEG  2500    // 地图右前 A 反转补偿，填写正数
#define LATERAL_FRICTION_LEFT_MOTOR       3U      // 地图左前 D（车体后右）；编号 A=0、B=1、C=2、D=3
#define LATERAL_FRICTION_RIGHT_MOTOR      0U      // 地图右前 A（车体后左）
#define LATERAL_FRICTION_MIN_Y_MPS        0.015f  // 侧向指令至少达到此速度才启用，单位 m/s
#define LATERAL_FRICTION_MAX_X_MPS        0.06f   // 允许少量纵向纠偏；明显前后行驶时不补偿
#define LATERAL_FRICTION_FADE_MPS         0.08f   // 实际轮速达到此值时补偿降为 0，单位 m/s
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
