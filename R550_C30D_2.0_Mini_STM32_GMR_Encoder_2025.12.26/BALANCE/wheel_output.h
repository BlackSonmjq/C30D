#ifndef WHEEL_OUTPUT_H
#define WHEEL_OUTPUT_H

/* One per-wheel output model for every mecanum motion source. Wheel targets
 * stay unchanged; calibration and measured speed determine feedforward/load
 * assist. Map-front A/D are heavier in the current 180-degree orientation.
 * Values are initial PE-mat tuning, not measured mass or motor torque. */
#define WHEEL_OUTPUT_BOOST_ENABLE       1
#define WHEEL_OUTPUT_GAIN_PERCENT       100
#define WHEEL_OUTPUT_PWM_LIMIT          16700
#define WHEEL_FEEDFORWARD_ENABLE        1
#define WHEEL_FEEDFORWARD_MIN_MPS       0.005f
#define WHEEL_FEEDFORWARD_FULL_MPS      0.03f
#define WHEEL_FEEDFORWARD_MAX_PWM       6000.0f
#define WHEEL_LOAD_ASSIST_ENABLE        1
#define WHEEL_LOAD_FULL_ERROR_MPS       0.03f
/* Preserve each profile's original small-error assist slope while allowing
 * larger maximum assist under substantial speed deficit. */
#define WHEEL_HEAVY_LOAD_FULL_ERROR_MPS 0.03f
/* Fade the driving feedforward when a lightly loaded wheel runs ahead.
 * PI braking remains intact; allow a small encoder-speed tolerance. */
#define WHEEL_OVERSPEED_ALLOWANCE_MPS   0.005f
#define WHEEL_OVERSPEED_FADE_MPS        0.03f
#define WHEEL_HEAVY_OVERSPEED_FADE_MPS  0.03f

/* Extra breakaway PWM only for lateral translation near standstill. */
#define WHEEL_STRAFE_START_HEAVY_PWM    2000.0f
#define WHEEL_STRAFE_START_LIGHT_PWM    1000.0f
#define WHEEL_STRAFE_START_EXIT_MPS     0.04f
static int wheel_output_lateral_start;
static void WheelOutput_SetMotion(float vx, float vy)
{
    float x = vx < 0.0f ? -vx : vx;
    float y = vy < 0.0f ? -vy : vy;
    wheel_output_lateral_start = x <= 0.005f && y > 0.005f;
}

/* PWM and PWM/(m/s); forward/reverse Ks can be calibrated separately. */
#define WHEEL_A_KS_FORWARD_PWM          2000.0f
#define WHEEL_A_KS_REVERSE_PWM          2000.0f
#define WHEEL_A_KV_PWM                  5000.0f
#define WHEEL_A_ERROR_ASSIST_PWM        1500.0f
#define WHEEL_B_KS_FORWARD_PWM          1000.0f
#define WHEEL_B_KS_REVERSE_PWM          1000.0f
#define WHEEL_B_KV_PWM                  4000.0f
#define WHEEL_B_ERROR_ASSIST_PWM        500.0f
#define WHEEL_C_KS_FORWARD_PWM          1000.0f
#define WHEEL_C_KS_REVERSE_PWM          1000.0f
#define WHEEL_C_KV_PWM                  4000.0f
#define WHEEL_C_ERROR_ASSIST_PWM        500.0f
#define WHEEL_D_KS_FORWARD_PWM          2000.0f
#define WHEEL_D_KS_REVERSE_PWM          2000.0f
#define WHEEL_D_KV_PWM                  5000.0f
#define WHEEL_D_ERROR_ASSIST_PWM        1500.0f

#if WHEEL_OUTPUT_GAIN_PERCENT < 100 || WHEEL_OUTPUT_GAIN_PERCENT > 200
#error Wheel_output_gain_must_be_between_100_and_200_percent
#endif

typedef struct {
    float ks_forward_pwm, ks_reverse_pwm, kv_pwm, error_assist_pwm, full_error_mps, overspeed_fade_mps;
} WheelOutput_Calibration;
static const WheelOutput_Calibration wheel_output_calibration[4] = {
    {WHEEL_A_KS_FORWARD_PWM, WHEEL_A_KS_REVERSE_PWM, WHEEL_A_KV_PWM, WHEEL_A_ERROR_ASSIST_PWM, WHEEL_HEAVY_LOAD_FULL_ERROR_MPS, WHEEL_HEAVY_OVERSPEED_FADE_MPS},
    {WHEEL_B_KS_FORWARD_PWM, WHEEL_B_KS_REVERSE_PWM, WHEEL_B_KV_PWM, WHEEL_B_ERROR_ASSIST_PWM, WHEEL_LOAD_FULL_ERROR_MPS, WHEEL_OVERSPEED_FADE_MPS},
    {WHEEL_C_KS_FORWARD_PWM, WHEEL_C_KS_REVERSE_PWM, WHEEL_C_KV_PWM, WHEEL_C_ERROR_ASSIST_PWM, WHEEL_LOAD_FULL_ERROR_MPS, WHEEL_OVERSPEED_FADE_MPS},
    {WHEEL_D_KS_FORWARD_PWM, WHEEL_D_KS_REVERSE_PWM, WHEEL_D_KV_PWM, WHEEL_D_ERROR_ASSIST_PWM, WHEEL_HEAVY_LOAD_FULL_ERROR_MPS, WHEEL_HEAVY_OVERSPEED_FADE_MPS}
};

static float WheelOutput_Taper(float speed)
{
    if (speed <= WHEEL_FEEDFORWARD_MIN_MPS) return 0.0f;
    if (speed >= WHEEL_FEEDFORWARD_FULL_MPS) return 1.0f;
    return (speed - WHEEL_FEEDFORWARD_MIN_MPS) /
           (WHEEL_FEEDFORWARD_FULL_MPS - WHEEL_FEEDFORWARD_MIN_MPS);
}

static float WheelOutput_Feedforward(unsigned wheel, float target, int mecanum)
{
    const WheelOutput_Calibration *cal;
    float speed = target < 0.0f ? -target : target;
    float output;
    if (!WHEEL_FEEDFORWARD_ENABLE || !mecanum || wheel >= 4U) return 0.0f;
    cal = &wheel_output_calibration[wheel];
    output = ((target > 0.0f ? cal->ks_forward_pwm : cal->ks_reverse_pwm) +
              cal->kv_pwm * speed) * WheelOutput_Taper(speed);
    if (output > WHEEL_FEEDFORWARD_MAX_PWM) output = WHEEL_FEEDFORWARD_MAX_PWM;
    return target > 0.0f ? output : -output;
}

static float WheelOutput_LoadAssist(unsigned wheel, float pwm, float target, float feedback, int mecanum)
{
    float speed = target < 0.0f ? -target : target;
    float sign = target > 0.0f ? 1.0f : -1.0f;
    float error, ratio;
    if (!WHEEL_LOAD_ASSIST_ENABLE || !mecanum || wheel >= 4U ||
        speed <= WHEEL_FEEDFORWARD_MIN_MPS || pwm * sign < 0.0f) return 0.0f;
    error = speed - feedback * sign;
    if (error <= 0.0f) return 0.0f;
    ratio = error / wheel_output_calibration[wheel].full_error_mps;
    if (ratio > 1.0f) ratio = 1.0f;
    return sign * wheel_output_calibration[wheel].error_assist_pwm * ratio * WheelOutput_Taper(speed);
}

static float WheelOutput_TrackingFeedforward(unsigned wheel, float target, float feedback, int mecanum)
{
    float speed = target < 0.0f ? -target : target;
    float sign = target > 0.0f ? 1.0f : -1.0f;
    float excess = feedback * sign - speed - WHEEL_OVERSPEED_ALLOWANCE_MPS;
    float output = WheelOutput_Feedforward(wheel, target, mecanum);
    if (!mecanum || wheel >= 4U) return output;
    if (excess <= 0.0f) return output;
    if (excess >= wheel_output_calibration[wheel].overspeed_fade_mps) return 0.0f;
    return output * (1.0f - excess / wheel_output_calibration[wheel].overspeed_fade_mps);
}

static float WheelOutput_StartupAssist(unsigned wheel, float pwm, float target, float feedback, int mecanum)
{
    float speed = target < 0.0f ? -target : target;
    float sign = target > 0.0f ? 1.0f : -1.0f;
    float measured = feedback * sign;
    float exit_speed, amount;
    if (!mecanum || !wheel_output_lateral_start || wheel >= 4U ||
        speed <= WHEEL_FEEDFORWARD_MIN_MPS || pwm * sign < 0.0f) return 0.0f;
    exit_speed = speed < WHEEL_STRAFE_START_EXIT_MPS ? speed : WHEEL_STRAFE_START_EXIT_MPS;
    if (measured >= exit_speed) return 0.0f;
    if (measured < 0.0f) measured = 0.0f;
    amount = wheel == 0U || wheel == 3U ? WHEEL_STRAFE_START_HEAVY_PWM : WHEEL_STRAFE_START_LIGHT_PWM;
    return sign * amount * (1.0f - measured / exit_speed) * WheelOutput_Taper(speed);
}

/* Both the PI clamp and final output use the same wheel/feedback/bias. This
 * reserves headroom for ALL compensation before the integrator saturates. */
static float WheelOutput_ClampPi(unsigned wheel, float pwm, float target, float feedback, int mecanum)
{
    float bias = WheelOutput_TrackingFeedforward(wheel, target, feedback, mecanum) +
                 WheelOutput_LoadAssist(wheel, pwm, target, feedback, mecanum) +
                 WheelOutput_StartupAssist(wheel, pwm, target, feedback, mecanum);
    float gain = WHEEL_OUTPUT_BOOST_ENABLE && mecanum ? WHEEL_OUTPUT_GAIN_PERCENT / 100.0f : 1.0f;
    float upper = (WHEEL_OUTPUT_PWM_LIMIT - bias) / gain;
    float lower = (-WHEEL_OUTPUT_PWM_LIMIT - bias) / gain;
    if (pwm > upper) pwm = upper;
    if (pwm < lower) pwm = lower;
    return pwm;
}

static int WheelOutput_Apply(unsigned wheel, int pwm, float target, float feedback, int mecanum)
{
    float gain = WHEEL_OUTPUT_BOOST_ENABLE && mecanum ? WHEEL_OUTPUT_GAIN_PERCENT / 100.0f : 1.0f;
    float output = pwm * gain + WheelOutput_TrackingFeedforward(wheel, target, feedback, mecanum) +
                   WheelOutput_LoadAssist(wheel, (float)pwm, target, feedback, mecanum) +
                   WheelOutput_StartupAssist(wheel, (float)pwm, target, feedback, mecanum);
    if (output > WHEEL_OUTPUT_PWM_LIMIT) output = WHEEL_OUTPUT_PWM_LIMIT;
    if (output < -WHEEL_OUTPUT_PWM_LIMIT) output = -WHEEL_OUTPUT_PWM_LIMIT;
    return (int)output;
}
#endif
