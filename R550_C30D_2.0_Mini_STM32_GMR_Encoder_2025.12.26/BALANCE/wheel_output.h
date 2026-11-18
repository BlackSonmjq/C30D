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

/* Per-wheel breakaway assist for all mecanum motion sources. */
#define WHEEL_START_HEAVY_PWM          8000.0f
#define WHEEL_START_LIGHT_PWM          5000.0f
#define WHEEL_START_HOLD_RATIO         0.375f
#define WHEEL_START_EXIT_MPS           0.04f
#define WHEEL_START_STALL_MPS          0.005f
#define WHEEL_START_STALL_TICKS        20U
static signed char wheel_start_direction[4];
static unsigned char wheel_start_complete[4];
static unsigned char wheel_start_stall_ticks[4];
static int wheel_output_translation_active;
static int wheel_output_reset_pi;

static void WheelOutput_ResetStartup(void)
{
    unsigned wheel;
    wheel_output_translation_active = wheel_output_reset_pi = 0;
    for (wheel = 0U; wheel < 4U; ++wheel) {
        wheel_start_direction[wheel] = 0;
        wheel_start_complete[wheel] = 0U;
        wheel_start_stall_ticks[wheel] = 0U;
    }
}

/* Update once before PI and final output so both use the same startup state. */
static void WheelOutput_UpdateStartup(unsigned wheel, float target, float feedback, int active)
{
    float speed = target < 0.0f ? -target : target;
    int direction = target > 0.0f ? 1 : -1;
    float exit_speed = speed < WHEEL_START_EXIT_MPS ? speed : WHEEL_START_EXIT_MPS;
    if (!active || speed <= WHEEL_FEEDFORWARD_MIN_MPS) {
        wheel_start_direction[wheel] = 0;
        wheel_start_complete[wheel] = 0U;
        wheel_start_stall_ticks[wheel] = 0U;
        return;
    }
    if (wheel_start_direction[wheel] != direction) {
        wheel_start_direction[wheel] = (signed char)direction;
        wheel_start_complete[wheel] = 0U;
        wheel_start_stall_ticks[wheel] = 0U;
    }
    if (feedback * direction >= exit_speed) {
        wheel_start_complete[wheel] = 1U;
        wheel_start_stall_ticks[wheel] = 0U;
    } else if (wheel_start_complete[wheel] &&
               feedback >= -WHEEL_START_STALL_MPS && feedback <= WHEEL_START_STALL_MPS) {
        if (++wheel_start_stall_ticks[wheel] >= WHEEL_START_STALL_TICKS) {
            wheel_start_complete[wheel] = 0U;
            wheel_start_stall_ticks[wheel] = 0U;
        }
    } else {
        wheel_start_stall_ticks[wheel] = 0U;
    }
}

/* Extra damping on map-front A/D once all four wheel targets are zero. */
static int wheel_output_stopping;
#define WHEEL_STOP_DEADBAND_MPS         0.005f
#define WHEEL_HEAVY_STOP_PWM_PER_MPS    20000.0f
#define WHEEL_HEAVY_STOP_MAX_PWM        1200.0f
#define WHEEL_LIGHT_STOP_PWM_PER_MPS    10000.0f
#define WHEEL_LIGHT_STOP_MAX_PWM        600.0f
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
    float exit_speed, amount, ratio;
    if (!mecanum || wheel >= 4U || wheel_start_complete[wheel] ||
        speed <= WHEEL_FEEDFORWARD_MIN_MPS || pwm * sign < 0.0f) return 0.0f;
    exit_speed = speed < WHEEL_START_EXIT_MPS ? speed : WHEEL_START_EXIT_MPS;
    if (measured >= exit_speed) return 0.0f;
    if (measured < 0.0f) measured = 0.0f;
    /* Preserve translation breakaway; taper small pure-yaw targets below. */
    amount = wheel == 0U || wheel == 3U ? WHEEL_START_HEAVY_PWM : WHEEL_START_LIGHT_PWM;
    /* Small pure-yaw commands need a gradual breakaway effort near the target.
     * Translation and larger turns retain full load compensation. */
    if (!wheel_output_translation_active && speed < WHEEL_START_EXIT_MPS) {
        ratio = speed / WHEEL_START_EXIT_MPS;
        amount *= ratio * ratio;
    }
    ratio = (exit_speed - measured) / (exit_speed * (1.0f - WHEEL_START_HOLD_RATIO));
    if (ratio > 1.0f) ratio = 1.0f;
    return sign * amount * ratio * ratio;
}

static float WheelOutput_StopAssist(unsigned wheel, float feedback, int mecanum)
{
    float speed, amount, maximum;
    int heavy = wheel == 0U || wheel == 3U;
    if (!mecanum || !wheel_output_stopping || wheel >= 4U) return 0.0f;
    speed = feedback < 0.0f ? -feedback : feedback;
    if (speed <= WHEEL_STOP_DEADBAND_MPS) return 0.0f;
    amount = (speed - WHEEL_STOP_DEADBAND_MPS) *
        (heavy ? WHEEL_HEAVY_STOP_PWM_PER_MPS : WHEEL_LIGHT_STOP_PWM_PER_MPS);
    maximum = heavy ? WHEEL_HEAVY_STOP_MAX_PWM : WHEEL_LIGHT_STOP_MAX_PWM;
    if (amount > maximum) amount = maximum;
    return feedback > 0.0f ? -amount : amount;
}

/* Both the PI clamp and final output use the same wheel/feedback/bias. This
 * reserves headroom for ALL compensation before the integrator saturates. */
static float WheelOutput_ClampPi(unsigned wheel, float pwm, float target, float feedback, int mecanum)
{
    float bias, gain, upper, lower;
    /* At rest, opposite stored PI effort is stale; moving-wheel braking stays. */
    if (mecanum && feedback >= -WHEEL_START_STALL_MPS && feedback <= WHEEL_START_STALL_MPS &&
        (target > WHEEL_FEEDFORWARD_MIN_MPS || target < -WHEEL_FEEDFORWARD_MIN_MPS) &&
        pwm * target < 0.0f)
        pwm = 0.0f;
    bias = WheelOutput_TrackingFeedforward(wheel, target, feedback, mecanum) +
                 WheelOutput_LoadAssist(wheel, pwm, target, feedback, mecanum) +
                 WheelOutput_StartupAssist(wheel, pwm, target, feedback, mecanum) +
                 WheelOutput_StopAssist(wheel, feedback, mecanum);
    gain = WHEEL_OUTPUT_BOOST_ENABLE && mecanum ? WHEEL_OUTPUT_GAIN_PERCENT / 100.0f : 1.0f;
    upper = (WHEEL_OUTPUT_PWM_LIMIT - bias) / gain;
    lower = (-WHEEL_OUTPUT_PWM_LIMIT - bias) / gain;
    if (pwm > upper) pwm = upper;
    if (pwm < lower) pwm = lower;
    return pwm;
}

static int WheelOutput_Apply(unsigned wheel, int pwm, float target, float feedback, int mecanum)
{
    float gain = WHEEL_OUTPUT_BOOST_ENABLE && mecanum ? WHEEL_OUTPUT_GAIN_PERCENT / 100.0f : 1.0f;
    float output = pwm * gain + WheelOutput_TrackingFeedforward(wheel, target, feedback, mecanum) +
                   WheelOutput_LoadAssist(wheel, (float)pwm, target, feedback, mecanum) +
                   WheelOutput_StartupAssist(wheel, (float)pwm, target, feedback, mecanum) +
                   WheelOutput_StopAssist(wheel, feedback, mecanum);
    if (output > WHEEL_OUTPUT_PWM_LIMIT) output = WHEEL_OUTPUT_PWM_LIMIT;
    if (output < -WHEEL_OUTPUT_PWM_LIMIT) output = -WHEEL_OUTPUT_PWM_LIMIT;
    return (int)output;
}
#endif
