#include "heading_control.h"

#define HEADING_CONTROL_PI 3.14159265358979323846f

typedef struct
{
    float yaw;
    float integral;
    u8 initialized;
} HEADING_CONTROL_STATE_t;

static HEADING_CONTROL_STATE_t heading_state;

static float HeadingControl_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float HeadingControl_Limit(float value, float lower, float upper)
{
    if(value < lower) return lower;
    if(value > upper) return upper;
    return value;
}

static float HeadingControl_NormalizeAngle(float angle)
{
    const float two_pi = 2.0f * HEADING_CONTROL_PI;

    while(angle > HEADING_CONTROL_PI) angle -= two_pi;
    while(angle < -HEADING_CONTROL_PI) angle += two_pi;
    return angle;
}

void HeadingControl_Reset(void)
{
    heading_state.yaw = 0.0f;
    heading_state.integral = 0.0f;
    heading_state.initialized = 0;
}

static float HeadingControl_Output(float error, float gyro_z_rad_s, float dt)
{
    float output, minimum, error_scale;
    heading_state.integral = HeadingControl_Limit(
        heading_state.integral + error * dt,
        -HEADING_CONTROL_INTEGRAL_LIMIT,
        HEADING_CONTROL_INTEGRAL_LIMIT);
    /*
     * Do not release the correction while the chassis is still rotating.
     * Using gyro rate directly for the D term also avoids amplifying angle
     * integration noise and provides immediate braking near the target.
     */
    if((HeadingControl_Abs(error) <= HEADING_CONTROL_YAW_TOLERANCE) &&
       (HeadingControl_Abs(gyro_z_rad_s) <= HEADING_CONTROL_GYRO_TOLERANCE))
    {
        heading_state.integral = 0.0f;
        return 0.0f;
    }

    output = HEADING_CONTROL_KP * error
           + HEADING_CONTROL_KI * heading_state.integral
           - HEADING_CONTROL_KD * gyro_z_rad_s;

    /* 仅在近乎静止、且 PID 仍要求朝误差方向运动时补起转死区。
     * 靠近目标逐步退出，不把小误差强行抬到固定 0.08 rad/s。
     * D 项要求反向制动或恰好抵消 P 项时，原样保留，不强造纠偏方向。 */
    if(HeadingControl_Abs(gyro_z_rad_s) <= HEADING_CONTROL_MIN_ASSIST_RATE &&
       output * error > 0.0f)
    {
        error_scale = HeadingControl_Limit(
            HeadingControl_Abs(error) / HEADING_CONTROL_MIN_FULL_ERROR_RAD, 0.0f, 1.0f);
        minimum = HEADING_CONTROL_MIN_ANGULAR_VELOCITY * error_scale;
        if(HeadingControl_Abs(output) < minimum)
            output = output > 0.0f ? minimum : -minimum;
    }

    return HeadingControl_Limit(
        output,
        -HEADING_CONTROL_MAX_ANGULAR_VELOCITY,
        HEADING_CONTROL_MAX_ANGULAR_VELOCITY);
}

float HeadingControl_Update(short gyro_z_raw)
{
    const float dt = 1.0f / HEADING_CONTROL_RATE_HZ;
    const float gyro_z_dps =
        (float)gyro_z_raw * HEADING_CONTROL_GYRO_FULL_SCALE_DPS / 32768.0f;
    const float gyro_z_rad_s = gyro_z_dps * HEADING_CONTROL_PI / 180.0f;
    const float gyro_z_for_integration =
        HeadingControl_Abs(gyro_z_rad_s) <= HEADING_CONTROL_GYRO_INTEGRATION_DEADZONE ?
        0.0f : gyro_z_rad_s;
    float error;

    /* Lock the current chassis direction when ROS serial control takes over. */
    if(heading_state.initialized == 0)
    {
        HeadingControl_Reset();
        heading_state.initialized = 1;
        return 0.0f;
    }

    heading_state.yaw = HeadingControl_NormalizeAngle(
        heading_state.yaw + gyro_z_for_integration * dt);
    error = HeadingControl_NormalizeAngle(-heading_state.yaw);

    return HeadingControl_Output(error, gyro_z_rad_s, dt);
}

/* Route odometry keeps yaw continuous while parked or waiting for the tool. */
float HeadingControl_HoldYaw(float target_yaw, float measured_yaw, short gyro_z_raw)
{
    const float gyro_rate = (float)gyro_z_raw * HEADING_CONTROL_GYRO_FULL_SCALE_DPS *
                            HEADING_CONTROL_PI / (32768.0f * 180.0f);
    return HeadingControl_Output(HeadingControl_NormalizeAngle(target_yaw - measured_yaw),
                                 gyro_rate, 1.0f / HEADING_CONTROL_RATE_HZ);
}
