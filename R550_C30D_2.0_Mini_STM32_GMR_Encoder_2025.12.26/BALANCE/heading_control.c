#include "heading_control.h"

#define HEADING_CONTROL_PI 3.14159265358979323846f

typedef struct
{
    float yaw;
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

float HeadingControl_NormalizeAngle(float angle)
{
    const float two_pi = 2.0f * HEADING_CONTROL_PI;

    while(angle > HEADING_CONTROL_PI) angle -= two_pi;
    while(angle < -HEADING_CONTROL_PI) angle += two_pi;
    return angle;
}

void HeadingControl_Reset(void)
{
    heading_state.yaw = 0.0f;
    heading_state.initialized = 0;
}

float HeadingControl_GetIntegralOutput(void)
{
    return 0.0f;
}

static float HeadingControl_Output(float error, float gyro_z_rad_s, float dt)
{
    (void)dt;
    if (HeadingControl_Abs(error) <= HEADING_CONTROL_YAW_TOLERANCE) {
        if (HeadingControl_Abs(gyro_z_rad_s) <= HEADING_CONTROL_GYRO_TOLERANCE)
            return 0.0f;
        error = 0.0f;
    }
    return HeadingControl_Limit(HEADING_CONTROL_KP*error-HEADING_CONTROL_KD*gyro_z_rad_s,
        -HEADING_CONTROL_MAX_ANGULAR_VELOCITY, HEADING_CONTROL_MAX_ANGULAR_VELOCITY);
}

float HeadingControl_Update(short gyro_z_raw)
{
    const float dt = 1.0f / HEADING_CONTROL_RATE_HZ;
    const float gyro_z_dps =
        NAV_GYRO_SIGN * (float)gyro_z_raw / NAV_GYRO_LSB_PER_DPS;
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
    const float gyro_rate = NAV_GYRO_SIGN * (float)gyro_z_raw * HEADING_CONTROL_PI / (NAV_GYRO_LSB_PER_DPS * 180.0f);
    return HeadingControl_Output(HeadingControl_NormalizeAngle(target_yaw - measured_yaw),
                                 gyro_rate, 1.0f / HEADING_CONTROL_RATE_HZ);
}
