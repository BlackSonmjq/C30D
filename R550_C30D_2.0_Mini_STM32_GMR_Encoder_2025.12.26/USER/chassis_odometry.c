#include "chassis_odometry.h"
#include "route_config.h"
#include "heading_control.h"
#include "imu_task.h"
#include "nav_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

static ChassisOdomFrame pose;
static uint8_t sampled;
static uint8_t gap_fault;

void ChassisOdom_Reset(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    memset(&pose, 0, sizeof(pose));
    pose.tick_ms = now_ms;
    pose.valid = 0U;
    sampled = gap_fault = 0U;
    taskEXIT_CRITICAL();
}

void ChassisOdom_UpdateDelta(float forward_mm, float lateral_mm, short gyro_z,
                            uint32_t now_ms, uint8_t calibrated)
{
    uint32_t elapsed;
    float dt, middle_yaw, dx, dy;
    IMU_HeadingFrame heading;
    (void)gyro_z;
    Imu_ReadHeading(&heading);
    calibrated = calibrated && heading.ready &&
        (uint32_t)(now_ms-heading.tick_ms) < NAV_IMU_TTL_MS;

    taskENTER_CRITICAL();
    elapsed = now_ms - pose.tick_ms;
    pose.tick_ms = now_ms;
    pose.sample_dt_ms = elapsed;
    if (!calibrated || !sampled) {
        pose.valid = 0U;
        pose.vx_mps = pose.vy_mps = 0.0f;
        sampled = calibrated;
        if (calibrated) pose.yaw_rad = heading.yaw_rad;
        taskEXIT_CRITICAL();
        return;
    }
    if (elapsed > pose.max_sample_dt_ms) pose.max_sample_dt_ms = elapsed;
    if (elapsed > ROUTE_ODOM_MAX_GAP_MS) {
        gap_fault = 1U;
        pose.valid = 0U;
        pose.vx_mps = pose.vy_mps = 0.0f;
        taskEXIT_CRITICAL();
        return;
    }
    dt = (float)elapsed * 0.001f;
    middle_yaw = pose.yaw_rad +
        HeadingControl_NormalizeAngle(heading.yaw_rad-pose.yaw_rad)*0.5f;
    /* 传入的是本次编码器位移，禁止再乘 dt。即使同一个 tick 内补执行，
     * 也只累加这次读取到的新计数；dt 为 0 时不计算速度、不积分角速度。 */
    dx = forward_mm * ROUTE_ODOM_FORWARD_SCALE;
    dy = lateral_mm * ROUTE_ODOM_LATERAL_SCALE;
    pose.vx_mps = dt > 0.0f ? dx * 0.001f / dt : 0.0f;
    pose.vy_mps = dt > 0.0f ? dy * 0.001f / dt : 0.0f;
    pose.x_mm += cosf(middle_yaw) * dx - sinf(middle_yaw) * dy;
    pose.y_mm += sinf(middle_yaw) * dx + cosf(middle_yaw) * dy;
    pose.yaw_rad = heading.yaw_rad;
    pose.valid = !gap_fault;
    taskEXIT_CRITICAL();
}

void ChassisOdom_Read(ChassisOdomFrame *frame)
{
    taskENTER_CRITICAL();
    *frame = pose;
    taskEXIT_CRITICAL();
}

void ChassisOdom_CorrectAxis(uint8_t axis, float position_mm)
{
    taskENTER_CRITICAL();
    if (axis == 0U) pose.x_mm = position_mm;
    else if (axis == 1U) pose.y_mm = position_mm;
    taskEXIT_CRITICAL();
}
