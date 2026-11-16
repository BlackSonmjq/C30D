#include "route_io.h"
#include "route_config.h"
#include "route_plan.h"
#include "my_chassis_task.h"
#include "chassis_odometry.h"
#include "balance.h"
#include "heading_control.h"
#include "stepper_task.h"
#include <math.h>
#include <string.h>

typedef struct { float vx_mps, vy_mps; TickType_t tick; } RouteCommand;
static RouteCommand command;
static volatile uint8_t start_requested;
static RouteLidarFrame lidar;
static TickType_t lidar_tick;
static uint8_t command_seen;
static volatile uint8_t pending_work, work_reserved, work_allowed;

void RouteIO_PublishLidar(uint16_t front, uint16_t behind, uint16_t left, uint16_t right)
{
    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    lidar.mm[ROUTE_LIDAR_FRONT] = front;
    lidar.mm[ROUTE_LIDAR_BEHIND] = behind;
    lidar.mm[ROUTE_LIDAR_LEFT] = left;
    lidar.mm[ROUTE_LIDAR_RIGHT] = right;
    if (++lidar.sequence == 0U) ++lidar.sequence;
    lidar.fresh = 1U;
    lidar_tick = now;
    taskEXIT_CRITICAL();
}

void RouteIO_ReadLidar(RouteLidarFrame *frame)
{
    TickType_t published_at;
    taskENTER_CRITICAL();
    *frame = lidar;
    published_at = lidar_tick;
    taskEXIT_CRITICAL();
    if (frame->sequence == 0U || (TickType_t)(xTaskGetTickCount() - published_at) >=
        pdMS_TO_TICKS(ROUTE_LIDAR_TTL_MS)) {
        memset(frame->mm, 0, sizeof(frame->mm));
        frame->fresh = 0U;
    }
}

uint8_t RouteIO_BaseReady(void)
{
    return (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550) &&
           SysVal.Time_count >= CONTROL_DELAY && Voltage >= 10.0f && EN && !Flag_Stop;
}
uint8_t RouteIO_Ready(void)
{
    return RouteIO_BaseReady() && !StepperTask_IsBusy();
}
uint8_t RouteIO_ControlOwned(void)
{
    return ROUTE_DEMO_ENABLE && (!ROUTE_DEBUG_MODE || start_requested);
}

void RouteIO_RequestStart(void)
{
    RouteLidarFrame frame;
    ChassisOdomFrame pose;
    RouteIO_ReadLidar(&frame);
    ChassisOdom_Read(&pose);
    if (!start_requested && Route_TaskCreated && RoutePlan_Calibrated() &&
        RouteIO_Ready() && RouteIO_ChassisStopped() && frame.fresh && pose.valid) {
        ChassisOdom_Reset((uint32_t)xTaskGetTickCount());
        HeadingControl_Reset();
        start_requested = 1U;
    }
}
uint8_t RouteIO_StartRequested(void) { return start_requested; }

void RouteIO_SetVelocity(float vx_mps, float vy_mps)
{
    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    command.vx_mps = vx_mps;
    command.vy_mps = vy_mps;
    command.tick = now;
    command_seen = 1U;
    taskEXIT_CRITICAL();
}

void RouteIO_ReadDiagnostics(RouteIODiagnostics *d)
{
    RouteCommand latest;
    RouteLidarFrame frame;
    uint8_t seen;
    taskENTER_CRITICAL();
    latest = command;
    seen = command_seen;
    taskEXIT_CRITICAL();
    RouteIO_ReadLidar(&frame);
    d->can_fresh = frame.fresh;
    d->rear_mm = frame.mm[ROUTE_LIDAR_BEHIND];
    d->command_fresh = seen && (TickType_t)(xTaskGetTickCount() - latest.tick) <
                       pdMS_TO_TICKS(ROUTE_COMMAND_TTL_MS);
    d->vx_mps = latest.vx_mps;
    d->vy_mps = latest.vy_mps;
}

void RouteIO_ApplyDrive(void)
{
    RouteCommand latest;
    ChassisOdomFrame pose;
    taskENTER_CRITICAL();
    latest = command;
    taskEXIT_CRITICAL();
    ChassisOdom_Read(&pose);
    if (!start_requested || !RouteIO_Ready() || !pose.valid ||
        (TickType_t)(xTaskGetTickCount() - latest.tick) >= pdMS_TO_TICKS(ROUTE_COMMAND_TTL_MS) ||
        (latest.vx_mps == 0.0f && latest.vy_mps == 0.0f)) {
        smooth_control.VX = smooth_control.VY = smooth_control.VZ = 0.0f;
        Drive_Motor(0.0f, 0.0f, 0.0f);
        return;
    }
    Drive_Motor(latest.vx_mps, latest.vy_mps, HEADING_CONTROL_ENABLE ?
                HeadingControl_HoldYaw(0.0f, pose.yaw_rad, imu.gyro.z) : 0.0f);
}

uint8_t RouteIO_ChassisStopped(void)
{
    return fabsf(MOTOR_A.Encoder) <= ROUTE_STOP_WHEEL_MPS &&
           fabsf(MOTOR_B.Encoder) <= ROUTE_STOP_WHEEL_MPS &&
           fabsf(MOTOR_C.Encoder) <= ROUTE_STOP_WHEEL_MPS &&
           fabsf(MOTOR_D.Encoder) <= ROUTE_STOP_WHEEL_MPS;
}

uint8_t RouteIO_QueueWorkFromISR(uint8_t mango_id)
{
    uint32_t mask = __get_PRIMASK();
    uint8_t accepted = 0U;
    __disable_irq();
    if (mango_id >= 1U && mango_id <= 4U && start_requested && work_allowed && !work_reserved && !StepperTask_IsBusy()) {
        pending_work = mango_id;
        work_reserved = 1U;
        work_allowed = 0U;
        accepted = 1U;
    }
    __set_PRIMASK(mask);
    return accepted;
}
uint8_t RouteIO_TakeWorkRequest(void)
{
    uint32_t mask = __get_PRIMASK();
    uint8_t id;
    __disable_irq();
    id = pending_work;
    pending_work = 0U;
    __set_PRIMASK(mask);
    return id;
}
void RouteIO_AllowWork(uint8_t allowed) { work_allowed = allowed; }
void RouteIO_CompleteWork(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    pending_work = work_reserved = 0U;
    __set_PRIMASK(mask);
}
