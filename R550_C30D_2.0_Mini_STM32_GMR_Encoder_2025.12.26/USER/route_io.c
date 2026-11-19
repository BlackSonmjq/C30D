#include "route_io.h"
#include "route_config.h"
#include "nav_can.h"
#include "nav_config.h"
#include "balance.h"
#include "stepper_task.h"
#include <math.h>
#include <string.h>

typedef struct { float vx_mps, vy_mps; TickType_t tick; uint8_t quick_stop; } RouteCommand;
static RouteCommand command;
static volatile uint8_t start_requested, start_pending, stop_pending;
static volatile uint32_t clock_ms;
static float heading_target;
static volatile NavLidar lidar;
static uint8_t command_seen;
static volatile uint8_t pending_work, work_reserved, work_allowed;

void RouteIO_Tick1ms(void) { ++clock_ms; }
uint32_t RouteIO_Millis(void) { return clock_ms; }

/* CAN priority 1 never calls FreeRTOS. Timestamp the actual received frame. */
void RouteIO_ReceiveCan(uint32_t id, uint8_t ide, uint8_t rtr, uint8_t len, const uint8_t *data)
{
    NavLidar next;
    if (!NavCan_Decode(id, ide, rtr, len, data, &next)) return;
    next.received_ms = clock_ms;
    next.sequence = lidar.sequence + 1U;
    if (!next.sequence) next.sequence = 1U;
    lidar = next;
}

void RouteIO_PublishLidar(uint16_t front, uint16_t behind, uint16_t left, uint16_t right)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    lidar.mm[NAV_FRONT] = front;
    lidar.mm[NAV_BACK] = behind;
    lidar.mm[NAV_LEFT] = left;
    lidar.mm[NAV_RIGHT] = right;
    if (++lidar.sequence == 0U) ++lidar.sequence;
    lidar.received_ms = clock_ms;
    __set_PRIMASK(mask);
}

void RouteIO_ReadNavigationLidar(NavLidar *frame)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *frame = lidar;
    __set_PRIMASK(mask);
}

void RouteIO_ReadLidar(RouteLidarFrame *frame)
{
    NavLidar snapshot;
    unsigned i;
    RouteIO_ReadNavigationLidar(&snapshot);
    frame->sequence = snapshot.sequence;
    frame->fresh = snapshot.sequence && (uint32_t)(clock_ms - snapshot.received_ms) < NAV_LIDAR_TTL_MS;
    frame->mm[ROUTE_LIDAR_FRONT] = snapshot.mm[NAV_FRONT];
    frame->mm[ROUTE_LIDAR_BEHIND] = snapshot.mm[NAV_BACK];
    frame->mm[ROUTE_LIDAR_LEFT] = snapshot.mm[NAV_LEFT];
    frame->mm[ROUTE_LIDAR_RIGHT] = snapshot.mm[NAV_RIGHT];
    if (!frame->fresh) memset(frame->mm, 0, sizeof(frame->mm));
    for (i = 0U; i < ROUTE_LIDAR_COUNT; ++i)
        if (frame->mm[i] < NAV_LIDAR_MIN_MM || frame->mm[i] > NAV_LIDAR_MAX_MM)
            frame->mm[i] = 0U;
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
uint8_t RouteIO_ControlOwned(void) { return start_requested; }
uint8_t RouteIO_ReverseTestRunning(void) { return 0U; }
void RouteIO_RequestStart(void) { start_pending = 1U; }
void RouteIO_RequestManual(void) { stop_pending = 1U; start_requested = 0U; }
static uint8_t take_request(volatile uint8_t *request)
{
    uint8_t value;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    value = *request;
    *request = 0U;
    __set_PRIMASK(mask);
    return value;
}
uint8_t RouteIO_TakeStartRequest(void) { return take_request(&start_pending); }
uint8_t RouteIO_TakeStopRequest(void) { return take_request(&stop_pending); }
void RouteIO_SetRunning(uint8_t running)
{
    /* An input interrupt/task may have requested manual takeover during Nav_Tick. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    start_requested = running && !stop_pending;
    __set_PRIMASK(mask);
}
void RouteIO_SetHeading(float yaw_rad) { heading_target = yaw_rad; }
uint8_t RouteIO_StartRequested(void) { return start_requested; }

void RouteIO_SetVelocity(float vx_mps, float vy_mps, uint8_t quick_stop)
{
    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    command.vx_mps = vx_mps;
    command.vy_mps = vy_mps;
    command.tick = now;
    command.quick_stop = quick_stop;
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
    RouteLidarFrame ranges;
    taskENTER_CRITICAL();
    latest = command;
    taskEXIT_CRITICAL();
    RouteIO_ReadLidar(&ranges);
    if (!start_requested || !RouteIO_Ready() || !command_seen || !ranges.fresh ||
        (TickType_t)(xTaskGetTickCount() - latest.tick) >= pdMS_TO_TICKS(ROUTE_COMMAND_TTL_MS)) {
        smooth_control.VX = smooth_control.VY = smooth_control.VZ = 0.0f;
        Drive_Motor(0.0f, 0.0f, 0.0f);
        return;
    }
    if (latest.quick_stop) smooth_control.VX = smooth_control.VY = 0.0f;
    Chassis_NavigationDrive(latest.vx_mps, latest.vy_mps, heading_target);
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
uint8_t RouteIO_WorkReserved(void) { return work_reserved; }
void RouteIO_AllowWork(uint8_t allowed) { work_allowed = allowed; }
void RouteIO_CompleteWork(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    pending_work = work_reserved = 0U;
    __set_PRIMASK(mask);
}
