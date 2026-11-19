#include "my_chassis_task.h"
#include "route_config.h"
#include "route_io.h"
#include "chassis_odometry.h"
#include "route_debug.h"
#include "stepper_task.h"
#include "vision_align.h"
#include "nav_core.h"
#include "nav_button.h"
#include "balance.h"
#include "heading_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

volatile uint8_t Route_Stage;
static NavCore navigator;
static RouteStatus published_status;
static RouteNavigationStatus navigation_status;

void Route_ReadStatus(RouteStatus *status)
{
    taskENTER_CRITICAL();
    *status = published_status;
    taskEXIT_CRITICAL();
}
void Route_ReadNavigationStatus(RouteNavigationStatus *status)
{
    taskENTER_CRITICAL();
    *status = navigation_status;
    taskEXIT_CRITICAL();
}
static void publish_status(void)
{
    static const RoutePhase phases[] = {
        ROUTE_WAIT_READY, ROUTE_WAIT_START, ROUTE_ODOM_RUN, ROUTE_CONFIRM,
        ROUTE_WAIT_LIDAR, ROUTE_WORK_STOP, ROUTE_WAIT_TOOL, ROUTE_FAULT, ROUTE_DONE
    };
    RouteStatus legacy;
    RouteNavigationStatus snapshot;
    memset(&legacy, 0, sizeof(legacy));
    legacy.phase = phases[navigator.state];
    legacy.target_point = navigator.target;
    legacy.at_point = navigator.confirmed;
    legacy.sensor = ROUTE_LIDAR_COUNT;
    legacy.aux_sensor = ROUTE_LIDAR_COUNT;
    snapshot.state = navigator.state;
    snapshot.reason = navigator.reason;
    snapshot.target = navigator.target;
    snapshot.confirmed = navigator.confirmed;
    snapshot.main_error_mm = navigator.main_error_mm;
    snapshot.cross_error_mm = navigator.cross_error_mm;
    snapshot.heading_error_rad = navigator.heading_error_rad;
    snapshot.start_front_mm = navigator.start_front_mm;
    snapshot.command_vx_mps = navigator.output.vx;
    snapshot.command_vy_mps = navigator.output.vy;
    snapshot.reference_mask = navigator.reference_mask;
    taskENTER_CRITICAL();
    published_status = legacy;
    navigation_status = snapshot;
    Route_Stage = navigator.state == NAV_DONE ? 18U : navigator.target;
    taskEXIT_CRITICAL();
}

void my_chassis_task(void *parameters)
{
    const TickType_t period = pdMS_TO_TICKS(ROUTE_PERIOD_MS);
    TickType_t wake = xTaskGetTickCount();
    NavInput in;
    NavLidar lidar;
    ChassisOdomFrame pose, previous_pose;
    StepperTask_Data tool;
    uint32_t previous_ms = RouteIO_Millis();
    uint8_t work_id = 0U, work_started = 0U;
    NavButton button;
    (void)parameters;
    Nav_Init(&navigator, &nav_calibration, NAV_CALIBRATED);
    NavButton_Init(&button);
    RouteDebug_Reset();
    ChassisOdom_Read(&previous_pose);
    for (;;) {
        uint8_t vision_owned = VisionAlign_ControlOwned();
        uint8_t request = RouteIO_TakeStartRequest();
        uint8_t stop = RouteIO_TakeStopRequest();
        uint8_t active = (uint8_t)Nav_IsActive(&navigator);
        float dx, dy, wheel[4];
        unsigned i;
        memset(&in, 0, sizeof(in));
        in.now_ms = RouteIO_Millis();
        RouteIO_ReadNavigationLidar(&lidar);
        ChassisOdom_Read(&pose);
        in.enabled = RouteIO_BaseReady() && Check == 0;
        in.imu_ready = pose.valid &&
            (uint32_t)(in.now_ms - pose.tick_ms) <= ROUTE_ODOM_MAX_GAP_MS;
        in.yaw_rad = pose.yaw_rad;
        in.yaw_rate_rad_s = (float)imu.gyro.z * HEADING_CONTROL_GYRO_FULL_SCALE_DPS *
            0.0174532925f / 32768.0f;
        wheel[0] = MOTOR_A.Encoder; wheel[1] = MOTOR_B.Encoder;
        wheel[2] = MOTOR_C.Encoder; wheel[3] = MOTOR_D.Encoder;
        for (i = 0U; i < 4U; ++i)
            if (fabsf(wheel[i]) > in.wheel_max_mps) in.wheel_max_mps = fabsf(wheel[i]);
        {
            uint8_t can_start = navigator.calibrated && in.enabled && in.imu_ready &&
                !vision_owned && !StepperTask_IsBusy() && in.wheel_max_mps <= NAV_STOP_WHEEL_MPS &&
                lidar.sequence && (uint32_t)(in.now_ms-lidar.received_ms) < NAV_LIDAR_TTL_MS &&
                navigator.filter[NAV_FRONT].count == NAV_FILTER_FRAMES;
            unsigned event = NavButton_Update(&button, KEY == 0, active, can_start, in.now_ms);
            if (event == NAV_KEY_START) request = 1U;
            if (event == NAV_KEY_STOP) stop = 1U;
        }
        dx = pose.x_mm - previous_pose.x_mm;
        dy = pose.y_mm - previous_pose.y_mm;
        in.forward_step_mm = cosf(pose.yaw_rad) * dx + sinf(pose.yaw_rad) * dy;
        in.lateral_step_mm = -sinf(pose.yaw_rad) * dx + cosf(pose.yaw_rad) * dy;
        previous_pose = pose;
        if (active && !vision_owned) {
            float effort = (MOTOR_A.Motor_Pwm - MOTOR_B.Motor_Pwm +
                            MOTOR_C.Motor_Pwm - MOTOR_D.Motor_Pwm) * 0.25f;
            if (Move_Y > 0.0f && effort >= NAV_CROSS_STALL_PWM_RATIO * NAV_PWM_LIMIT)
                in.loaded_lateral = 1;
            if (Move_Y < 0.0f && effort <= -NAV_CROSS_STALL_PWM_RATIO * NAV_PWM_LIMIT)
                in.loaded_lateral = -1;
        }
        StepperTask_GetData(&tool);
        if (!work_id) work_id = RouteIO_TakeWorkRequest();
        if (work_started && !StepperTask_IsBusy() &&
            tool.state == STEPPER_RETURNED && tool.mango_id == work_id) {
            RouteIO_CompleteWork();
            work_id = work_started = 0U;
        }
        in.stop = stop;
        in.start = request && !active && !vision_owned && !StepperTask_IsBusy();
        in.pause = vision_owned || work_id || StepperTask_IsBusy();
        in.resume = navigator.pause_latched && !in.pause;
        /* Visual/tool pauses preserve the exit retreat time budget. */
        if (navigator.pause_latched && navigator.target == 17U)
            navigator.exit_started_ms += (uint32_t)(in.now_ms - previous_ms);
        previous_ms = in.now_ms;
        Nav_Tick(&navigator, &in, &lidar);
        active = (uint8_t)Nav_IsActive(&navigator);
        if (!active && work_id && !work_started) {
            work_id = 0U;
            RouteIO_CompleteWork();
        }
        if (work_id && !work_started && navigator.output.work_stopped && !vision_owned &&
            !StepperTask_IsBusy()) {
            if (StepperTask_RequestMango(work_id)) work_started = 1U;
            else { work_id = 0U; RouteIO_CompleteWork(); }
        }
        RouteIO_SetHeading(navigator.heading_origin_rad);
        RouteIO_SetVelocity(navigator.output.vx, navigator.output.vy, navigator.output.quick_stop);
        RouteIO_SetRunning(active);
        RouteIO_AllowWork(active && !vision_owned && !work_id);
        publish_status();
        vTaskDelayUntil(&wake, period);
    }
}
