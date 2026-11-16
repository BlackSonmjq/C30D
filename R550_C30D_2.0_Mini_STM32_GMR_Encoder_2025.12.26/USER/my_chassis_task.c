#include "my_chassis_task.h"
#include "route_config.h"
#include "route_io.h"
#include "chassis_odometry.h"
#include "route_debug.h"
#include "stepper_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile uint8_t Route_Stage;
static RouteController navigator;
static RouteStatus published_status;

void Route_ReadStatus(RouteStatus *status)
{
    taskENTER_CRITICAL();
    *status = published_status;
    taskEXIT_CRITICAL();
}

static void publish_status(void)
{
    taskENTER_CRITICAL();
    published_status = navigator.status;
    Route_Stage = navigator.status.phase == ROUTE_DONE ? ROUTE_POINT_COUNT + 1U :
                  navigator.status.target_point;
    taskEXIT_CRITICAL();
}

void my_chassis_task(void *parameters)
{
    const TickType_t period = pdMS_TO_TICKS(ROUTE_PERIOD_MS);
    TickType_t wake = xTaskGetTickCount();
    RouteInputs in;
    RouteOutput out;
    float body_vx, body_vy;
    StepperTask_Data tool;
    (void)parameters;
    RouteNav_Init(&navigator);
    RouteDebug_Reset();

    for (;;) {
        in.now_ms = (uint32_t)xTaskGetTickCount();
        RouteIO_ReadLidar(&in.lidar);
        ChassisOdom_Read(&in.odom);
        StepperTask_GetData(&tool);
        in.ready = RouteIO_BaseReady();
        in.tool_busy = StepperTask_IsBusy();
        in.tool_result = (uint8_t)tool.result;
        in.tool_fault = tool.state == STEPPER_FAULT;
        in.chassis_stopped = RouteIO_ChassisStopped();
        in.work_request = 0U;

        if (ROUTE_REVERSE_TEST_ENABLE) {
            RouteIODiagnostics test_command;
            RouteIO_ReadDiagnostics(&test_command);
            navigator.status.target_point = 0U;
            navigator.status.phase = RouteIO_ReverseTestRunning() ? ROUTE_ODOM_RUN : ROUTE_WAIT_START;
            navigator.status.motion = RouteIO_ReverseTestRunning() ? ROUTE_BACK : ROUTE_STOP;
            navigator.status.reason = RouteIO_Ready() ? ROUTE_ERROR_NONE : ROUTE_ERROR_READY;
            navigator.status.elapsed_ms = in.now_ms;
            RouteDebug_Capture(&navigator, &in, test_command.vx_mps, 0.0f);
            publish_status();
            vTaskDelayUntil(&wake, period);
            continue;
        }

        if (!RouteIO_StartRequested()) {
            navigator.status.reason = !RoutePlan_Calibrated() ? ROUTE_ERROR_CALIB :
                in.tool_fault ? ROUTE_ERROR_TOOL :
                !in.ready || in.tool_busy || !in.chassis_stopped ? ROUTE_ERROR_READY :
                !in.odom.valid ? ROUTE_ERROR_ODOM :
                !in.lidar.fresh ? ROUTE_ERROR_LIDAR : ROUTE_ERROR_NONE;
            /* 正式模式由开关上电后完成校准、检查并自主起跑；调试由 PE0 请求。 */
            if (!ROUTE_DEBUG_MODE && navigator.status.reason == ROUTE_ERROR_NONE)
                RouteIO_RequestStart();
        }
        if (RouteIO_StartRequested() && !navigator.started) {
            ChassisOdom_Read(&in.odom);
            RouteNav_Start(&navigator, &in);
        }
        /* 底盘暂时失能时保留已锁存的作业请求，恢复后再交给状态机。 */
        if (navigator.started && navigator.status.phase != ROUTE_DONE &&
            navigator.status.phase != ROUTE_FAULT && in.ready && in.odom.valid &&
            (uint32_t)(in.now_ms - in.odom.tick_ms) <= ROUTE_ODOM_MAX_GAP_MS)
            in.work_request = RouteIO_TakeWorkRequest();

        RouteNav_Step(&navigator, &in, &out);
        if (out.start_work != 0U && !StepperTask_RequestMango(out.start_work)) {
            RouteNav_Fail(&navigator, ROUTE_ERROR_TOOL);
            RouteIO_CompleteWork();
        }
        if (out.work_finished) RouteIO_CompleteWork();
        if (out.correct_axis < 2U)
            ChassisOdom_CorrectAxis(out.correct_axis, out.correct_position_mm);

        /* 导航速度沿起跑时的地图轴，按连续 IMU 航向转换为当前车体速度。 */
        body_vx = cosf(in.odom.yaw_rad) * out.vx_mps + sinf(in.odom.yaw_rad) * out.vy_mps;
        body_vy = -sinf(in.odom.yaw_rad) * out.vx_mps + cosf(in.odom.yaw_rad) * out.vy_mps;
        RouteIO_SetVelocity(body_vx, body_vy);
        RouteDebug_Capture(&navigator, &in, body_vx, body_vy);
        RouteIO_AllowWork(navigator.started &&
                          (navigator.status.phase == ROUTE_ODOM_RUN ||
                           navigator.status.phase == ROUTE_LASER_NEAR ||
                           navigator.status.phase == ROUTE_CAPTURE ||
                           navigator.status.phase == ROUTE_RECOVER ||
                           navigator.status.phase == ROUTE_CONFIRM ||
                           navigator.status.phase == ROUTE_WAIT_LIDAR));
        publish_status();
        vTaskDelayUntil(&wake, period);
    }
}
