/* Integration: task diagnostics use USART1. USART3 is the camera data link. */
#include "stepper_task.h"
#include "distance_task.h"
#include "servo_task.h"
#include "usart.h"

/* 联调主流程：选中激光 -> 等有效距离 -> 靠近至小于对应编号阈值 -> 舵机正转 180 度后回零
 * -> 按已输出的前进脉冲数返程 -> 关闭激光 -> 等待下一条串口命令。
 * 一次只处理一个芒果；STEP 计数为指令脉冲，不能检测机械失步或打滑。 */
#define MANGO_CHECK_MS 10U

/* 串口中断优先级为 3，高于 FreeRTOS 临界区的屏蔽阈值 5。
 * 因此任务访问以下中断共享变量时，需用短暂的 PRIMASK 全局关中断保护。 */
static volatile u8 job_busy = 0;
static volatile u8 pending_mango = 0;
static volatile u8 pending_laser = 0;
static volatile u8 rejected_laser = 0;
static volatile u8 rejected_command = 0;
static StepperTask_Data motion = {0, STEPPER_WAITING_COMMAND, 0, 0, 0, MANGO_OK};

/* Manual selection is applied by the same task that owns picking selection. */
u8 StepperTask_RequestLaserFromISR(u8 laser_id)
{
    if (laser_id < 1U || laser_id > 4U) return 0;
    if (job_busy) {
        rejected_laser = laser_id;
        return 0;
    }
    pending_laser = laser_id;
    return 1;
}

u8 StepperTask_RequestMangoFromISR(u8 mango_id)
{
    if (mango_id < 1U || mango_id > 4U) return 0;
    if (job_busy) {
        rejected_command = 1;
        return 0;
    }
    /* 收到首个编号立即占用，防止任务尚未调度时被后续编号覆盖。 */
    job_busy = 1;
    pending_mango = mango_id;
    return 1;
}

/* Includes accepted ISR requests, cleanup and latched return faults. */
u8 StepperTask_IsBusy(void)
{
    return job_busy;
}

void StepperTask_GetData(StepperTask_Data *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = motion;
    taskEXIT_CRITICAL();
}

static void publish(const StepperTask_Data *data)
{
    taskENTER_CRITICAL();
    motion = *data;
    taskEXIT_CRITICAL();
}

static u8 forward_high(u8 motor)
{
    return motor == A4988_MOTOR_Left ? MANGO_LEFT_FORWARD_HIGH : MANGO_RIGHT_FORWARD_HIGH;
}

static u16 target_mm(u8 mango_id)
{
    static const u16 targets[4] = {
        MANGO_TARGET_1_MM, MANGO_TARGET_2_MM, MANGO_TARGET_3_MM, MANGO_TARGET_4_MM
    };
    return mango_id >= 1U && mango_id <= 4U ? targets[mango_id - 1U] : 0U;
}

static void finish_job(StepperTask_Data *job)
{
    u32 primask;
    job->state = STEPPER_RETURNED;
    publish(job);
    any_printf(USART1, "mango %u done result=%u out=%lu back=%lu\r\n",
               (unsigned int)job->mango_id, (unsigned int)job->result,
               (unsigned long)job->outward_pulses, (unsigned long)job->pulses);
    /* 完成日志发送后再允许新请求，保持串口反馈和任务接收顺序一致。 */
    primask = __get_PRIMASK();
    __disable_irq();
    job_busy = 0;
    __set_PRIMASK(primask);
}

/* 先停止 STEP，再读取包含最后一个上升沿的计数，反向走相同脉冲数。
 * 前进异常时直接进入此流程，不执行击打；返程不再依赖激光距离。 */
static void begin_return(StepperTask_Data *job, TickType_t *phase_started, u32 forward_base)
{
    s32 signed_pulses;
    ServoTask_Cancel();
    StepperMotor_Disable(job->motor);
    job->outward_pulses = forward_base + StepperMotor_GetCompletedPulses(job->motor);
    job->pulses = 0;
    if (job->outward_pulses == 0U) {
        /* 初始已到位或测距未成功启动时，无需发返程脉冲。 */
        DistanceTask_SelectLaser(0);
        job->state = STEPPER_FINISHING;
        return;
    }
    signed_pulses = (s32)job->outward_pulses;
    if (forward_high(job->motor)) signed_pulses = -signed_pulses;
    if (!StepperMotor_StartMove(job->motor, signed_pulses)) {
        DistanceTask_SelectLaser(0);
        job->result = MANGO_RETURN_FAILED;
        job->state = STEPPER_FAULT;
        any_printf(USART1, "mango %u fault return failed dir=%u\r\n",
                   (unsigned int)job->mango_id, (unsigned int)(signed_pulses > 0));
        return; /* 原点不确定，保留忙碌锁定，人工检查原点并复位后再运行。 */
    }
    *phase_started = xTaskGetTickCount();
    job->state = STEPPER_RETURNING;
    any_printf(USART1, "mango %u returning %lu dir=%u pulses=%ld\r\n",
               (unsigned int)job->mango_id, (unsigned long)job->outward_pulses,
               (unsigned int)(signed_pulses > 0), (long)signed_pulses);
}

static void begin_strike(StepperTask_Data *job, TickType_t *phase_started, u32 forward_base)
{
    /* 必须先停步进并保存最终计数，再请求与芒果同编号的舵机。 */
    StepperMotor_Disable(job->motor);
    job->outward_pulses = forward_base + StepperMotor_GetCompletedPulses(job->motor);
    job->pulses = job->outward_pulses;
    if (!ServoTask_Start(job->mango_id)) {
        job->result = MANGO_PUSHROD_FAILED;
        begin_return(job, phase_started, forward_base);
        return;
    }
    *phase_started = xTaskGetTickCount();
    job->state = STEPPER_STRIKING;
    any_printf(USART1, "mango %u striking out=%lu\r\n", (unsigned int)job->mango_id,
               (unsigned long)job->outward_pulses);
}

/* Stream each new sample from the manually selected laser; never repeat cached data. */
typedef struct {
    u8 id;
    TickType_t started;
    TickType_t previous_update;
} ManualLaserReport;

static void report_manual_laser(ManualLaserReport *report, const DistanceTask_Data *data,
                                TickType_t now)
{
    const char *error;
    if (!report->id) return;
    if (data->laser_id == report->id && data->update_tick != report->previous_update &&
        data->status != VL53L0X_NOT_READY) {
        if (data->valid) {
            any_printf(USART1, "CMD %u: laser=%u distance=%u mm\r\n",
                       (unsigned)report->id, (unsigned)report->id, (unsigned)data->distance_mm);
        } else {
            switch (data->status) {
            case VL53L0X_ERROR_I2C: error = "I2C communication failed"; break;
            case VL53L0X_ERROR_TIMEOUT: error = "measurement timeout"; break;
            case VL53L0X_ERROR_ID: error = "sensor ID mismatch"; break;
            case VL53L0X_ERROR_RANGE: error = "invalid range"; break;
            case VL53L0X_ERROR_CONFIG: error = "sensor configuration failed"; break;
            default: error = "sensor not ready"; break;
            }
            any_printf(USART1, "CMD %u: laser=%u ERROR %s\r\n",
                       (unsigned)report->id, (unsigned)report->id, error);
        }
        report->previous_update = data->update_tick;
        report->started = now;
    } else if ((TickType_t)(now - report->started) >= pdMS_TO_TICKS(5000U)) {
        any_printf(USART1, "CMD %u: laser=%u ERROR no new measurement within 5000 ms\r\n",
                   (unsigned)report->id, (unsigned)report->id);
        report->started = now; /* Keep waiting and report again at most every 5 seconds. */
    }
}

void stepper_Task(void *pvParameters)
{
    DistanceTask_Data distance;
    ServoTask_Data servo;
    StepperTask_Data job = {0, STEPPER_WAITING_COMMAND, 0, 0, 0, MANGO_OK};
    TickType_t phase_started = 0, range_wait_started = 0, now;
    u32 forward_base = 0;
    ManualLaserReport laser_report = {0, 0, 0};
    u32 primask;
    u8 selected, rejected, laser, laser_busy;
    s32 signed_pulses;
    (void)pvParameters;

    StepperMotor_Disable(A4988_MOTOR_Left);
    StepperMotor_Disable(A4988_MOTOR_Right);
    StepperMotor_SetSpeed(A4988_MOTOR_Left, MANGO_LEFT_SPEED_ARR);
    StepperMotor_SetSpeed(A4988_MOTOR_Right, MANGO_RIGHT_SPEED_ARR);

    for (;;) {
        /* 原子取走串口请求；同一轮收到的忙碌请求合并为一次提示。 */
        primask = __get_PRIMASK();
        __disable_irq();
        laser = pending_laser;
        pending_laser = 0;
        laser_busy = rejected_laser;
        rejected_laser = 0;
        /* A pending picking request takes precedence over manual selection. */
        if (laser && !job_busy) DistanceTask_SelectLaser(laser);
        else if (laser) { laser_busy = laser; laser = 0; }
        selected = pending_mango;
        pending_mango = 0;
        rejected = rejected_command;
        rejected_command = 0;
        __set_PRIMASK(primask);
        if (rejected) any_printf(USART1, "mango busy\r\n");
        if (laser_busy) any_printf(USART1, "CMD %u: laser busy\r\n", (unsigned)laser_busy);
        if ((laser || selected) && laser_report.id) {
            any_printf(USART1, "CMD %u: measurement cancelled by new command\r\n", (unsigned)laser_report.id);
            laser_report.id = 0;
        }
        if (laser) {
            DistanceTask_GetData(&distance);
            laser_report.id = laser;
            laser_report.started = xTaskGetTickCount();
            laser_report.previous_update = distance.update_tick;
        }

        if (selected != 0U) {
            job.mango_id = selected;
            job.motor = selected <= 2U ? A4988_MOTOR_Left : A4988_MOTOR_Right;
            job.pulses = 0;
            job.outward_pulses = 0;
            job.result = MANGO_OK;
            forward_base = 0;
            /* 每轮都清零旧计数，避免本轮无需前进时仍按上轮步数返程。 */
            StepperMotor_ResetCompletedPulses(job.motor);
            ServoTask_Cancel();
            DistanceTask_SelectLaser(selected);
            phase_started = xTaskGetTickCount();
            job.state = STEPPER_WAITING_DISTANCE;
            any_printf(USART1, "mango %u start motor=%s\r\n", (unsigned int)selected,
                       job.motor == A4988_MOTOR_Left ? "left" : "right");
        }

        DistanceTask_GetData(&distance);
        now = xTaskGetTickCount();
        report_manual_laser(&laser_report, &distance, now);
        switch (job.state) {
        case STEPPER_WAITING_DISTANCE:
            /* 只使用当前选中激光的有效数据；未接入模块最多等待 5 秒。 */
            if (distance.valid && distance.laser_id == job.mango_id) {
                if (distance.distance_mm < target_mm(job.mango_id)) {
                    begin_strike(&job, &phase_started, forward_base);
                } else {
                    signed_pulses = forward_high(job.motor) ?
                        (s32)MANGO_MAX_PULSES : -(s32)MANGO_MAX_PULSES;
                    if (StepperMotor_StartMove(job.motor, signed_pulses)) {
                        job.state = STEPPER_MOVING;
                        phase_started = now;
                        any_printf(USART1, "mango %u approaching dir=%u pulses=%ld\r\n",
                                   (unsigned int)job.mango_id,
                                   (unsigned int)(signed_pulses > 0), (long)signed_pulses);
                    } else {
                        job.result = MANGO_MOVE_FAILED;
                        begin_return(&job, &phase_started, forward_base);
                    }
                }
            } else if ((TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_RANGE_WAIT_MS)) {
                job.result = MANGO_RANGE_TIMEOUT;
                begin_return(&job, &phase_started, forward_base);
            }
            break;

        case STEPPER_MOVING:
            job.pulses = forward_base + StepperMotor_GetCompletedPulses(job.motor);
            if (job.pulses >= MANGO_MAX_PULSES || !StepperMotor_IsRunning(job.motor)) {
                job.result = MANGO_PULSE_LIMIT;
                begin_return(&job, &phase_started, forward_base);
            } else if ((TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_MOTION_TIMEOUT_MS)) {
                job.result = MANGO_MOTION_TIMEOUT;
                begin_return(&job, &phase_started, forward_base);
            } else if (!distance.valid || distance.laser_id != job.mango_id) {
                StepperMotor_Disable(job.motor);
                job.pulses = forward_base + StepperMotor_GetCompletedPulses(job.motor);
                range_wait_started = now;
                job.state = STEPPER_RECHECKING_DISTANCE;
                any_printf(USART1, "mango %u range pause status=%u laser=%u pulses=%lu\r\n",
                    (unsigned)job.mango_id, (unsigned)distance.status,
                    (unsigned)distance.laser_id, (unsigned long)job.pulses);
            } else if (distance.distance_mm < target_mm(job.mango_id)) {
                begin_strike(&job, &phase_started, forward_base);
            }
            break;

        case STEPPER_RECHECKING_DISTANCE:
            /* Never drive using invalid data; the overall motion clock keeps running. */
            if (job.pulses >= MANGO_MAX_PULSES) {
                job.result = MANGO_PULSE_LIMIT;
                begin_return(&job, &phase_started, forward_base);
            } else if ((TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_MOTION_TIMEOUT_MS)) {
                job.result = MANGO_MOTION_TIMEOUT;
                begin_return(&job, &phase_started, forward_base);
            } else if ((TickType_t)(now - range_wait_started) >= pdMS_TO_TICKS(MANGO_RANGE_RECHECK_MS)) {
                job.result = MANGO_SENSOR_LOST;
                begin_return(&job, &phase_started, forward_base);
            } else if (distance.valid && distance.laser_id == job.mango_id) {
                if (distance.distance_mm < target_mm(job.mango_id)) {
                    begin_strike(&job, &phase_started, forward_base);
                } else {
                    signed_pulses = (s32)(MANGO_MAX_PULSES - job.pulses);
                    if (!forward_high(job.motor)) signed_pulses = -signed_pulses;
                    if (StepperMotor_StartMove(job.motor, signed_pulses)) {
                        forward_base = job.pulses;
                        job.state = STEPPER_MOVING;
                        any_printf(USART1, "mango %u range recovered total=%lu remaining=%ld\r\n",
                            (unsigned)job.mango_id, (unsigned long)forward_base,
                            (long)signed_pulses);
                    } else {
                        job.result = MANGO_MOVE_FAILED;
                        begin_return(&job, &phase_started, forward_base);
                    }
                }
            }
            break;

        case STEPPER_STRIKING:
            /* 舵机任务确认正转 180 度并回零、发布 DONE 后才返程。
             * 额外 1 秒只用于检测任务执行异常，不延长正常舵机动作。 */
            ServoTask_GetData(&servo);
            if (servo.id == job.mango_id && servo.state == SERVO_TASK_DONE) {
                begin_return(&job, &phase_started, forward_base);
            } else if (servo.id != job.mango_id || servo.state == SERVO_TASK_ERROR ||
                       (TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_STRIKE_MS + 1000UL)) {
                job.result = MANGO_PUSHROD_FAILED;
                begin_return(&job, &phase_started, forward_base);
            }
            break;

        case STEPPER_RETURNING:
            /* 定时器负责限制返程脉冲数；停止后再次核对最终计数。 */
            job.pulses = StepperMotor_GetCompletedPulses(job.motor);
            if (!StepperMotor_IsRunning(job.motor) ||
                (TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_MOTION_TIMEOUT_MS)) {
                StepperMotor_Disable(job.motor);
                job.pulses = StepperMotor_GetCompletedPulses(job.motor);
                DistanceTask_SelectLaser(0);
                if (job.pulses == job.outward_pulses) {
                    job.state = STEPPER_FINISHING;
                } else {
                    job.result = MANGO_RETURN_FAILED;
                    job.state = STEPPER_FAULT;
                    any_printf(USART1, "mango %u fault out=%lu back=%lu\r\n",
                               (unsigned int)job.mango_id, (unsigned long)job.outward_pulses,
                               (unsigned long)job.pulses);
                }
            }
            break;

        case STEPPER_FINISHING:
            /* 确认激光实际关闭后才接受新任务，防止重复选择同一编号时
             * 复用上一轮的有效距离，跳过新一轮初始化和测距。 */
            if (distance.laser_id == 0U) finish_job(&job);
            break;

        default:
            break;
        }
        publish(&job);
        vTaskDelay(pdMS_TO_TICKS(MANGO_CHECK_MS));
    }
}

u8 StepperTask_RequestMango(u8 mango_id)
{
    u8 result;
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    result=StepperTask_RequestMangoFromISR(mango_id);
    __set_PRIMASK(mask);
    return result;
}
