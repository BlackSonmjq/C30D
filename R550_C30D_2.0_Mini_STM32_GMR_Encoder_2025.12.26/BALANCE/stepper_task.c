#include "stepper_task.h"
#include "distance_task.h"
#include "pushrod_task.h"
#include "usart.h"

/* 联调主流程：选中激光 -> 等有效距离 -> 靠近至 150 mm -> 推杆击打 5 秒
 * -> 按已输出的前进脉冲数返程 -> 关闭激光 -> 等待下一条串口命令。
 * 一次只处理一个芒果；STEP 计数为指令脉冲，不能检测机械失步或打滑。 */
#define MANGO_CHECK_MS 10U

/* 串口中断优先级为 3，高于 FreeRTOS 临界区的屏蔽阈值 5。
 * 因此任务访问以下中断共享变量时，需用短暂的 PRIMASK 全局关中断保护。 */
static volatile u8 job_busy = 0;
static volatile u8 pending_mango = 0;
static volatile u8 rejected_command = 0;
static StepperTask_Data motion = {0, STEPPER_WAITING_COMMAND, 0, 0, 0, MANGO_OK};

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
static void begin_return(StepperTask_Data *job, TickType_t *phase_started)
{
    s32 signed_pulses;
    PushrodTask_Cancel();
    StepperMotor_Disable(job->motor);
    job->outward_pulses = StepperMotor_GetCompletedPulses(job->motor);
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
        any_printf(USART1, "mango %u fault return failed\r\n", (unsigned int)job->mango_id);
        return; /* 原点不确定，保留忙碌锁定，人工检查原点并复位后再运行。 */
    }
    *phase_started = xTaskGetTickCount();
    job->state = STEPPER_RETURNING;
    any_printf(USART1, "mango %u returning %lu\r\n", (unsigned int)job->mango_id,
               (unsigned long)job->outward_pulses);
}

static void begin_strike(StepperTask_Data *job, TickType_t *phase_started)
{
    /* 必须先停步进并保存最终计数，再请求与芒果同编号的推杆。 */
    StepperMotor_Disable(job->motor);
    job->outward_pulses = StepperMotor_GetCompletedPulses(job->motor);
    job->pulses = job->outward_pulses;
    if (!PushrodTask_Start(job->mango_id)) {
        job->result = MANGO_PUSHROD_FAILED;
        begin_return(job, phase_started);
        return;
    }
    *phase_started = xTaskGetTickCount();
    job->state = STEPPER_STRIKING;
    any_printf(USART1, "mango %u striking out=%lu\r\n", (unsigned int)job->mango_id,
               (unsigned long)job->outward_pulses);
}

void stepper_Task(void *pvParameters)
{
    DistanceTask_Data distance;
    PushrodTask_Data pushrod;
    StepperTask_Data job = {0, STEPPER_WAITING_COMMAND, 0, 0, 0, MANGO_OK};
    TickType_t phase_started = 0, now;
    u32 primask;
    u8 selected, rejected;
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
        selected = pending_mango;
        pending_mango = 0;
        rejected = rejected_command;
        rejected_command = 0;
        __set_PRIMASK(primask);
        if (rejected) any_printf(USART1, "mango busy\r\n");

        if (selected != 0U) {
            job.mango_id = selected;
            job.motor = selected <= 2U ? A4988_MOTOR_Left : A4988_MOTOR_Right;
            job.pulses = 0;
            job.outward_pulses = 0;
            job.result = MANGO_OK;
            /* 每轮都清零旧计数，避免本轮无需前进时仍按上轮步数返程。 */
            StepperMotor_ResetCompletedPulses(job.motor);
            PushrodTask_Cancel();
            DistanceTask_SelectLaser(selected);
            phase_started = xTaskGetTickCount();
            job.state = STEPPER_WAITING_DISTANCE;
            any_printf(USART1, "mango %u start motor=%s\r\n", (unsigned int)selected,
                       job.motor == A4988_MOTOR_Left ? "left" : "right");
        }

        DistanceTask_GetData(&distance);
        now = xTaskGetTickCount();
        switch (job.state) {
        case STEPPER_WAITING_DISTANCE:
            /* 只使用当前选中激光的有效数据；未接入模块最多等待 5 秒。 */
            if (distance.valid && distance.laser_id == job.mango_id) {
                if (distance.distance_mm <= MANGO_TARGET_MM) {
                    begin_strike(&job, &phase_started);
                } else {
                    signed_pulses = forward_high(job.motor) ?
                        (s32)MANGO_MAX_PULSES : -(s32)MANGO_MAX_PULSES;
                    if (StepperMotor_StartMove(job.motor, signed_pulses)) {
                        job.state = STEPPER_MOVING;
                        phase_started = now;
                        any_printf(USART1, "mango %u approaching\r\n", (unsigned int)job.mango_id);
                    } else {
                        job.result = MANGO_MOVE_FAILED;
                        begin_return(&job, &phase_started);
                    }
                }
            } else if ((TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_RANGE_WAIT_MS)) {
                job.result = MANGO_RANGE_TIMEOUT;
                begin_return(&job, &phase_started);
            }
            break;

        case STEPPER_MOVING:
            /* 距离失效先停车返程；仅有效距离达到阈值时允许进入击打。 */
            job.pulses = StepperMotor_GetCompletedPulses(job.motor);
            if (!distance.valid || distance.laser_id != job.mango_id) {
                job.result = MANGO_SENSOR_LOST;
                begin_return(&job, &phase_started);
            } else if (distance.distance_mm <= MANGO_TARGET_MM) {
                begin_strike(&job, &phase_started);
            } else if (job.pulses >= MANGO_MAX_PULSES || !StepperMotor_IsRunning(job.motor)) {
                job.result = MANGO_PULSE_LIMIT;
                begin_return(&job, &phase_started);
            } else if ((TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_MOTION_TIMEOUT_MS)) {
                job.result = MANGO_MOTION_TIMEOUT;
                begin_return(&job, &phase_started);
            }
            break;

        case STEPPER_STRIKING:
            /* 推杆任务实际拉高输出后计时，确认输出拉低并发布 DONE 才返程。
             * 额外 1 秒只用于检测任务执行异常，不延长正常击打时间。 */
            PushrodTask_GetData(&pushrod);
            if (pushrod.id == job.mango_id && pushrod.state == PUSHROD_TASK_DONE) {
                begin_return(&job, &phase_started);
            } else if (pushrod.id != job.mango_id || pushrod.state == PUSHROD_TASK_ERROR ||
                       (TickType_t)(now - phase_started) >= pdMS_TO_TICKS(MANGO_STRIKE_MS + 1000UL)) {
                job.result = MANGO_PUSHROD_FAILED;
                begin_return(&job, &phase_started);
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
