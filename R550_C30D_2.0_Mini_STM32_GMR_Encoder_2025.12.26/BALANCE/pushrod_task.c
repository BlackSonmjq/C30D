#include "pushrod_task.h"

static PushrodTask_Data actuator = {0, PUSHROD_TASK_IDLE};
static uint8_t start_pending = 0; /* 请求锁存，GPIO 操作和计时由推杆任务执行。 */

uint8_t PushrodTask_Start(uint8_t id)
{
    uint8_t accepted = 0;
    if (id < 1U || id > PUSHROD_COUNT) return 0;
    taskENTER_CRITICAL();
    if (actuator.state != PUSHROD_TASK_RUNNING) {
        /* 提交时即标为运行中，防止任务尚未执行时被另一请求覆盖。 */
        actuator.id = id;
        actuator.state = PUSHROD_TASK_RUNNING;
        start_pending = 1;
        accepted = 1;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void PushrodTask_Cancel(void)
{
    uint8_t id;
    taskENTER_CRITICAL();
    start_pending = 0;
    /* 同时撤销尚未启动的请求，避免返程后又被推杆任务执行。 */
    for (id = 1U; id <= PUSHROD_COUNT; ++id) Pushrod_Stop(id);
    actuator.id = 0;
    actuator.state = PUSHROD_TASK_IDLE;
    taskEXIT_CRITICAL();
}

void PushrodTask_GetData(PushrodTask_Data *data)
{
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = actuator;
    taskEXIT_CRITICAL();
}

void pushrod_task(void *pvParameters)
{
    (void)pvParameters;
    for (;;) {
        taskENTER_CRITICAL();
        if (start_pending) {
            start_pending = 0;
            /* TIM6 到时直接关闭输出，不依赖本任务下一次获得调度。 */
            if (Pushrod_StartTimed(actuator.id, MANGO_STRIKE_MS) != PUSHROD_OK)
                actuator.state = PUSHROD_TASK_ERROR;
        }
        if (actuator.state == PUSHROD_TASK_RUNNING &&
            Pushrod_TimedDone(actuator.id)) {
            /* 中断已确认输出拉低；任务只发布完成状态。 */
            actuator.state = PUSHROD_TASK_DONE;
        }
        taskEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
