#ifndef __PUSHROD_TASK_H
#define __PUSHROD_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "pushrod.h"

#define PUSHROD_TASK_PRIO 1
#define PUSHROD_STK_SIZE 128
#define MANGO_STRIKE_MS 5000UL /* 从对应输出实际拉高开始计时，持续 5 秒。 */

typedef enum {
    PUSHROD_TASK_IDLE = 0, /* 无待执行击打，输出保持低。 */
    PUSHROD_TASK_RUNNING,  /* 请求已接收，等待任务启动输出或正在计时。 */
    PUSHROD_TASK_DONE,     /* 击打时间到，且对应输出已成功拉低。 */
    PUSHROD_TASK_ERROR     /* 启动或停止输出失败。 */
} PushrodTask_State;

typedef struct {
    uint8_t id; /* 推杆编号 1～4，对应 PC0～PC3；0 表示无当前推杆。 */
    PushrodTask_State state;
} PushrodTask_Data;

/* 以下接口仅供任务上下文使用；Pushrod_Init() 已在板级启动早期调用。
 * Start 只提交一次击打请求，执行中不接受新请求；接受返回 1，否则返回 0。 */
uint8_t PushrodTask_Start(uint8_t id);
/* 清除待执行请求，并将四路输出全部拉低。 */
void PushrodTask_Cancel(void);
/* 读取一致的执行快照，供协调任务确认本次击打是否完成。 */
void PushrodTask_GetData(PushrodTask_Data *data);
void pushrod_task(void *pvParameters);

#endif
