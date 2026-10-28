#ifndef __STEPPER_TASK_H
#define __STEPPER_TASK_H

#include "stepper_motor.h"

#define STEPPER_TASK_PRIO 1
#define STEPPER_STK_SIZE 384

/* 左右机构均以 DIR 低电平靠近芒果、高电平返程，可分别调整方向。 */
#define MANGO_LEFT_FORWARD_HIGH  0U
#define MANGO_RIGHT_FORWARD_HIGH 0U
/* 定时器计数频率 1 MHz：ARR=199 对应目标巡航频率 5000 脉冲/秒。
 * 前进、返程共用对应电机的速度配置，底层保留加减速。 */
#define MANGO_LEFT_SPEED_ARR     199U
#define MANGO_RIGHT_SPEED_ARR    199U
#define MANGO_TARGET_MM          150U      /* 有效距离不大于此值时停止靠近，允许击打。 */
#define MANGO_MAX_PULSES         70000UL   /* 前进脉冲上限，达到上限但未到位时只返程。 */
#define MANGO_RANGE_WAIT_MS      5000UL    /* 首次有效测距最长等待时间。 */
#define MANGO_MOTION_TIMEOUT_MS  420000UL  /* 前进或返程单段运动最长时间。 */

typedef enum {
    STEPPER_WAITING_COMMAND = 0, /* 上电等待串口芒果编号。 */
    STEPPER_WAITING_DISTANCE,    /* 等待选中激光完成初始化并提供有效距离。 */
    STEPPER_MOVING,              /* 对应步进前进，循环判断距离、脉冲上限和超时。 */
    STEPPER_STRIKING,            /* 步进保持停止，等待对应推杆完成 5 秒击打。 */
    STEPPER_RETURNING,           /* 按实际前进脉冲数反向返回，不依赖测距。 */
    STEPPER_FINISHING,           /* 等待激光实际关闭后释放任务占用。 */
    STEPPER_RETURNED,            /* 本轮已结束，可以接受下一个编号。 */
    STEPPER_FAULT                /* 返程失败，原点不确定，锁定至人工检查并复位。 */
} StepperTask_State;

typedef enum {
    MANGO_OK = 0,          /* 正常击打并完成返程。 */
    MANGO_RANGE_TIMEOUT,   /* 首次有效测距等待超时。 */
    MANGO_SENSOR_LOST,     /* 前进中距离无效、过期或通道不符。 */
    MANGO_PULSE_LIMIT,     /* 未到目标距离即达到脉冲上限或检测到前进停止。 */
    MANGO_MOVE_FAILED,     /* 前进启动失败。 */
    MANGO_PUSHROD_FAILED,  /* 推杆请求失败、执行异常或完成等待超时。 */
    MANGO_MOTION_TIMEOUT,  /* 前进超时，停止靠近并尝试返程。 */
    MANGO_RETURN_FAILED    /* 返程启动失败、异常停止或最终脉冲数不符。 */
} Mango_Result;

typedef struct {
    u32 pulses;          /* 当前前进或返程阶段的 STEP 脉冲数，包含细分。 */
    StepperTask_State state;
    u32 outward_pulses; /* 停止前进后保存的最终计数，作为返程脉冲目标。 */
    u8 mango_id;        /* 1/2 对应 left，3/4 对应 right；推杆与激光同编号。 */
    u8 motor;           /* 当前使用的 A4988 电机编号。 */
    Mango_Result result; /* 本轮执行结果，串口 done 时需同时检查此字段。 */
} StepperTask_Data;

/* 任务上下文读取一致的状态快照，供显示等任务使用。 */
void StepperTask_GetData(StepperTask_Data *data);
/* 串口 1 中断提交 1～4 号任务：只锁存请求，不调用 FreeRTOS。
 * 接受返回 1；编号无效或忙碌返回 0。忙碌请求不排队、不切换当前目标。 */
u8 StepperTask_RequestMangoFromISR(u8 mango_id);
/* 独占两路步进控制，协调测距、靠近、击打、返程和收尾。 */
void stepper_Task(void *pvParameters);

#endif
