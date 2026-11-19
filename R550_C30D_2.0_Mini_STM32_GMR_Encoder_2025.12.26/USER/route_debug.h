#ifndef ROUTE_DEBUG_H
#define ROUTE_DEBUG_H

#include "route_nav.h"
#include "camera_protocol.h"
#include "task_priority.h"

/* 诊断保存本次状态机输入和结果；故障后返回第一份故障快照，直到重新启动任务。 */
typedef struct {
    RouteStatus route;
    ChassisOdomFrame odom;
    uint32_t tick_ms, lidar_sequence;
    uint16_t raw_mm[ROUTE_LIDAR_COUNT];
    float command_vx_mps, command_vy_mps, lidar_age_ms;
    uint8_t can_fresh, frozen;
} RouteDebugFrame;

/* Keep vision/wheel channels 0..42; append merged navigation diagnostics. */
#define ROUTE_VOFA_CHANNEL_COUNT 58U
#define ROUTE_VOFA_FRAME_BYTES (ROUTE_VOFA_CHANNEL_COUNT * 4U + 4U)
#define ROUTE_VOFA_TASK_PRIO TASK_PRIORITY_VISION
#define ROUTE_VOFA_TASK_STACK 256U

void RouteDebug_Reset(void);
void RouteDebug_Capture(const RouteController *controller, const RouteInputs *inputs,
                        float command_vx_mps, float command_vy_mps);
uint8_t RouteDebug_Read(RouteDebugFrame *frame);
uint8_t RouteDebug_ReadFault(RouteDebugFrame *frame);
/* 实时相机快照，小端 float 数组 + 00 00 80 7F；参数无效/空间不足返回 0。
 * 未收到视觉帧时值为初始 0；不依赖路线快照，不随路线故障冻结。 */
uint16_t RouteDebug_BuildPacket(uint8_t *bytes, uint16_t capacity);

/* 当前构建是否将蓝牙 USART2 专用于 VOFA；供波特率选择和原协议隔离使用。 */
uint8_t RouteVofa_IsActive(void);
/* 在独立 20 Hz 视觉任务中调用 Poll；USART2 中断调用 TxIRQ，均不等待发送完成。 */
void RouteVofa_Poll(void);
void RouteVofa_Uart2TxIRQ(void);
/* Dedicated visual publisher, separate from logs and actuator diagnostics. */
void RouteVofa_Task(void *parameters);

#endif
