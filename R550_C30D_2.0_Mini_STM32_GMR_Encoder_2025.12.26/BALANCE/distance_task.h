#ifndef DISTANCE_TASK_H
#define DISTANCE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "vl53l0x.h"

#define DISTANCE_TASK_PRIO 1
#define DISTANCE_STK_SIZE 256
#define DISTANCE_OLED_PAGE 2U

typedef struct {
    uint16_t distance_mm; /* 距离单位为毫米，只有 valid 非零时才能使用。 */
    uint8_t valid;        /* 有效性包含通信状态、通道匹配和数据时效。 */
    VL53L0X_Status status;
    TickType_t update_tick; /* 最近一次发布数据的系统节拍。 */
    uint8_t laser_id; /* 0=全关；1/2/3/4 分别对应 PA2/PA3/PA6/PA7。 */
} DistanceTask_Data;

void distance_task(void *pvParameters);
/* 串口中断和调度器启动前，将四路 XSHUT 全部配置为低电平输出。 */
void DistanceTask_InitShutdownPins(void);
/* 仅由协调任务调用：0=全关，1～4=选中设备；这里只锁存请求，不操作 I2C。 */
void DistanceTask_SelectLaser(uint8_t laser_id);
/* 读取线程安全快照，仅当 valid 非零时使用 distance_mm。
 * 通道已切换或数据达到 500 ms 未更新时，返回无效状态。 */
void DistanceTask_GetData(DistanceTask_Data *data);
/* 只绘制帧缓冲，由 show_task 统一调用和刷新 OLED。 */
void DistanceTask_DrawOLED(void);

#endif
