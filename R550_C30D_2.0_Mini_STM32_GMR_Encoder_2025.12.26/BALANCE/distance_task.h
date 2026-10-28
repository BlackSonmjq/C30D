#ifndef DISTANCE_TASK_H
#define DISTANCE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "vl53l0x.h"

#define DISTANCE_TASK_PRIO 1
#define DISTANCE_STK_SIZE 256
#define DISTANCE_OLED_PAGE 2U

typedef struct {
    uint16_t distance_mm;
    uint8_t valid;
    VL53L0X_Status status;
    TickType_t update_tick;
    uint8_t laser_id; /* 0=both off, 1=PA2 high, 2=PA3 high. */
} DistanceTask_Data;

void distance_task(void *pvParameters);
/* Board startup, before USART interrupts/scheduler: both XSHUT pins low. */
void DistanceTask_InitShutdownPins(void);
/* USART1 ISR: latch selection 1 (PA2) or 2 (PA3); no I2C/RTOS calls. */
void DistanceTask_SelectLaserFromISR(uint8_t laser_id);
/* Thread-safe snapshot. Use distance_mm only when valid != 0.
 * Readings older than 500ms are returned as invalid/timeout. */
void DistanceTask_GetData(DistanceTask_Data *data);
/* Framebuffer drawing only; called and refreshed by show_task. */
void DistanceTask_DrawOLED(void);

#endif
