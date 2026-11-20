#ifndef __SERVO_TASK_H
#define __SERVO_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"

#include "task_priority.h"
#define SERVO_TASK_PRIO TASK_PRIORITY_SERVO
#define SERVO_STK_SIZE 256
#define MANGO_SERVO_PHASE_TIMEOUT_MS 3000UL
#define MANGO_SERVO_QUERY_MS 150UL
#define MANGO_STRIKE_MS (3UL * MANGO_SERVO_PHASE_TIMEOUT_MS)

typedef enum {
    SERVO_TASK_IDLE = 0,
    SERVO_TASK_RUNNING,
    SERVO_TASK_DONE,
    SERVO_TASK_ERROR
} ServoTask_State;

typedef struct {
    uint8_t id;
    ServoTask_State state;
} ServoTask_Data;

void ServoTask_InitRest(void);
uint16_t ServoTask_GetRetryCount(void);
void ServoTask_ResetRetryCount(void);
/* ISR-safe: reject all jobs during startup and failed IDs after startup. */
uint8_t ServoTask_CanStrike(uint8_t id);
uint8_t ServoTask_Start(uint8_t id);
/* Single absolute move, using the same feedback and timeout as a strike. */
uint8_t ServoTask_MoveTo(uint8_t id, uint16_t pulse);
uint8_t ServoTask_IsBusy(void);
void ServoTask_Cancel(void);
void ServoTask_GetData(ServoTask_Data *data);
void servo_task(void *pvParameters);

#endif
