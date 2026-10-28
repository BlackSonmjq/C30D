#ifndef __STEPPER_TASK_H
#define __STEPPER_TASK_H

#include "stepper_motor.h"

#define STEPPER_TASK_PRIO 1
#define STEPPER_STK_SIZE 256

typedef enum {
    STEPPER_WAITING_DISTANCE = 0,
    STEPPER_MOVING,
    STEPPER_TARGET_REACHED,
    STEPPER_SENSOR_STOP,
    STEPPER_WAITING_COMMAND,
    STEPPER_RETURNING,
    STEPPER_RETURNED,
    STEPPER_RETURN_STOP,
    STEPPER_LIMIT_REACHED
} StepperTask_State;

typedef struct {
    u32 pulses; /* STEP pulses, including microsteps; not measured shaft steps. */
    StepperTask_State state;
} StepperTask_Data;

/* Task-context snapshot; final count is retained after target/fault stop. */
void StepperTask_GetData(StepperTask_Data *data);
/* USART1 ISR only: latch ASCII start command, no FreeRTOS API calls. */
void StepperTask_RequestStartFromISR(void);
void StepperTask_RequestReturnFromISR(void);
/* ISR-safe status check; pulse-limit/fault stops are not position ready. */
u8 StepperTask_IsPositionReadyFromISR(void);
void stepper_Task(void *pvParameters);

#endif
