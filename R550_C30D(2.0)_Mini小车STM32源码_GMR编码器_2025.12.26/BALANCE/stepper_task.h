#ifndef __STEPPER_TASK_H
#define __STEPPER_TASK_H

#include "stepper_motor.h"

#define STEPPER_TASK_PRIO 1
#define STEPPER_STK_SIZE 256

void stepper_Task(void *pvParameters);

#endif /* __STEPPER_TASK_H */
