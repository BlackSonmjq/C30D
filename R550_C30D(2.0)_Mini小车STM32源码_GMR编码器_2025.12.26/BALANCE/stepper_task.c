#include "stepper_task.h"
#include "FreeRTOS.h"
#include "task.h"

void stepper_Task(void *pvParameters)
{
    (void)pvParameters;
    StepperMotor_Init();
    /* A FreeRTOS task must not return. Initialization is one-shot. */
    vTaskDelete(NULL);
}
