#include "stepper_task.h"
#include "FreeRTOS.h"
#include "task.h"

void stepper_Task(void *pvParameters)
{
    (void)pvParameters;
    StepperMotor_SetSpeed(A4988_MOTOR_Left, 99);

    for (;;) {
        StepperMotor_MovePulses(A4988_MOTOR_Left, -40000);
        vTaskDelay(pdMS_TO_TICKS(1000U));
        StepperMotor_MovePulses(A4988_MOTOR_Left, 40000);
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
