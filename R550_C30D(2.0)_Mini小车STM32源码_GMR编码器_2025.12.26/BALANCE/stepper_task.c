#include "stepper_task.h"
#include "FreeRTOS.h"
#include "task.h"

/* One revolution for a 1.8-degree motor with 1/16 microstepping. */
#define STEPPER_PULSES_PER_REV (200 * 16)
#define STEPPER_PAUSE_MS 1000u

void stepper_Task(void *pvParameters)
{
    (void)pvParameters;
    StepperMotor_Init();

    while (1) {
        StepperMotor_SetSpeed(A4988_MOTOR_Left, 99u);
        StepperMotor_MovePulses(A4988_MOTOR_Left, STEPPER_PULSES_PER_REV);

        vTaskDelay(pdMS_TO_TICKS(STEPPER_PAUSE_MS));

        StepperMotor_SetSpeed(A4988_MOTOR_Left, 299u);
        StepperMotor_MovePulses(A4988_MOTOR_Left, STEPPER_PULSES_PER_REV);
        vTaskDelay(pdMS_TO_TICKS(STEPPER_PAUSE_MS));

    }
}
