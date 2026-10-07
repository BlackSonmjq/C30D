#include "stepper_task.h"

void stepper_task(void *pvParameters)
{
    stepper_init();
    while(1)
    {
        stepper_run(STEPPER_1, 3200, 1000);
        vTaskDelay(10*1000);
        stepper_stop(STEPPER_1);
        vTaskDelay(10*1000);
    }
}