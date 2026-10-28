#include "pushrod_task.h"
#include "pushrod.h"
#include "stepper_task.h"

static volatile u8 start_requested = 0;

void PushrodTask_RequestStartFromISR(void)
{
    /* USART1 IRQ priority 3: no FreeRTOS API calls here. */
    if (StepperTask_IsPositionReadyFromISR())
        start_requested = 1;
}

void pushrod_task(void *pvParameters)
{
    StepperTask_Data stepper;
    (void)pvParameters;
    Pushrod_Init();
    while(1)
    {
        /* Recheck in the task: a return may have started after command 3. */
        taskENTER_CRITICAL();
        if (start_requested) {
            start_requested = 0;
            StepperTask_GetData(&stepper);
            if (stepper.state == STEPPER_TARGET_REACHED)
                Pushrod_Stop(1); /* Keep PC0 low. */
        }
        taskEXIT_CRITICAL();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
