#include "pushrod_task.h"
#include "pushrod.h"

void pushrod_task(void *pvParameters)
{   
    Pushrod_Init();
    while(1)
    {
        Pushrod_Forward(1); 
        vTaskDelay(10*1000);
        Pushrod_Stop(1);
        vTaskDelay(10*1000);
    }
}
