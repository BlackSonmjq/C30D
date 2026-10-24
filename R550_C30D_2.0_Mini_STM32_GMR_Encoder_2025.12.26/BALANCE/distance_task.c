#include "distance_task.h"
#include "stepper_task.h"
#include "oled.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define DISTANCE_POLL_MS 50U
#define DISTANCE_RETRY_MS 1000U
#define DISTANCE_STALE_MS 500U

static DistanceTask_Data distance_data = {0, 0, VL53L0X_NOT_READY, 0};

static void enable_sensor(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_SetBits(GPIOA, GPIO_Pin_3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);
}

static void publish(VL53L0X_Status status, const VL53L0X_Result *result)
{
    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    distance_data.status = status;
    distance_data.valid = (status == VL53L0X_OK);
    distance_data.update_tick = now;
    distance_data.distance_mm = (status == VL53L0X_OK && result != NULL) ? result->distance_mm : 0;
    taskEXIT_CRITICAL();
}

void DistanceTask_GetData(DistanceTask_Data *data)
{
    TickType_t now;
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = distance_data;
    taskEXIT_CRITICAL();
    now = xTaskGetTickCount();
    if (data->valid && (TickType_t)(now - data->update_tick) >= pdMS_TO_TICKS(DISTANCE_STALE_MS)) {
        data->valid = 0;
        data->distance_mm = 0;
        data->status = VL53L0X_ERROR_TIMEOUT;
    }
}

void distance_task(void *pvParameters)
{
    VL53L0X_Status status;
    VL53L0X_Result result;
    (void)pvParameters;
    enable_sensor();
    vTaskDelay(pdMS_TO_TICKS(10U));
    for (;;) {
        status = VL53L0X_Init();
        if (status != VL53L0X_OK) {
            publish(status, NULL);
            vTaskDelay(pdMS_TO_TICKS(DISTANCE_RETRY_MS));
            continue;
        }
        publish(VL53L0X_NOT_READY, NULL);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(DISTANCE_POLL_MS));
            status = VL53L0X_ReadSingle(&result);
            publish(status, status == VL53L0X_OK ? &result : NULL);
            if (status == VL53L0X_OK || status == VL53L0X_ERROR_RANGE) continue;
            vTaskDelay(pdMS_TO_TICKS(DISTANCE_RETRY_MS));
            break;
        }
    }
}

void DistanceTask_DrawOLED(void)
{
    DistanceTask_Data data;
    StepperTask_Data stepper;
    DistanceTask_GetData(&data);
    StepperTask_GetData(&stepper);
    OLED_ShowString(0, 0, "Laser Distance");
    if (data.valid) {
        OLED_ShowString16(0, 20, "Range:");
        OLED_ShowNumber(54, 20, data.distance_mm, 5, 16);
        OLED_ShowString(98, 24, "mm");
    } else {
        OLED_ShowString(0, 20, "Range: ----- mm");
    }
    /* 10 decimal digits cover the full u32 pulse count within 128 pixels. */
    OLED_ShowString(0, 36, "Steps:");
    OLED_ShowNumber(48, 36, stepper.pulses, 10, 12);
    switch (data.status) {
        case VL53L0X_OK: OLED_ShowString(0, 50, "OK"); break;
        case VL53L0X_NOT_READY: OLED_ShowString(0, 50, "INIT / WAIT"); break;
        case VL53L0X_ERROR_RANGE: OLED_ShowString(0, 50, "INVALID RANGE"); break;
        case VL53L0X_ERROR_TIMEOUT: OLED_ShowString(0, 50, "TIMEOUT / RETRY"); break;
        case VL53L0X_ERROR_STATE: OLED_ShowString(0, 50, "TASK STATE ERROR"); break;
        default: OLED_ShowString(0, 50, "SENSOR ERROR"); break;
    }
}
