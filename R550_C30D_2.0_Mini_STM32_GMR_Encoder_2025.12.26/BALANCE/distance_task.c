#include "distance_task.h"
#include "stepper_task.h"
#include "oled.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define DISTANCE_POLL_MS 50U
#define DISTANCE_RETRY_MS 1000U
#define DISTANCE_STALE_MS 500U

static DistanceTask_Data distance_data = {0, 0, VL53L0X_NOT_READY, 0, 0};
static volatile uint8_t requested_laser = 0;
static uint8_t active_laser = 0;

void DistanceTask_InitShutdownPins(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);
}

void DistanceTask_SelectLaserFromISR(uint8_t laser_id)
{
    if (laser_id == 1U || laser_id == 2U)
        requested_laser = laser_id;
}

/* Retry waits remain responsive to a newly selected sensor. */
static uint8_t wait_selected(uint8_t laser_id, uint32_t ms)
{
    while (ms != 0U) {
        uint32_t slice = ms > 10U ? 10U : ms;
        if (requested_laser != laser_id) return 0;
        vTaskDelay(pdMS_TO_TICKS(slice));
        ms -= slice;
    }
    return requested_laser == laser_id;
}

static void publish(VL53L0X_Status status, const VL53L0X_Result *result)
{
    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    distance_data.status = status;
    distance_data.valid = (status == VL53L0X_OK && active_laser != 0U &&
                           active_laser == requested_laser);
    distance_data.laser_id = active_laser;
    distance_data.update_tick = now;
    distance_data.distance_mm = (distance_data.valid && result != NULL) ? result->distance_mm : 0;
    taskEXIT_CRITICAL();
}

void DistanceTask_GetData(DistanceTask_Data *data)
{
    TickType_t now;
    if (data == NULL) return;
    taskENTER_CRITICAL();
    *data = distance_data;
    taskEXIT_CRITICAL();
    if (data->laser_id != requested_laser) {
        data->valid = 0;
        data->distance_mm = 0;
        data->status = VL53L0X_NOT_READY;
    }
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
    for (;;) {
        if (active_laser != requested_laser) {
            /* Both devices use address 0x29. Never release both XSHUTs. */
            GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
            active_laser = 0;
            publish(VL53L0X_NOT_READY, NULL);
            vTaskDelay(pdMS_TO_TICKS(2U));
            active_laser = requested_laser;
            if (active_laser != 0U)
                GPIO_SetBits(GPIOA, active_laser == 1U ? GPIO_Pin_2 : GPIO_Pin_3);
            publish(VL53L0X_NOT_READY, NULL);
            if (!wait_selected(active_laser, 10U)) continue;
        }
        if (active_laser == 0U) {
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }
        status = VL53L0X_Init();
        if (active_laser != requested_laser) continue;
        if (status != VL53L0X_OK) {
            publish(status, NULL);
            wait_selected(active_laser, DISTANCE_RETRY_MS);
            continue;
        }
        publish(VL53L0X_NOT_READY, NULL);
        for (;;) {
            if (!wait_selected(active_laser, DISTANCE_POLL_MS)) break;
            status = VL53L0X_ReadSingle(&result);
            if (active_laser != requested_laser) break;
            publish(status, status == VL53L0X_OK ? &result : NULL);
            if (status == VL53L0X_OK || status == VL53L0X_ERROR_RANGE) continue;
            wait_selected(active_laser, DISTANCE_RETRY_MS);
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
    if (data.laser_id == 1U)
        OLED_ShowString(0, 0, "Laser 1 PA2     ");
    else if (data.laser_id == 2U)
        OLED_ShowString(0, 0, "Laser 2 PA3     ");
    else
        OLED_ShowString(0, 0, "Lasers OFF      ");
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
