#include "distance_task.h"
#include "stepper_task.h"
#include "oled.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define DISTANCE_POLL_MS 50U    /* 每次单次测距前的任务等待时间，不含测量耗时。 */
#define DISTANCE_RETRY_MS 1000U /* 初始化或通信失败后的重试间隔。 */
#define DISTANCE_STALE_MS 500U  /* 超过此时效的数据不能用于控制电机。 */
#define LASER_SHUT_PINS (GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7)

/* 四路共用默认 I2C 地址，必须通过独立 XSHUT 互斥启用。
 * 当前实物接入激光 1 和 3，编号仍按 1～4 保留。 */
static const uint16_t laser_shut_pin[4] = {
    GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_6, GPIO_Pin_7
};

static DistanceTask_Data distance_data = {0, 0, VL53L0X_NOT_READY, 0, 0};
static volatile uint8_t requested_laser = 0;
static uint8_t active_laser = 0;

void DistanceTask_InitShutdownPins(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* 先预置输出寄存器为低，再切换输出模式，避免上电时误释放传感器。 */
    GPIO_ResetBits(GPIOA, LASER_SHUT_PINS);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = LASER_SHUT_PINS;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);
}

void DistanceTask_SelectLaser(uint8_t laser_id)
{
    if (laser_id <= 4U)
        requested_laser = laser_id;
}

/* 将轮询和重试等待拆成最多 10 ms 的片段，及时响应切换或全关请求。 */
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
    /* 只发布当前请求通道的有效距离；切换中和异常时清零距离。 */
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
        /* 即使测距任务还未切换引脚，也立即禁止控制任务使用旧通道数据。 */
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
            /* 地址均为 0x29：先关闭全部设备并等待 2 ms，再仅释放选中一路。 */
            GPIO_ResetBits(GPIOA, LASER_SHUT_PINS);
            active_laser = 0;
            publish(VL53L0X_NOT_READY, NULL);
            vTaskDelay(pdMS_TO_TICKS(2U));
            active_laser = requested_laser;
            if (active_laser != 0U)
                GPIO_SetBits(GPIOA, laser_shut_pin[active_laser - 1U]);
            publish(VL53L0X_NOT_READY, NULL);
            /* 选中设备启动后等待 10 ms，再访问寄存器进行初始化。 */
            if (!wait_selected(active_laser, 10U)) continue;
        }
        if (active_laser == 0U) {
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }
        /* 本任务独占 VL53L0X 驱动与总线，其他任务只读快照。 */
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
            /* 无效距离继续测量；通信等错误则等待后重新初始化。 */
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
    else if (data.laser_id == 3U)
        OLED_ShowString(0, 0, "Laser 3 PA6     ");
    else if (data.laser_id == 4U)
        OLED_ShowString(0, 0, "Laser 4 PA7     ");
    else
        OLED_ShowString(0, 0, "Lasers OFF      ");
    if (data.valid) {
        OLED_ShowString16(0, 20, "Range:");
        OLED_ShowNumber(54, 20, data.distance_mm, 5, 16);
        OLED_ShowString(98, 24, "mm");
    } else {
        OLED_ShowString(0, 20, "Range: ----- mm");
    }
    /* 以 10 位十进制显示完整 u32 计数，包含细分脉冲。 */
    OLED_ShowString(0, 36, "Steps:");
    OLED_ShowNumber(48, 36, stepper.pulses, 10, 12);
    switch (stepper.state) {
        case STEPPER_WAITING_COMMAND: OLED_ShowString(0, 50, "WAIT CMD 1-4"); return;
        case STEPPER_WAITING_DISTANCE: OLED_ShowString(0, 50, "WAIT RANGE"); return;
        case STEPPER_MOVING: OLED_ShowString(0, 50, "APPROACH"); return;
        case STEPPER_STRIKING: OLED_ShowString(0, 50, "STRIKE 5s"); return;
        case STEPPER_RETURNING: OLED_ShowString(0, 50, "RETURNING"); return;
        case STEPPER_FINISHING: OLED_ShowString(0, 50, "LASER OFF"); return;
        case STEPPER_RETURNED:
            if (stepper.result == MANGO_OK) OLED_ShowString(0, 50, "DONE / READY");
            else OLED_ShowString(0, 50, "ABORT / READY");
            return;
        case STEPPER_FAULT: OLED_ShowString(0, 50, "RETURN FAULT"); return;
        default: break;
    }
    switch (data.status) {
        case VL53L0X_OK: OLED_ShowString(0, 50, "OK"); break;
        case VL53L0X_NOT_READY: OLED_ShowString(0, 50, "INIT / WAIT"); break;
        case VL53L0X_ERROR_RANGE: OLED_ShowString(0, 50, "INVALID RANGE"); break;
        case VL53L0X_ERROR_TIMEOUT: OLED_ShowString(0, 50, "TIMEOUT / RETRY"); break;
        case VL53L0X_ERROR_STATE: OLED_ShowString(0, 50, "TASK STATE ERROR"); break;
        default: OLED_ShowString(0, 50, "SENSOR ERROR"); break;
    }
}
