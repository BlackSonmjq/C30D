#include "can_test.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

volatile uint32_t g_can_test_rx_count;
volatile uint32_t g_can_test_last_counter;
volatile uint32_t g_can_test_lost_count;
volatile uint32_t g_can_test_error_count;
volatile uint8_t g_can_test_online;

volatile uint16_t g_can_distance_mm[4];
static TickType_t last_rx_tick;
static volatile uint8_t rx_pending;

void CAN_Test_Init(void)
{
	g_can_test_rx_count = 0;
	g_can_test_last_counter = 0;
	rx_pending = 0;
	g_can_test_lost_count = 0;
	g_can_test_error_count = 0;
	g_can_test_online = 0;
    { unsigned i; for (i = 0; i < 4; ++i) g_can_distance_mm[i] = 0; }
	last_rx_tick = xTaskGetTickCount();
}

void CAN_Test_Parse(u8 ide, u8 rtr, u8 len, const u8 *data)
{
	uint32_t counter;

	if(ide != CAN_Id_Standard || rtr != CAN_RTR_Data || len != 8 || data == 0)
	{
		g_can_test_error_count++;
		return;
	}

    for (counter = 0; counter < 4; ++counter)
        g_can_distance_mm[counter] = ((uint16_t)data[2*counter] << 8) | data[2*counter+1];
	g_can_test_rx_count++;
	/* CAN IRQ priority 1 must never call FreeRTOS APIs, including before startup. */
	rx_pending = 1;
}

void CAN_Test_Process(void)
{
	TickType_t now = xTaskGetTickCount();
    uint32_t mask = __get_PRIMASK();
    uint8_t received;

    /* BASEPRI critical sections cannot mask the priority-1 CAN interrupt. */
    __disable_irq();
    received = rx_pending;
    rx_pending = 0;
    __set_PRIMASK(mask);
    if (received) {
        last_rx_tick = now;
        g_can_test_online = 1;
    }

	if((TickType_t)(now - last_rx_tick) >= pdMS_TO_TICKS(CAN_TEST_TIMEOUT_MS))
		g_can_test_online = 0;
}

void CAN_TestTask(void *pvParameters)
{
	(void)pvParameters;
	for(;;)
	{
		CAN_Test_Process();
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}
