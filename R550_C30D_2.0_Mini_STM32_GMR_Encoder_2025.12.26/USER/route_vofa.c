#include "route_debug.h"
#include "route_config.h"
#include "vision_strike.h"
#include "FreeRTOS.h"
#include "task.h"

#if ROUTE_DEMO_ENABLE && ROUTE_DEBUG_MODE && ROUTE_VOFA_ENABLE
#include "stm32f4xx_usart.h"
#include "sys.h"
#include <string.h>

static uint8_t tx_bytes[ROUTE_VOFA_FRAME_BYTES];
static volatile uint16_t tx_index, tx_length;
static volatile uint8_t tx_busy;
static TickType_t last_tx_tick;
static uint8_t sent_once;

void RouteVofa_Poll(void)
{
    uint8_t packet[ROUTE_VOFA_FRAME_BYTES];
    uint16_t length;
    uint32_t mask;
    TickType_t now = xTaskGetTickCount();
    if ((sent_once && (TickType_t)(now - last_tx_tick) < pdMS_TO_TICKS(ROUTE_VOFA_PERIOD_MS)) ||
        tx_busy) return;
    length = RouteDebug_BuildPacket(packet, sizeof(packet));
    if (length == 0U) return;

    /* 只在提交一个短缓冲区时屏蔽中断。
     * 通道编码和实际串口发送都在临界区外，不调用阻塞 printf。 */
    mask = __get_PRIMASK();
    __disable_irq();
    if (!tx_busy) {
        memcpy(tx_bytes, packet, length);
        tx_index = 0U;
        tx_length = length;
        tx_busy = 1U;
        last_tx_tick = now;
        sent_once = 1U;
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    }
    __set_PRIMASK(mask);
}

uint8_t RouteVofa_IsActive(void) { return 1U; }

void RouteVofa_Uart2TxIRQ(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) == RESET) return;
    if (tx_busy && tx_index < tx_length) {
        USART_SendData(USART2, tx_bytes[tx_index]);
        ++tx_index;
    } else {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        tx_busy = 0U;
    }
}
#else
/* 原遥控模式、正式模式或关闭 VOFA 时，USART2 保持原蓝牙功能。 */
uint8_t RouteVofa_IsActive(void) { return 0U; }
void RouteVofa_Poll(void) { }
void RouteVofa_Uart2TxIRQ(void) { }
#endif

void RouteVofa_Task(void *parameters)
{
    TickType_t wake = xTaskGetTickCount();
    (void)parameters;
    for (;;) {
        VisionStrike_Process();
        RouteVofa_Poll();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(50U));
    }
}
