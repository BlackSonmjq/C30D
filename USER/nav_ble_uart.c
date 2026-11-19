/* TXE 发送结构复用旧 servo 工程 route_vofa.c；不执行控制命令。 */
#include "system.h"
#include "nav_vofa.h"
#if NAV_BLE_PERIOD_MS*NAV_BLE_BAUD < NAV_VOFA_FRAME_BYTES*10000U
#error Bluetooth telemetry exceeds the configured UART bandwidth.
#endif
static NavVofaQueue tx;
void NavBle_Poll(void)
{
#if NAV_BLE_TELEMETRY_ENABLE
    NavStatus snapshot;
    uint8_t packet[NAV_VOFA_FRAME_BYTES];
    uint32_t now=NavBoard_Millis(),mask;
    if(!NavVofaQueue_Due(&tx,now)) return;
    NavBoard_ReadStatus(&snapshot);
    if(!NavVofa_Build(&snapshot,NavVofaQueue_NextSequence(&tx),packet,sizeof(packet))) return;
    mask=__get_PRIMASK(); __disable_irq();
    if(NavVofaQueue_Submit(&tx,packet,sizeof(packet),now))
        USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
    __set_PRIMASK(mask);
#endif
}
void NavBle_TxIRQ(void)
{
    uint8_t byte;
    if(USART_GetITStatus(USART2,USART_IT_TXE)==RESET) return;
    if(NavVofaQueue_Take(&tx,&byte)) USART_SendData(USART2,byte);
    else USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
}
