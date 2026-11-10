#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"
static TickType_t fake_tick;
#define xTaskGetTickCount() fake_tick
#define xTaskGetTickCountFromISR() fake_tick

static CAN_TypeDef fake_can;
#undef CAN1
#define CAN1 (&fake_can)
#define NVIC_DisableIRQ(x) ((void)(x))
#define NVIC_ClearPendingIRQ(x) ((void)(x))
static unsigned checks, rx_count, rx_pos, overrun;
static uint32_t clock_hz = 42000000;
static uint8_t init_result = CAN_InitStatus_Success;
static CAN_InitTypeDef configuration;
static CAN_FilterInitTypeDef filtering;
static uint8_t mail_status[3], mail_used[3];
static CanTxMsg sent[3];
static CanRxMsg incoming[8];
#define CHECK(e) do { ++checks; if (!(e)) { printf("FAIL line %u: %s\n", __LINE__, #e); exit(1); } } while (0)
void RCC_GetClocksFreq(RCC_ClocksTypeDef *c) { memset(c,0,sizeof(*c)); c->PCLK1_Frequency=clock_hz; }
void RCC_AHB1PeriphClockCmd(uint32_t x, FunctionalState s) { (void)x;(void)s; }
void RCC_APB1PeriphClockCmd(uint32_t x, FunctionalState s) { (void)x;(void)s; }
void GPIO_StructInit(GPIO_InitTypeDef *g) { memset(g,0,sizeof(*g)); }
void GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p;(void)g; }
void GPIO_PinAFConfig(GPIO_TypeDef *p,uint16_t pin,uint8_t af) { (void)p;(void)pin; CHECK(af==GPIO_AF_CAN1); }
void CAN_DeInit(CAN_TypeDef *p) { (void)p; memset(mail_used,0,sizeof(mail_used)); }
void CAN_StructInit(CAN_InitTypeDef *c) { memset(c,0,sizeof(*c)); }
uint8_t CAN_Init(CAN_TypeDef *p,CAN_InitTypeDef *c) { (void)p; configuration=*c; return init_result; }
void CAN_FilterInit(CAN_FilterInitTypeDef *f) { filtering=*f; }
void CAN_ITConfig(CAN_TypeDef *p,uint32_t bits,FunctionalState s) { (void)p;(void)bits;(void)s; }
void NVIC_Init(NVIC_InitTypeDef *n) { CHECK(n->NVIC_IRQChannelPreemptionPriority==6); }
uint8_t CAN_Transmit(CAN_TypeDef *p,CanTxMsg *m) {
    unsigned i;(void)p;
    for(i=0;i<3;++i) if(!mail_used[i]) { mail_used[i]=1;mail_status[i]=CAN_TxStatus_Pending;sent[i]=*m;return i; }
    return CAN_TxStatus_NoMailBox;
}
uint8_t CAN_TransmitStatus(CAN_TypeDef *p,uint8_t i) { (void)p;return mail_status[i]; }
FlagStatus CAN_GetFlagStatus(CAN_TypeDef *p,uint32_t flag) { (void)p;(void)flag;return overrun?SET:RESET; }
void CAN_ClearFlag(CAN_TypeDef *p,uint32_t flag) { (void)p;(void)flag;overrun=0; }
uint8_t CAN_MessagePending(CAN_TypeDef *p,uint8_t fifo) { (void)p;(void)fifo;return rx_count-rx_pos; }
void CAN_Receive(CAN_TypeDef *p,uint8_t fifo,CanRxMsg *m) { (void)p;(void)fifo;*m=incoming[rx_pos++]; }
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/can_comm.c"
static void receive(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t len,uint8_t value) {
    memset(incoming,0,sizeof(incoming)); rx_count=1;rx_pos=0;
    incoming[0].StdId=id;incoming[0].IDE=ide;incoming[0].RTR=rtr;incoming[0].DLC=len;incoming[0].Data[0]=value;
    CAN1_RX0_IRQHandler();
}
static void complete(unsigned i,uint8_t status) { mail_used[i]=0;mail_status[i]=status; }
static void laser_frame(const uint8_t bytes[8]) {
    memset(incoming,0,sizeof(incoming));rx_count=1;rx_pos=0;
    incoming[0].StdId=0x321;incoming[0].IDE=CAN_Id_Standard;
    incoming[0].RTR=CAN_RTR_Data;incoming[0].DLC=8;
    memcpy(incoming[0].Data,bytes,8);CAN1_RX0_IRQHandler();
}
int main(void) {
    CAN_Comm_Stats s; CAN_Comm_LaserData d;
    uint8_t data[8]={0xE8,0x03,0xD0,0x07,0xB8,0x0B,0xFF,0xFF};
    uint8_t limits[8]={0,0,0xFE,0xFF,0x01,0x80,0x34,0x12};
    CHECK(CAN_Comm_Send(0x302,data,1)==CAN_COMM_NOT_READY);
    CHECK(CAN_Comm_Init()==CAN_COMM_OK);
    CAN_Comm_GetLaserData(&d);CHECK(!d.can_online && !d.can_rx_count && !d.laser_valid[0]);
    CHECK(d.laser_distance[0]==0xFFFF && d.laser_distance[3]==0xFFFF);
    CHECK(configuration.CAN_Prescaler==6 && configuration.CAN_BS1==CAN_BS1_11tq && configuration.CAN_BS2==CAN_BS2_2tq);
    CHECK(configuration.CAN_Mode==CAN_Mode_Normal && configuration.CAN_SJW==CAN_SJW_1tq);
    CHECK(configuration.CAN_ABOM==ENABLE && configuration.CAN_NART==DISABLE);
    CHECK(filtering.CAN_FilterIdHigh==(0x321U<<5) && filtering.CAN_FilterMaskIdHigh==0xFFE0U);
    CHECK(filtering.CAN_FilterMaskIdLow==6 && filtering.CAN_FilterFIFOAssignment==CAN_Filter_FIFO0);
    fake_tick=100;laser_frame(data);CAN_Comm_GetLaserData(&d);
    CHECK(d.can_online && d.can_rx_count==1 && d.last_rx_ms==100);
    CHECK(d.laser_distance[0]==1000 && d.laser_distance[1]==2000 && d.laser_distance[2]==3000 && d.laser_distance[3]==65535);
    CHECK(d.laser_valid[0] && d.laser_valid[1] && d.laser_valid[2] && !d.laser_valid[3]);
    CAN_Comm_Process();CAN_Comm_GetStats(&s);CHECK(s.tx_queued==0); /* No old 0x302 automatic replies. */
    fake_tick=299;CAN_Comm_TickFromISR();CAN_Comm_GetLaserData(&d);CHECK(d.can_online);
    fake_tick=300;CAN_Comm_TickFromISR();CAN_Comm_GetLaserData(&d);CHECK(d.can_online);
    fake_tick=301;CAN_Comm_TickFromISR();CAN_Comm_GetLaserData(&d);
    CHECK(!d.can_online && !d.laser_valid[0] && !d.laser_valid[1] && !d.laser_valid[2] && !d.laser_valid[3]);
    CHECK(d.laser_distance[0]==1000 && d.last_rx_ms==100 && d.can_rx_count==1);
    fake_tick=400;laser_frame(limits);CAN_Comm_GetLaserData(&d);
    CHECK(d.can_online && d.laser_distance[0]==0 && d.laser_distance[1]==65534 && d.laser_distance[2]==32769 && d.laser_distance[3]==0x1234);
    CHECK(d.laser_valid[0] && d.laser_valid[1] && d.laser_valid[2] && d.laser_valid[3]);
    fake_tick=500;
    receive(0x321,CAN_Id_Extended,CAN_RTR_Data,8,0x11);
    receive(0x321,CAN_Id_Standard,CAN_RTR_Remote,8,0x11);
    receive(0x321,CAN_Id_Standard,CAN_RTR_Data,7,0x11);
    receive(0x321,CAN_Id_Standard,CAN_RTR_Data,9,0x11);
    receive(0x301,CAN_Id_Standard,CAN_RTR_Data,8,0x11);
    receive(0x181,CAN_Id_Standard,CAN_RTR_Data,8,0x11);
    CAN_Comm_GetLaserData(&d);CHECK(d.can_rx_count==2 && d.last_rx_ms==400 && d.can_error_count==6);
    CAN_Comm_GetStats(&s);CHECK(s.rx_valid==2 && s.rx_ignored==6);
    fake_tick=601;CAN_Comm_GetLaserData(&d);CHECK(!d.can_online); /* Getter enforces timeout too. */
    fake_tick=700;memset(data,0xFF,8);laser_frame(data);CAN_Comm_GetLaserData(&d);
    CHECK(d.can_online && !d.laser_valid[0] && !d.laser_valid[1] && !d.laser_valid[2] && !d.laser_valid[3]);
    CHECK(d.last_rx_ms==700 && d.can_rx_count==3);
    fake_tick=0xFFFFFF80U;laser_frame(limits);
    fake_tick=0x48U;CAN_Comm_TickFromISR();CAN_Comm_GetLaserData(&d);CHECK(d.can_online);
    fake_tick=0x49U;CAN_Comm_TickFromISR();CAN_Comm_GetLaserData(&d);CHECK(!d.can_online);
    CHECK(CAN_Comm_Send(0x800,data,1)==CAN_COMM_INVALID_ARGUMENT);
    CHECK(CAN_Comm_Send(0x302,data,9)==CAN_COMM_INVALID_ARGUMENT);
    CHECK(CAN_Comm_Send(0x302,NULL,1)==CAN_COMM_INVALID_ARGUMENT);
    CHECK(CAN_Comm_Send(0x7FF,data,8)==CAN_COMM_OK);CHECK(memcmp(sent[0].Data,data,8)==0);
    CHECK(CAN_Comm_Send(0x302,NULL,0)==CAN_COMM_OK);
    CHECK(CAN_Comm_Send(0x302,data,1)==CAN_COMM_OK);
    CHECK(CAN_Comm_Send(0x302,data,1)==CAN_COMM_BUSY);
    complete(0,CAN_TxStatus_Ok);complete(2,CAN_TxStatus_Failed);CAN_Comm_Process();
    CAN_Comm_GetStats(&s);CHECK(s.tx_success==1 && s.tx_failed==1);
    overrun=1;rx_count=rx_pos=0;CAN1_RX0_IRQHandler();CAN_Comm_GetStats(&s);CHECK(s.rx_overruns==1);
    CAN_Comm_GetLaserData(&d);CHECK(d.can_error_count==7);
    fake_can.ESR=0x10;CAN_Comm_Process();CAN_Comm_GetLaserData(&d);CHECK(d.can_error_count==8);
    CAN_Comm_Process();CAN_Comm_GetLaserData(&d);CHECK(d.can_error_count==8);
    /* Three frame IRQ budget, then the next IRQ drains the remainder. */
    rx_count=5;rx_pos=0;memset(incoming,0,sizeof(incoming));
    for(unsigned i=0;i<5;++i){incoming[i].StdId=0x321;incoming[i].DLC=8;}
    CAN1_RX0_IRQHandler();CHECK(rx_pos==3);CAN1_RX0_IRQHandler();CHECK(rx_pos==5);
    __disable_irq();CAN_Comm_GetLaserData(&d);CHECK(__get_PRIMASK()==1);__enable_irq();
    clock_hz=36000000;CHECK(CAN_Comm_Init()==CAN_COMM_OK);CAN_Comm_GetStats(&s);
    CHECK(clock_hz==500000U*s.prescaler*(1U+s.bs1_tq+s.bs2_tq));
    clock_hz=1234567;CHECK(CAN_Comm_Init()==CAN_COMM_CLOCK_ERROR);
    CHECK(CAN_Comm_Send(0x302,data,1)==CAN_COMM_NOT_READY);
    CAN_Comm_GetLaserData(&d);CHECK(!d.can_online && !d.laser_valid[0]);
    clock_hz=42000000;init_result=CAN_InitStatus_Failed;CHECK(CAN_Comm_Init()==CAN_COMM_INIT_ERROR);
    CHECK(CAN_Comm_Send(0x302,data,1)==CAN_COMM_NOT_READY);
    printf("PASS: %u CAN laser checks\n",checks);return 0;
}
