#include "usart.h"
#include <string.h>
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
    (void)f;
#if USART1_SERVO_LOG_ONLY
    return ch; /* Suppress legacy printf telemetry. */
#else
    return USART_WriteByte(USART1, (uint8_t)ch) ? ch : EOF;
#endif
}

//任意串口printf
#include "stdarg.h"
/* A finite poll budget works before the scheduler and with IRQs masked. */
#define USART_TX_POLL_LIMIT 1000000UL
volatile uint32_t usart_tx_timeout_count;
volatile uint32_t usart_tx_disabled_count;

static int USART_WaitFlag(USART_TypeDef *port, uint32_t flag)
{
    uint32_t remaining = USART_TX_POLL_LIMIT;
    while ((port->SR & flag) == 0U) {
        if (--remaining == 0U) {
            ++usart_tx_timeout_count;
            return 0;
        }
    }
    return 1;
}

int USART_WriteByte(USART_TypeDef *port, uint8_t byte)
{
    if ((port->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
        (USART_CR1_UE | USART_CR1_TE)) {
        ++usart_tx_disabled_count;
        return 0;
    }
    /* TXE permits the first write even when TC was previously cleared. */
    if (!USART_WaitFlag(port, USART_SR_TXE)) return 0;
    port->DR = byte;
    return USART_WaitFlag(port, USART_SR_TC);
}

void any_printf(USART_TypeDef *USARTx, char *format, ...)
{
    char buffer[256];
    unsigned int i;
    int length;
    va_list args;
#if USART1_SERVO_LOG_ONLY
    if (USARTx == USART1 && strncmp(format, "SERVO ", 6U) != 0 &&
        strncmp(format, "servo ", 6U) != 0) return;
#endif
    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0) return;
    if ((unsigned int)length >= sizeof(buffer)) length = sizeof(buffer) - 1U;
    for (i = 0; i < (unsigned int)length; ++i) {
        if (!USART_WriteByte(USARTx, (uint8_t)buffer[i])) return;
    }
}

#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、


	
}
#endif





 



