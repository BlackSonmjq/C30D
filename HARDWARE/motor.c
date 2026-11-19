#include "motor.h"

/* Balance task owns this gate. Before timers are configured all eight pins stay low. */
static int motor_output_state = -1;
static void motor_pin_mode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef io;
    GPIO_StructInit(&io);
    io.GPIO_Mode=mode; io.GPIO_OType=GPIO_OType_PP;
    io.GPIO_Speed=GPIO_Speed_100MHz; io.GPIO_PuPd=GPIO_PuPd_DOWN;
    io.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9; GPIO_Init(GPIOB,&io);
    io.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
    GPIO_Init(GPIOE,&io);
}
void Motor_OutputInit(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOE,ENABLE);
    GPIO_ResetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9);
    GPIO_ResetBits(GPIOE,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14);
    motor_pin_mode(GPIO_Mode_OUT);
    motor_output_state=-1;
}
static void motor_channel_mode(TIM_TypeDef *timer,uint16_t channel,uint16_t mode)
{
    /* SelectOCxM clears CCxE; explicitly reconnect the channel after changing mode. */
    TIM_SelectOCxM(timer,channel,mode);
    TIM_CCxCmd(timer,channel,TIM_CCx_Enable);
}
void Motor_OutputEnable(u8 enable)
{
    int next=enable?1:0;
    uint16_t mode=next?TIM_OCMode_PWM1:TIM_ForcedAction_InActive;
    if(motor_output_state==next) return;
    motor_channel_mode(TIM1,TIM_Channel_1,mode);
    motor_channel_mode(TIM1,TIM_Channel_2,mode);
    motor_channel_mode(TIM1,TIM_Channel_3,mode);
    motor_channel_mode(TIM1,TIM_Channel_4,mode);
    motor_channel_mode(TIM9,TIM_Channel_1,mode);
    motor_channel_mode(TIM9,TIM_Channel_2,mode);
    motor_channel_mode(TIM10,TIM_Channel_1,mode);
    motor_channel_mode(TIM11,TIM_Channel_1,mode);
    if(motor_output_state<0) motor_pin_mode(GPIO_Mode_AF);
    motor_output_state=next;
}



/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
函数功能：使能开关引脚初始化
入口参数：无
返回  值：无 
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //KEY对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOB14
} 

void TIM1_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse=0;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	  //TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTC时钟	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;   //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //上拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PC口
	
	//Sets the value of the auto-reload register cycle for the next update event load activity
	//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Sets the pre-divider value used as the TIMX clock frequency divisor
	//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	//Set the clock split :TDTS = Tck_tim
	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//Up counting mode 
	//向上计数模式  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TIMEBASEINITSTRUCT
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 

  //Select Timer mode :TIM Pulse Width Modulation mode 1
  //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//比较输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  //Output polarity :TIM output polarity is higher	
  //输出极性:TIM输出比较极性高	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 
	
	// Advanced timer output must be enabled
	//高级定时器输出必须使能这句		
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	//CH1 is pre-loaded and enabled
	//CH1预装载使能	 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  

  // Enable the TIMX preloaded register on the ARR
  //使能TIMx在ARR上的预装载寄存器	
	TIM_ARRPreloadConfig(TIM1, ENABLE); 
	
	//Enable TIM8
	//使能TIM8
	TIM_Cmd(TIM1, ENABLE);  
}

void TIM9_PWM_Init(u16 arr,u16 psc)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse=0;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);/*使能定时器11时钟*/
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);/*使能GPIOF时钟*/

GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);/*复用*/
GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);/*复用*/

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;           //GPIOF9
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //上拉
GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PF9

TIM_TimeBaseInitStructure.TIM_Period = arr;/*自动重装载*/
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*预分频*/
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*时钟分频*/
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*向上计数*/
TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);/*初始化*/

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWM模式*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*输出*/
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*比较极性高*/
TIM_OC1Init(TIM9,&TIM_OCInitStructure);
TIM_OC2Init(TIM9,&TIM_OCInitStructure);

TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);/*输出比较预装载使能*/
TIM_OC2PreloadConfig(TIM9,TIM_OCPreload_Enable);/*输出比较预装载使能*/

TIM_ARRPreloadConfig(TIM9,ENABLE);/*自动重载预装载使能*/

TIM_Cmd(TIM9,ENABLE);/*计数使能*/

}


void TIM10_PWM_Init(u16 arr,u16 psc)
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse=0;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);/*使能GPIOF时钟*/
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);/*使能定时器11时钟*/

GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM10);/*复用*/

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;/*复用*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*推挽输出*/
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;/*PF7*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;/*上拉*/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
GPIO_Init(GPIOB,&GPIO_InitStructure);/*初始化IO*/

TIM_TimeBaseInitStructure.TIM_Period = arr;/*自动重装载*/
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*预分频*/
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*时钟分频*/
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*向上计数*/
TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);/*初始化*/

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWM模式*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*输出*/
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*比较极性高*/
TIM_OC1Init(TIM10,&TIM_OCInitStructure);

TIM_OC1PreloadConfig(TIM10,TIM_OCPreload_Enable);/*输出比较预装载使能*/

//TIM_CtrlPWMOutputs(TIM10,ENABLE);

TIM_ARRPreloadConfig(TIM10,ENABLE);/*自动重载预装载使能*/

TIM_Cmd(TIM10,ENABLE);/*计数使能*/

}

void TIM11_PWM_Init(u16 arr,u16 psc)
{

TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse=0;

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);/*使能GPIOF时钟*/
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);/*使能定时器11时钟*/

GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11);/*复用*/

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;/*复用*/
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;/*推挽输出*/
GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;/*PF7*/
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;/*上拉*/
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/**/
GPIO_Init(GPIOB,&GPIO_InitStructure);/*初始化IO*/

TIM_TimeBaseInitStructure.TIM_Period = arr;/*自动重装载*/
TIM_TimeBaseInitStructure.TIM_Prescaler = psc;/*预分频*/
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;/*时钟分频*/
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;/*向上计数*/
TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);/*初始化*/

TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;/*PWM模式*/
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;/*输出*/
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;/*比较极性高*/
TIM_OC1Init(TIM11,&TIM_OCInitStructure);

TIM_OC1PreloadConfig(TIM11,TIM_OCPreload_Enable);/*输出比较预装载使能*/

/* TIM11 has no advanced-timer MOE/BDTR output control. */

TIM_ARRPreloadConfig(TIM11,ENABLE);/*自动重载预装载使能*/

TIM_Cmd(TIM11,ENABLE);/*计数使能*/

}








