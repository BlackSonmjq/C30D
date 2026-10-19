#include "timer.h"
#include "stepper_motor.h"

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��1���벶���־������λ�������־����6λ�������־		
u8 TIM8CH1_CAPTURE_STA = 0;	
u16 TIM8CH1_CAPTURE_UPVAL;
u16 TIM8CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��2���벶���־������λ�������־����6λ�������־	
u8 TIM8CH2_CAPTURE_STA = 0;		
u16 TIM8CH2_CAPTURE_UPVAL;
u16 TIM8CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��3���벶���־������λ�������־����6λ�������־	
u8 TIM8CH3_CAPTURE_STA = 0;		
u16 TIM8CH3_CAPTURE_UPVAL;
u16 TIM8CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//ͨ��4���벶���־������λ�������־����6λ�������־
u8 TIM8CH4_CAPTURE_STA = 0;			
u16 TIM8CH4_CAPTURE_UPVAL;
u16 TIM8CH4_CAPTURE_DOWNVAL;

u32 TIM8_T1;
u32 TIM8_T2;
u32 TIM8_T3;
u32 TIM8_T4;

//Variables related to remote control acquisition of model aircraft
//��ģң�زɼ���ر���
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//��ģң�ؽ��ձ���
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
�������ܣ���ģң�س�ʼ����������ʱ��1���벶���ʼ��
��ڲ�����arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ�� 
�� �� ֵ����
**************************************************************************/ 
void TIM8_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTEʱ��	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //GPIOC
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);

	/*** Initialize timer 1 || ��ʼ����ʱ��1 ***/
	//Set the counter to automatically reload //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 

	/*** ��ʼ��TIM1���벶�������ͨ��1 || Initialize TIM1 for the capture parameter, channel 1 ***/
	//Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  //Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
  //IC1F=0000 Configure input filter //���������˲���
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	/*** ��ʼ��TIM1���벶�������ͨ��2 || Initialize TIM1 for the capture parameter, channel 2 ***/
	//CC1S=01 Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲���
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	/*** ��ʼ��TIM1���벶�������ͨ��3 || Initialize TIM1 for the capture parameter, channel 3 ***/
	//Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	//Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//IC1F=0000 Configure input filter //���������˲��������˲�  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

	/*** ��ʼ��TIM1���벶�������ͨ��4 || Initialize TIM1 for the capture parameter, channel 4 ***/
	//Select input //ѡ������� 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	//Rising edge capture //�����ز���
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	   
	//IC1F=0000 Configure input filter //���������˲��������˲�  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure);

  /*** interrupt packet initialization || �жϷ����ʼ�� ***/
  //TIM1 interrupts //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn; 
  //Preempt priority 0 //��ռ���ȼ�0��	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
	//Level 0 from priority //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	//IRQ channels are enabled //IRQͨ����ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	NVIC_Init(&NVIC_InitStructure);   
	
  //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
  //�����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�	
	TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   
	//Advanced timer output must be enabled //�߼���ʱ���������ʹ�����	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	//Enable timer //ʹ�ܶ�ʱ��
	TIM_Cmd(TIM8, ENABLE); 		
}
/**************************************************************************
Function: Model aircraft remote control receiving interrupt, namely timer 8 input capture interrupt
Input   : none
Output  : none
�������ܣ���ģң�ؽ����жϣ�����ʱ��8���벶���ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/ 
void TIM8_CC_IRQHandler(void)
{
	static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	
	//���Ӻ�ģңң��������Ҫ����ǰ���ˣ��ſ�����ʽ��ģ����С��
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
  if(Remoter_Ch2>1600&&Remote_ON_Flag==0&&SysVal.Time_count>=CONTROL_DELAY)
  {
		//Model aircraft remote control mark position 1, other marks position 0
		//��ģң�ر�־λ��1��������־λ��0
		Remote_ON_Flag=1;
	  APP_ON_Flag=0;
		PS2_ON_Flag=0;
		CAN_ON_Flag=0;
		//Usart_ON_Flag=0; 
		Usart1_ON_Flag=0;
		Usart5_ON_Flag=0;
	}
	
//	//Channel 1 //ͨ��һ
	if ((TIM8CH1_CAPTURE_STA & 0X80) == 0) 			
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) //A capture event occurred on channel 1 //ͨ��1���������¼�
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); //Clear the interrupt flag bit //����жϱ�־λ
			if (TIM8CH1_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
			{
				TIM8CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM8); //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM8CH1_CAPTURE_DOWNVAL < TIM8CH1_CAPTURE_UPVAL)
				{
					TIM8_T1 = 9999;
				}
				else
					TIM8_T1 = 0;
				Remoter_Ch1 = TIM8CH1_CAPTURE_DOWNVAL - TIM8CH1_CAPTURE_UPVAL + TIM8_T1;	//Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if( ch1_filter_times<=5 ) Remoter_Ch1=L_Remoter_Ch1; //Filter //�˲�	
					else ch1_filter_times=0;
				}
				else
				{
					ch1_filter_times=0;
				}
				L_Remoter_Ch1=Remoter_Ch1;
				
				TIM8CH1_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
				TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //����Ϊ�����ز���		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
				TIM8CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM8); //Obtain rising edge data //��ȡ����������
				TIM8CH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
				TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //����Ϊ�½��ز���
			}
		}
	}
  //Channel 2 //ͨ����
	if ((TIM8CH2_CAPTURE_STA & 0X80) == 0)		
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)	//A capture event occurred on channel 2 //ͨ��2���������¼�
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC2); //Clear the interrupt flag bit //����жϱ�־λ
			if (TIM8CH2_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
			{
				TIM8CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM8); //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM8CH2_CAPTURE_DOWNVAL < TIM8CH2_CAPTURE_UPVAL)
				{
					TIM8_T2 = 9999;
				}
				else
					TIM8_T2 = 0;
				Remoter_Ch2 = TIM8CH2_CAPTURE_DOWNVAL - TIM8CH2_CAPTURE_UPVAL + TIM8_T2; //Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if( ch2_filter_times<=5 ) Remoter_Ch2=L_Remoter_Ch2; //Filter //�˲�	
					else ch2_filter_times=0;
				}
				else
				{
					ch2_filter_times=0;
				}
				
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM8CH2_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //����Ϊ�����ز���		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
				TIM8CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM8); //Obtain rising edge data //��ȡ����������
				TIM8CH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
				TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //����Ϊ�½��ز���
			}
		}
	}
  //Channel 3 //ͨ����
	if ((TIM8CH3_CAPTURE_STA & 0X80) == 0)			
	{
		if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)	//A capture event occurred on channel 3 //ͨ��3���������¼�
		{
			TIM_ClearITPendingBit(TIM8, TIM_IT_CC3); //Clear the interrupt flag bit //����жϱ�־λ
			if (TIM8CH3_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
			{
				TIM8CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM8); //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM8CH3_CAPTURE_DOWNVAL < TIM8CH3_CAPTURE_UPVAL)
				{
					TIM8_T3 = 9999;
				}
				else
					TIM8_T3 = 0;
				Remoter_Ch3 = TIM8CH3_CAPTURE_DOWNVAL - TIM8CH3_CAPTURE_UPVAL + TIM8_T3; //Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
				if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)
				{
					ch3_filter_times++;
					if( ch3_filter_times<=5 ) Remoter_Ch3=L_Remoter_Ch3; //Filter //�˲�	
					else ch3_filter_times=0;
				}
				else
				{
					ch3_filter_times=0;
				}
				L_Remoter_Ch3=Remoter_Ch3;	
				TIM8CH3_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
				TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //����Ϊ�����ز���		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
				TIM8CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM8); //Obtain rising edge data //��ȡ����������
				TIM8CH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
				TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //����Ϊ�½��ز���
			}
		}
	}
//	

		//Channel 4 //ͨ����
		if ((TIM8CH4_CAPTURE_STA & 0X80) == 0)		
		{
			if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET)	//A capture event occurred on channel 4 //ͨ��4���������¼�
			{
				TIM_ClearITPendingBit(TIM8, TIM_IT_CC4); //Clear the interrupt flag bit //����жϱ�־λ
				if (TIM8CH4_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
				{
					TIM8CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM8); //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
					if (TIM8CH4_CAPTURE_DOWNVAL < TIM8CH4_CAPTURE_UPVAL)
					{
						TIM8_T4 = 9999;
					}
					else
						TIM8_T4 = 0;
					Remoter_Ch4 = TIM8CH4_CAPTURE_DOWNVAL - TIM8CH4_CAPTURE_UPVAL + TIM8_T4; //Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
					if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)
					{
						ch4_filter_times++;
						if( ch4_filter_times<=5 ) Remoter_Ch4=L_Remoter_Ch4; //Filter //�˲�	
						else ch4_filter_times=0;
					}
					else
					{
						ch4_filter_times=0;
					}
					
					L_Remoter_Ch4=Remoter_Ch4;			
					TIM8CH4_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
					TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Rising); //Set to rising edge capture //����Ϊ�����ز���		  
				}
				else 
				{
					//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				  //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
					TIM8CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM8); //Obtain rising edge data //��ȡ����������
					TIM8CH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
					TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //����Ϊ�½��ز���
				}
			}
		}
}
/**************************************************************************
Function: TIM1 Update Interrupt
Input   : none
Output  : none
�������ܣ���ʱ��8�����ж�
��ڲ�������
����  ֵ���� 
**************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{
    StepperMotor_UpdateIRQ(A4988_MOTOR_Right);
}

void TIM8_SERVO_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //��ʱ��
	TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM���
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTEʱ��	
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); 


	/*** Initialize timer 1 || ��ʼ����ʱ��1 ***/
	//Set the counter to automatically reload //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 


	//-----------�����ʼ��-----------//
	//Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   
	
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); 
	//Channel preload enable
	//ͨ��Ԥװ��ʹ��	 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	//-----------�����ʼ��-----------//
	TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	DISABLE);
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	//Enable timer //ʹ�ܶ�ʱ��
	TIM_Cmd(TIM8, ENABLE); 		 

  //The channel value is initialized to 1500, corresponding to the steering gear zero
	//ͨ��ֵ��ʼ��Ϊ1500���������Ӧֵ
//	TIM8->CCR1=1500;
//	TIM8->CCR2=1500;
//	TIM8->CCR3=1500;
//	TIM8->CCR4=1500;
}


void TIM12_SERVO_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;           //IO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //��ʱ��
	TIM_OCInitTypeDef  TIM_OCInitStructure;        //PWM���
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTEʱ��	
		
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; 	
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12); 


	/*** Initialize timer 1 || ��ʼ����ʱ��1 ***/
	//Set the counter to automatically reload //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM���ϼ���ģʽ	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure); 


	//-----------�����ʼ��-----------//
	//Select Timer mode :TIM Pulse Width Modulation mode 1
  //ѡ��ʱ��ģʽ:TIM������ȵ���ģʽ1
 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	//Compare output enablement
	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	//Set the pulse value of the capture comparison register to be loaded
	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_Pulse = 0; 
  //Output polarity :TIM output polarity is higher	
  //�������:TIM����Ƚϼ��Ը�	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;   
	
	//Initialize the peripheral TIMX based on the parameter specified in TIM_OCINITSTRUCT
  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx	
	TIM_OC1Init(TIM12, &TIM_OCInitStructure); 
		TIM_OC2Init(TIM12, &TIM_OCInitStructure); 
	//Channel preload enable
	//ͨ��Ԥװ��ʹ��	 
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);
		TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
	//-----------�����ʼ��-----------//


	TIM_CtrlPWMOutputs(TIM12,ENABLE); 	
	//Enable timer //ʹ�ܶ�ʱ��
	TIM_Cmd(TIM12, ENABLE); 		 

  //The channel value is initialized to 1500, corresponding to the steering gear zero
	//ͨ��ֵ��ʼ��Ϊ1500���������Ӧֵ
	TIM12->CCR2=1500;
	TIM12->CCR1=1500;

}

int Remoter_Ch1 = 1500;
int Remoter_Ch2 = 1500;
int Remoter_Ch3 = 1500;
int Remoter_Ch4 = 1500;

int L_Remoter_Ch1 = 1500;
int L_Remoter_Ch2 = 1500;
int L_Remoter_Ch3 = 1500;
int L_Remoter_Ch4 = 1500;
