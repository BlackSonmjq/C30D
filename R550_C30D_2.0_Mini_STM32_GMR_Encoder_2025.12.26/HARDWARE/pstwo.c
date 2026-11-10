#include "pstwo.h"

//����ps2�ֱ�ö��
#include "WiredPS2_gamepad.h"

#define DELAY_TIME  delay_us(5); 

static GamePadKeyStateType_t ClassicPS2_GetKeyState(uint8_t bit);

//����PS2�ֱ�
GamePadType_t Classic_PS2Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 0,
	.SetVibration = NULL,
	.getKeyState = ClassicPS2_GetKeyState
};

//
__weak void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	printf("%d,%d\r\n",keyid,event);
}


//Button value reading, zero time storage
//����ֵ��ȡ����ʱ�洢
u16 Handkey;	
//Start the order. Request data
//��ʼ�����������
u8 Comd[2]={0x01,0x42};	
//Data store array
//���ݴ洢����
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
�������ܣ�PS2�ֱ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
		//The task is run at 100hz
		//��������100Hz��Ƶ������ 	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		if(SysVal.HardWare_Ver==V1_0)//�ɿ�ps2�ֱ���ȡ
		{
			PS2_Read();
		}
		else if( SysVal.HardWare_Ver==V1_1 )
		{
			vTaskDelete(NULL);//�¿�usb�ӿ�ps2�ֱ����ڴ�����ִ��
		}
		
		//��Ϸ�ֱ�ģʽ����
		if( GamePadInterface->StartFlag == 1 && PS2_ON_Flag==0 && SysVal.Time_count>=CONTROL_DELAY )
		{
			PS2_ON_Flag=1,Remote_ON_Flag=0,APP_ON_Flag=0,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0; 
		}
    }
}  

//��ȡ�ֱ���ֵ״̬����
static GamePadKeyStateType_t ClassicPS2_GetKeyState(uint8_t bit)
{
	if( (Handkey>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}


//���尴������¼��ĸ�������ֵ
static GamePad_CheckEventType_t GamePadKeyCheckEvent[16] = { 0 };

void PS2_Key_Param_Init(void)
{
	gamepad_brand = PS2_USB_Wiredless;
	GamePadInterface = &Classic_PS2Gamepad;
}

/**************************************************************************
Function: Ps2 handle initializer
Input   : none
Output  : none
�������ܣ�PS2�ֱ���ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//Enable the ability port clock
	//ʹ�ܶ˿�ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12;		  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;          
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;        
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;     
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;         
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	PS2_Key_Param_Init();
}
/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
�������ܣ���ȡPS2�ֱ��Ŀ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_Read(void)
{
	//Reading key
  //��ȡ������ֵ
	PS2_DataKey(); 
	//Read the analog of the remote sensing x axis on the left
  //��ȡ���ң��X�᷽���ģ����	
	Classic_PS2Gamepad.LX=PS2_AnologData(PSS_LX); 
	//Read the analog of the directional direction of remote sensing on the left
  //��ȡ���ң��Y�᷽���ģ����	
	Classic_PS2Gamepad.LY=255-PS2_AnologData(PSS_LY);
	//Read the analog of the remote sensing x axis
  //��ȡ�ұ�ң��X�᷽���ģ����  
	Classic_PS2Gamepad.RX=PS2_AnologData(PSS_RX);
	//Read the analog of the directional direction of the remote sensing y axis
  //��ȡ�ұ�ң��Y�᷽���ģ����  
	Classic_PS2Gamepad.RY=PS2_AnologData(PSS_RY);  

	//LT��RT��ֵ
	if( ClassicPS2_GetKeyState(PS2KEY_L2)!=0 ) Classic_PS2Gamepad.LT = 255;
	else  Classic_PS2Gamepad.LT = 0;
	
	if( ClassicPS2_GetKeyState(PS2KEY_R2)!=0 ) Classic_PS2Gamepad.RT = 255;
	else  Classic_PS2Gamepad.RT = 0;
	
	//�����ص���������
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(Handkey,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//�����ص�����
		Classic_PS2GamePad_KeyEvent_Callback(key,event);
	}
	
}
/**************************************************************************
Function: Send commands to the handle
Input   : none
Output  : none
�������ܣ����ֱ���������
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;     //Output a control bit //���һλ����λ
		}
		else DO_L;

		CLK_H;      //Clock lift //ʱ������
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(16);
}
/**************************************************************************
Function: Whether it is a red light mode, 0x41= analog green light, 0x73= analog red light
Input   : none
Output  : 0: red light mode, other: other modes
�������ܣ��ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
��ڲ�������
����  ֵ��0�����ģʽ������������ģʽ
**************************************************************************/	
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders //��ʼ����
	PS2_Cmd(Comd[1]);  //Request data //��������
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/**************************************************************************
Function: Read the handle data
Input   : none
Output  : none
�������ܣ���ȡ�ֱ�����
��ڲ�������
����  ֵ����
**************************************************************************/	
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders //��ʼ����
	PS2_Cmd(Comd[1]);  //Request data //��������
	for(byte=2;byte<9;byte++) //Start receiving data //��ʼ��������
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(16);
	}
	CS_H;
}
/**************************************************************************
Function: Handle the data of the read 2 and handle only the key parts
Input   : none
Output  : 0: only one button presses the next time; No press
�������ܣ��Զ�������PS2�����ݽ��д���,ֻ������������ 
��ڲ�������
����  ֵ��0: ֻ��һ����������ʱ����; 1: δ����
**************************************************************************/	
u8 ps2_default_buf[9] = {0,0x73,0,0xff,0xff,128,128,128,128};

u8 PS2_DataKey()
{
	static u8 last_ps2_data[9] = {0,0,0,0xff,0xff,128,128,128,128};
	static u8 ps2_connect_timeout = 0;
	
	//��ȡPS2�ֱ�����
    PS2_ClearData();
    PS2_ReadData();
	
	//����ps2����ģʽ(0x41)������ps2�ֱ�����ģʽ(0x20),�������ģʽ�ֱ��������Ȳ���
	//255Ϊ�쳣����,�����ֱ�ֻ�����˲��ֶ�û����ȫ����ɶ���������
	if( Data[1]==0x41 || Data[1]==0x20 || Data[1]==255 )
	{
		u8 save = Data[1];
		PS2_SetInit();//��ʼ��Ϊģ��ģʽ
		memcpy(Data,ps2_default_buf,9);//��һ������ʹ��ģʽ����,��ֹʧ��
		Data[1] = save;//�����ε�ֵд��,��ֹ��ʼ�����ɹ����������˸��ж�
	}
	
	//�������������ݶ���0���������һ�ε�����(ȫ0��ȫ1,���������õĳ�ʼ���������й�)
	u8 a=0;
	for(u8 i=0;i<9;i++)
	{
		a+=Data[i];
	}
	if(a==0) //����ȫ0˵���ֱ����γ�
	{
		//��״̬�¿ɵ���ps2�ֱ�δ����,����һ�����������ps2����ֵ
		ps2_connect_timeout++;
		if( ps2_connect_timeout> 10 ) 
		{   //�ֱ����ӳ�ʱ,ʹ��Ĭ�ϱ���ֹͣС������
			ps2_connect_timeout = 11, memcpy(last_ps2_data,ps2_default_buf,9);
		}			
		
		//ȫ0ʱʹ����һ�ε�����(���а���ͬʱ���²�Ϊ0)
		memcpy(Data,last_ps2_data,9);
	}
	else
	{
		ps2_connect_timeout = 0;//��ʱ��λ
	}
	
	//���汾������
	memcpy(last_ps2_data,Data,9); 
	
	//This is 16 buttons, pressed down to 0, and not pressed for 1 
	//����16������������Ϊ0��δ����Ϊ1
    Handkey=(Data[4]<<8)|Data[3]; 
	
	//ת��Ϊ ����Ϊ1,δ����Ϊ0����Ӧusb ps2���ݸ�ʽ��
	Handkey = ~Handkey;
	
	//���ؼ�ֵ���
	return Handkey;
}
/**************************************************************************
Function: Get a simulation of a rocker
Input   : Rocker
Output  : Simulation of rocker, range 0~ 256
�������ܣ��õ�һ��ҡ�˵�ģ����
��ڲ�����ҡ��
����  ֵ��ҡ�˵�ģ����, ��Χ0~256
**************************************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}
/**************************************************************************
Function: Clear data buffer
Input   : none
Output  : none
�������ܣ�������ݻ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function: Handle vibration function
Input   : motor1: the right small vibrator, 0x00, other
          motor2: the left big shock motor 0x40~ 0xff motor is open, and the value is greater
Output  : none
�������ܣ��ֱ��𶯺���
��ڲ�����motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	        motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
����  ֵ����
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	delay_us(16);
  PS2_Cmd(0x01); //Start order //��ʼ����
	PS2_Cmd(0x42); //Request data //��������
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);  
}
/**************************************************************************
Function: Short press
Input   : none
Output  : none
�������ܣ��̰�
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Enter configuration
Input   : none
Output  : none
�������ܣ���������
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_EnterConfing(void)
{
  CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Send mode Settings
Input   : none
Output  : none
�������ܣ�����ģʽ����
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  Software Settings send mode �������÷���ģʽ
	PS2_Cmd(0x03); //0x03 lock storage setup, which cannot be set by the key "mode" set mode. //0x03�������ã�������ͨ��������MODE������ģʽ��
				         //0xee non-locking software Settings can be set by the key "mode" set mode.//0xEE�������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Vibration setting
Input   : none
Output  : none
�������ܣ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Complete and save the configuration
Input   : none
Output  : none
�������ܣ���ɲ���������
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Handle configuration initialization
Input   : none
Output  : none
�������ܣ��ֱ����ó�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		  //Enter configuration mode //��������ģʽ
	PS2_TurnOnAnalogMode();	//The "traffic light" configuration mode and select whether to save //�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	//PS2_VibrationMode();	//Open vibration mode //������ģʽ
	PS2_ExitConfing();		  //Complete and save the configuration //��ɲ���������
}
/**************************************************************************
Function: Read the handle information
Input   : none
Output  : none
�������ܣ���ȡ�ֱ���Ϣ
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_Receive (void)
{
	if(PS2_ON_Flag)
	{
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_RX=PS2_AnologData(PSS_RX);
		PS2_RY=PS2_AnologData(PSS_RY);
	}
	PS2_KEY=PS2_DataKey();
}

