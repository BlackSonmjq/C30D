#include "show.h"
#include "distance_task.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;


u8 oled_refresh_flag;
u8 oled_page=DISTANCE_OLED_PAGE;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½È¡ï¿½ï¿½Øµï¿½Ñ¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¼ì¡¢ï¿½ï¿½APPï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý¡ï¿½OLEDï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½Ú²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½ï¿½  Öµï¿½ï¿½ï¿½ï¿½
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½10Hzï¿½ï¿½Æµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		
		//ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		//The buzzer will beep briefly when the machine is switched on
		if(SysVal.Time_count<50)Buzzer_Set(1);
		else if(SysVal.Time_count>=51 && SysVal.Time_count<100)Buzzer_Set(0);
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(LowVoltage_2>=2 && LowVoltage_2%10==2)Buzzer_count=0;  // Ã¿10ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer_Set(1); //The buzzer is buzzing //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		else if(Buzzer_count==5)Buzzer_Set(0);
		
		//Read the battery voltage //ï¿½ï¿½È¡ï¿½ï¿½Øµï¿½Ñ¹
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All=0;
		
		if(Get_Charging_HardWare==1)
		{	//ï¿½ï¿½Ñ¹ï¿½ï¿½ï¿½Í£ï¿½ï¿½Õµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÅºÅ£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¶ï¿½ï¿½Ø³ä¹¦ï¿½ï¿½
			if(Voltage<10&&RED_STATE) Allow_Recharge=1;
		}
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //È·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö»ï¿½ï¿½0.5ï¿½ï¿½
		if(LowVoltage_2>=1)LowVoltage_2++; // Ö»Òª>=1ï¿½ï¿½ï¿½Û¼ï¿½
		
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5Vï¿½ï¿½ï¿½Íµï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Î±ï¿½ï¿½ï¿½
		else if(Voltage<10 && LowVoltage_2==0)LowVoltage_2=1; //Ö»ï¿½Úµï¿½ï¿½ï¿½0Ê±ï¿½ï¿½Ê¼ï¿½ï¿½
					
		APP_Show();	 //Send data to the APP //ï¿½ï¿½APPï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		
		if(oled_refresh_flag) OLED_Clear(),oled_refresh_flag=0;
		else oled_show(); //Tasks are displayed on the screen //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½
   }
}

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ü£ï¿½OLEDï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½Ú²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½ï¿½  Öµï¿½ï¿½ï¿½ï¿½
**************************************************************************/
extern volatile u8 clear_state;

uint8_t show_usb_enum = 0;
void OLED_ShowGamePadState(void)
{
	show_usb_enum = 1;
}

void oled_show(void)
{   
	int Car_Mode_Show;
	
	//Collect the tap information of the potentiometer, 
		 //and display the car model to be fitted when the car starts up in real time
		 //ï¿½É¼ï¿½ï¿½ï¿½Î»ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½Ï¢ï¿½ï¿½ÊµÊ±ï¿½ï¿½Ê¾Ð¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±Òªï¿½ï¿½ï¿½ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½Íºï¿½
		 Divisor_Mode=2048/CAR_NUMBER+5;
		POT_val = Get_adc_Average(Potentiometer,10);
		 Car_Mode_Show=(int) (POT_val/Divisor_Mode); 
		 if(Car_Mode_Show>7)Car_Mode_Show=7; 
		  //Car_Mode_Show=0;
	
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAMï¿½ï¿½ï¿½ãµ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë¢ï¿½Â£ï¿½ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½
	if(Check==0)
	{
		if( show_usb_enum == 1 )
		{
			OLED_DrawBMP(22,2,22+84,2+5,gImage_usb);
			static u8 show_delay=0;
			if(++show_delay<RATE_10_HZ*2)
			{
				return;
			}
			else 
			{
				show_delay=0;
				show_usb_enum = 0;
			}
		}
//		if( show_usb_enum && GamePadDebug.enmu_state == EnumDone)
//		{
//			if(clear) 
//			{
//				clear=0,oled_refresh_flag=1; //Ã¶ï¿½ï¿½ï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï¢
//				return;//Ö´ï¿½ï¿½1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//			}
//			OLED_DrawBMP(32,1,96,7,gImage_usb_bmp);
//			OLED_ShowString(12,50,"USB Init..");
//			OLED_Refresh_Line();
//			show_done = 1;
//			return;
//		}
//		else if( show_usb_enum && show_done )
//		{
//			if(show_done) //Ã¶ï¿½Ù³É¹ï¿½,ï¿½ï¿½Ê±ï¿½ï¿½Ê¾1ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½Ï¢
//			{
//				static u8 show_delay=0;
//				if(++show_delay<RATE_10_HZ)
//				{
//					OLED_ShowString(12,50,"USB Init OK   ");
//					OLED_Refresh_Line();
//					return;
//				}
//				show_done = 0;
//				show_delay=0;	
//				oled_refresh_flag = 1;//ï¿½Ó³ï¿½ï¿½ï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//				show_usb_enum = 0;
//			}
//			clear=1; //Ã¶ï¿½ï¿½ï¿½ï¿½ï¿½,ï¿½È´ï¿½ï¿½Â´ï¿½Ã¶ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//		}
		///////////// usb ps2 ï¿½è±¸ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ /////////////
	
	if(oled_page==0)
	{
		 Voltage_Show=Voltage*100; 
		memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAMï¿½ï¿½ï¿½ãµ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ë¢ï¿½Â£ï¿½ï¿½ï¿½Ö¹ï¿½ï¿½ï¿½ï¿½
		 if(Check==0)//The car displays normally when the self-check mode is not enabled //Ã»ï¿½Ð¿ï¿½ï¿½ï¿½ï¿½Ô¼ï¿½Ä£Ê½Ê±Ð¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾
		 {	
			 //The first line of the display displays the content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 if(Allow_Recharge) OLED_ShowString(0,0,"RCM ");
			 else
			 {
				 switch(Car_Mode_Show)
				 {
					case Mec_Car:       OLED_ShowString(0,0,"Mec  "); break; 
					case Omni_Car:      OLED_ShowString(0,0,"Omni "); break; 
					case Akm_Car:       OLED_ShowString(0,0,"Akm  "); break; 
					case Diff_Car:      OLED_ShowString(0,0,"Diff "); break; 
					case FourWheel_Car: OLED_ShowString(0,0,"4WD  "); break; 
					case Tank_Car:      OLED_ShowString(0,0,"Tank "); break; 
					case Mec_Car_V550:  OLED_ShowString(0,0,"Mec-V"); break;
					case FourWheel_Car_V550: OLED_ShowString(0,0,"4WD-V"); break;
				 } 
			 }
			 
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
			 {
				 //The Mec_car and omni_car show Z-axis angular velocity
				 //ï¿½ï¿½ï¿½Ö¡ï¿½È«ï¿½ï¿½ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½ï¿½Ê¾Zï¿½ï¿½ï¿½ï¿½Ù¶ï¿½
				if(Charging)
				{	//Ð¡ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
					OLED_ShowString(80,0," ");
					OLED_ShowString(85,0,"   ");
					OLED_ShowString(60,0,"Cur:");
					oled_showfloat(Charging_Current/1000.0f,90,0,1,2);
				}
				else
				{	//Ð¡ï¿½ï¿½ï¿½Ç³ï¿½ï¿½×´Ì¬ï¿½ï¿½Ê¾Zï¿½ï¿½ï¿½ï¿½Ù¶ï¿½
					//Display Z-axis angular velocity //ï¿½ï¿½Ê¾Zï¿½ï¿½ï¿½ï¿½Ù¶ï¿½
					OLED_ShowString(76,0,"    ");
					OLED_ShowString(120,0," ");
					OLED_ShowString(60,0,"GZ");
					if( imu.gyro.z<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-imu.gyro.z,5,12);
					else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, imu.gyro.z,5,12);		
				}
			 }
			 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car_V550)
			 {
				 //ps2ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾
//				 if(usb_ps2_ready==1)
//				 {
//					OLED_ShowString(55,0,"ps2 ready  ");
//				 }
//				 else
//				 {
//					 OLED_ShowString(55,0,"ps2 outline");
//				 }
//	
				 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
				 //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
				 OLED_ShowString(55,0,"BIAS");
				 if( imu.Deviation_accel.z<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-imu.Deviation_accel.z,3,12);  //Zero-drift data of gyroscope Z axis
				 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, imu.Deviation_accel.z,3,12);	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½zï¿½ï¿½ï¿½ï¿½ï¿½Æ¯ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½	
			 }
			 //The first line of the display displays the content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 

			 //The second line of the display displays the content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½2ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car||Car_Mode==Mec_Car_V550||Car_Mode==FourWheel_Car_V550)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
				//ï¿½ï¿½ï¿½Ö¡ï¿½È«ï¿½ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Aï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ÈºÍµï¿½Ç°Êµï¿½ï¿½ï¿½Ù¶ï¿½
				OLED_ShowString(0,10,"A");
				if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
															OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
				else                 	OLED_ShowString(15,10,"+"),
															OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
				
				if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
															OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,10,"+"),
															OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
				 //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¡ï¿½Ì¹ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½ï¿½Ê¾Zï¿½ï¿½ï¿½ï¿½Ù¶ï¿½
				 OLED_ShowString(00,10,"GYRO_Z:");
				 if( imu.gyro.z<0)  OLED_ShowString(60,10,"-"),
												 OLED_ShowNumber(75,10,-imu.gyro.z,5,12);
				 else            OLED_ShowString(60,10,"+"),
												 OLED_ShowNumber(75,10, imu.gyro.z,5,12);			
			 }	 
			 //The second line of the display displays the content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½2ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 
			 //Lines 3 and 4 of the display screen display content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½3ï¿½ï¿½4ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550||Car_Mode==Mec_Car_V550)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
				//ï¿½ï¿½ï¿½Ö¡ï¿½È«ï¿½ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Bï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ÈºÍµï¿½Ç°Êµï¿½ï¿½ï¿½Ù¶ï¿½
				OLED_ShowString(0,20,"B");		
				if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
				else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
				
				if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
				
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
				//ï¿½ï¿½ï¿½Ö¡ï¿½È«ï¿½ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Cï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ÈºÍµï¿½Ç°Êµï¿½ï¿½ï¿½Ù¶ï¿½
				OLED_ShowString(0,30,"C");
				if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
				else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
					
				if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
				 //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¡ï¿½ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Aï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ÈºÍµï¿½Ç°Êµï¿½ï¿½ï¿½Ù¶ï¿½
				 OLED_ShowString(0,20,"L:");
				 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
																OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
				 else                 	OLED_ShowString(15,20,"+"),
																OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
				 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
																OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,20,"+"),
																OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
				 //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¡ï¿½ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Bï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ÈºÍµï¿½Ç°Êµï¿½ï¿½ï¿½Ù¶ï¿½
				 OLED_ShowString(0,30,"R:");
				 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
																OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
				 else                 	OLED_ShowString(15,30,"+"),
																OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
					
				 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
																OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,30,"+"),
																OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

	//			 if( Remoter_Ch1<0)	    OLED_ShowString(15,20,"-"),
	//															OLED_ShowNumber(20,20,-Remoter_Ch1,5,12);
	//			 else                 	OLED_ShowString(15,20,"+"),
	//															OLED_ShowNumber(20,20, Remoter_Ch1,5,12);  
	//			 if( Remoter_Ch2<0)	    OLED_ShowString(60,20,"-"),
	//															OLED_ShowNumber(75,20,-Remoter_Ch2,5,12);
	//			 else                 	OLED_ShowString(60,20,"+"),
	//															OLED_ShowNumber(75,20, Remoter_Ch2,5,12);
	//			 if( Remoter_Ch3<0)	    OLED_ShowString(15,30,"-"),
	//															OLED_ShowNumber(20,30,-Remoter_Ch3,5,12);
	//			 else                 	OLED_ShowString(15,30,"+"),
	//															OLED_ShowNumber(20,30, Remoter_Ch3,5,12);  
	//			 if( Remoter_Ch4<0)	    OLED_ShowString(60,30,"-"),
	//															OLED_ShowNumber(75,30,-Remoter_Ch4,5,12);
	//			 else                 	OLED_ShowString(60,30,"+"),
	//															OLED_ShowNumber(75,30, Remoter_Ch4,5,12);
			 }
			 //Lines 3 and 4 of the display screen display content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½3ï¿½ï¿½4ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 
			 //Line 5 of the display displays the content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½5ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
			 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car||Car_Mode==Mec_Car_V550||Car_Mode==FourWheel_Car_V550)
			 {
				  //Mec_Car Display the target speed and current actual speed of motor D
					//ï¿½ï¿½ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Dï¿½ï¿½Ä¿ï¿½ï¿½ï¿½Ù¶ÈºÍµï¿½Ç°Êµï¿½ï¿½ï¿½Ù¶ï¿½
					OLED_ShowString(0,40,"D");
					if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
																OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
					else                 	OLED_ShowString(15,40,"+"),
																OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
					if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
																OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
					else                 	OLED_ShowString(60,40,"+"),
																OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Omni_Car)
			 {
				  // The Omni_car shows Z-axis angular velocity (1000 times magnification) in rad/s
					//È«ï¿½ï¿½ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½ï¿½Ê¾Zï¿½ï¿½ï¿½ï¿½Ù¶ï¿½(ï¿½Å´ï¿½1000ï¿½ï¿½)ï¿½ï¿½ï¿½ï¿½Î»rad/s
					OLED_ShowString(0,40,"MOVE_Z"); 			
					if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																							OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
					else                              	OLED_ShowString(60,40,"+"),
																							OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
			 }
			 else if(Car_Mode==Akm_Car)
			 {
					OLED_ShowString(00,40,"Akm_Car");
			 }
			 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
				 //ï¿½ï¿½ï¿½ï¿½Ð¡ï¿½ï¿½ï¿½ï¿½ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Òµï¿½ï¿½ï¿½ï¿½PWMï¿½ï¿½ï¿½ï¿½Öµ
																 OLED_ShowString(00,40,"MA");
				 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
																 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(20,40,"+"),
																 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
																 OLED_ShowString(60,40,"MB");
				 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
																 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(80,40,"+"),
																 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
			 }
			 //Line 5 of the display displays the content//
			 //ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½5ï¿½ï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½//
				 
			 //Displays the current control mode //ï¿½ï¿½Ê¾ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½Ä£Ê½
			 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
			 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
			 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
			 else if ((Usart1_ON_Flag || Usart5_ON_Flag)==1) OLED_ShowString(0,50,"USART");
				else if (nav_walk==1)      OLED_ShowString(0,50,"nav");
				else                       OLED_ShowString(0,50,"ROS  ");
				
			 //Displays whether controls are allowed in the current car
			 //ï¿½ï¿½Ê¾ï¿½ï¿½Ç°Ð¡ï¿½ï¿½ï¿½Ç·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
			 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
			 else                      OLED_ShowString(45,50,"OFF"); 
				
																	OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
										  OLED_ShowString(88,50,".");
																	OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
										  OLED_ShowString(110,50,"V");
			 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
			}	
		 
			OLED_Refresh_Gram();
		}
	
        else if(oled_page==DISTANCE_OLED_PAGE)
        {
            DistanceTask_DrawOLED();
            OLED_Refresh_Gram();
        }
		else if(oled_page==1)
		{
			//ï¿½Ô¶ï¿½ï¿½Ø³ï¿½ï¿½×¼ï¿½Debugï¿½ï¿½Ï¢
			OLED_ShowString(07,00,"LA  LB  RB  RA");
			OLED_ShowNumber(0+9,10,L_A,1,12);
			OLED_ShowNumber(30+9,10,L_B,1,12);
			OLED_ShowNumber(60+9,10,R_B,1,12);
			OLED_ShowNumber(90+9,10,R_A,1,12);
			OLED_ShowString(0,30,"cur:");
			OLED_ShowString(75,30,"A");
			oled_showfloat(Charging_Current/1000.0f,30,30,2,2);
			//IPï¿½ï¿½Ê¾

			OLED_ShowString(0, 50,"IP:");
			OLED_ShowNumber(24,50,Received_IP[0],3,12);
			OLED_ShowNumber(48,50,Received_IP[1],3,12);
			OLED_ShowNumber(72,50,Received_IP[2],3,12);
			OLED_ShowNumber(96,50,Received_IP[3],3,12);
			OLED_Refresh_Gram();	
		}
	}
	else
	 {
		 if(check_parameter.Process==0)							//ï¿½Ã»ï¿½ï¿½Ô¼ï¿½ï¿½ï¿½ï¿½
		 {
			 OLED_ShowCHinese(00, 00, "ÇëÏÈÔÄ¶ÁÊ¹ÓÃÊÖ²á");
			 OLED_ShowCHinese(00, 16, "  µ¥»÷ÓÃ»§°´¼ü  ");
			 OLED_ShowCHinese(00, 32, "    È·ÈÏ¿ªÊ¼    ");
			 OLED_ShowString(104, 50, "0/8");
		 }
		 
		 if(check_parameter.Process==1)					
		 {
			 OLED_Show_POT();
			 OLED_ShowCHinese12(8, 9, "Çë×ª¶¯µçÎ»Æ÷£º");
			 switch(Car_Mode_Show)
			 {
				case Mec_Car:       OLED_ShowString(92,10,"Mec "); break;
				case Omni_Car:      OLED_ShowString(92,10,"Omni"); break;
				case Akm_Car:       OLED_ShowString(92,10,"Akm "); break;
				case Diff_Car:      OLED_ShowString(92,10,"Diff"); break;
				case FourWheel_Car: OLED_ShowString(92,10,"4WD "); break;
				case Tank_Car:      OLED_ShowString(92,10,"Tank"); break;
				case Mec_Car_V550:  OLED_ShowString(92,10,"MecV"); break;
				case FourWheel_Car_V550: OLED_ShowString(92,10,"4WDV"); break;
			 }
			 OLED_ShowCHinese12(12, 22, "ÍË³öÇ°Ðèµ÷»ØÔ­³µÐÍ");
			 OLED_ShowCHinese(00, 34, "  Ä£Ê½£ºµçÎ»Æ÷  ");
			 OLED_ShowString(104, 50, "1/8");
		 }
		 if(check_parameter.Process==2)
		 {
			 OLED_ShowCHinese(24, 00, "¼Ü¿Õ³µÂÖºó");
			 OLED_ShowCHinese(00, 16, "µ¥»÷¿ªÊ¼¸ßËÙ×ª¶¯");
			 OLED_ShowCHinese(24, 32, "Ä£Ê½£ºµç»ú");
			 OLED_ShowString(104, 50, "2/8");
		 }
		 if(check_parameter.Process==3)
		 {
				if( MOTOR_A.Encoder<0)	OLED_ShowString(00,5,"A-"),
																OLED_ShowNumber(20,5,-MOTOR_A.Encoder*1000,5,12);
				else                		OLED_ShowString(00,5,"A+"),
																OLED_ShowNumber(20,5, MOTOR_A.Encoder*1000,5,12);
				
				if( MOTOR_B.Encoder<0)	OLED_ShowString(70,5,"B-"),
																OLED_ShowNumber(90,5,-MOTOR_B.Encoder*1000,5,12);
				else                		OLED_ShowString(70,5,"B+"),
																OLED_ShowNumber(90,5, MOTOR_B.Encoder*1000,5,12);
				
				if( MOTOR_C.Encoder<0)	OLED_ShowString(00,20,"C-"),
																OLED_ShowNumber(20,20,-MOTOR_C.Encoder*1000,5,12);
				else                		OLED_ShowString(00,20,"C+"),
																OLED_ShowNumber(20,20, MOTOR_C.Encoder*1000,5,12);
				
				if( MOTOR_D.Encoder<0)	OLED_ShowString(70,20,"D-"),
																OLED_ShowNumber(90,20,-MOTOR_D.Encoder*1000,5,12);
				else                		OLED_ShowString(70,20,"D+"),
																OLED_ShowNumber(90,20, MOTOR_D.Encoder*1000,5,12);
				OLED_ShowCHinese(24, 32, "Ä£Ê½£ºµç»ú");
				OLED_ShowString(104, 50, "2/8");
		 }
		 if(check_parameter.Process==4)
		 {
				OLED_ShowCHinese(00, 00, "    µ¥»÷½øÈë    ");
				OLED_ShowCHinese(00, 32, "  Ä£Ê½£ºÍÓÂÝÒÇ  ");
			  OLED_ShowString(104, 50, "3/8");
		 }
		 if(check_parameter.Process==5)
		 {
			 OLED_ShowCHinese(00, 00, "»Î¶¯Ð¡³µ¹Û²ìÊý¾Ý");
			 OLED_ShowString(00, 16, "G");
			 if(imu.gyro.x<0)							 OLED_ShowString(16, 16, "-"),
																	 OLED_ShowNumber(24, 16, -imu.gyro.x, 5, 12);
			 else							 					 OLED_ShowString(16, 16, "+"),
																	 OLED_ShowNumber(24, 16, imu.gyro.x, 5, 12);
			 
			 if(imu.gyro.y<0)							 OLED_ShowString(16, 26, "-"),
																	 OLED_ShowNumber(24, 26, -imu.gyro.y, 5, 12);
			 else							 					 OLED_ShowString(16, 26, "+"),
																	 OLED_ShowNumber(24, 26, imu.gyro.y, 5, 12);
			 
			 if(imu.gyro.z<0)	 						OLED_ShowString(16, 36, "-"),
																	OLED_ShowNumber(24, 36, -imu.gyro.z, 5, 12);
			 else						 						OLED_ShowString(16, 36, "+"),
																	OLED_ShowNumber(24, 36, imu.gyro.z, 5, 12);
			 
			 OLED_ShowString(64, 16, "|");
			 OLED_ShowString(64, 26, "|");
			 OLED_ShowString(64, 36, "|");
			 
			 OLED_ShowString(72, 16, "A");
			 if(imu.accel.x<0)							 OLED_ShowString(80, 16, "-"),
																	 OLED_ShowNumber(88, 16, -imu.accel.x, 5, 12);
			 else							 					 OLED_ShowString(80, 16, "+"),
																	 OLED_ShowNumber(88, 16, imu.accel.x, 5, 12);
			 
			 if(imu.accel.y<0)							 OLED_ShowString(80, 26, "-"),
																	 OLED_ShowNumber(88, 26, -imu.accel.y, 5, 12);
			 else							 					 OLED_ShowString(80, 26, "+"),
																	 OLED_ShowNumber(88, 26, imu.accel.y, 5, 12);
			 
			 if(imu.accel.z<0)							 OLED_ShowString(80, 36, "-"),
																	 OLED_ShowNumber(88, 36, -imu.accel.z, 5, 12);
			 else							 					 OLED_ShowString(80, 36, "+"),
																	 OLED_ShowNumber(88, 36, imu.accel.z, 5, 12);
			 
			 OLED_ShowString(104, 50, "3/8");
		 }

		 if(check_parameter.Process==14)
		 {
			 OLED_ShowCHinese(24, 16, "·äÃùÆ÷£º");
			 if(Buzzer==1)		OLED_ShowCHinese(88, 16, "¿ª");
			 else							OLED_ShowCHinese(88, 16, "¹Ø");
			 OLED_ShowCHinese(00, 32, "  Ä£Ê½£º·äÃùÆ÷  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(check_parameter.Process==15)
		 {
			 OLED_ShowCHinese(00, 00, "  ²¦¶¯Ê¹ÄÜ¿ª¹Ø  ");
			 OLED_ShowCHinese(32, 16, "¿ª¹Ø£º");
			 if(EN==1)					OLED_ShowCHinese(80, 16, "¿ª");
			 else								OLED_ShowCHinese(80, 16, "¹Ø");
			 OLED_ShowCHinese(8, 32, "Ä£Ê½£ºÊ¹ÄÜ¿ª¹Ø");
			 OLED_ShowString(104, 50, "6/8");
		 }
		 if(check_parameter.Process==16)
		 {
			  OLED_ShowCHinese(00, 00, "È·ÈÏÀ¶ÑÀºìµÆ³£ÁÁ");
			  OLED_ShowCHinese(24, 16, "ºóµ¥»÷½øÈë");
				OLED_ShowCHinese(24, 32, "Ä£Ê½£ºÀ¶ÑÀ");
			  OLED_ShowString(104, 50, "7/8");
		 }
		 if(check_parameter.Process==17)
		 {
			 OLED_ShowCHinese(8, 00, "Ð¡³µÐÐ¶¯×´Ì¬£º");
			 if(check_parameter.Usart2_check_dir==1)				OLED_ShowCHinese(48, 16, "ÏòÇ°");
			 else if(check_parameter.Usart2_check_dir==2)		OLED_ShowCHinese(48, 16, "ÓÒÉÏ");
			 else if(check_parameter.Usart2_check_dir==3)		OLED_ShowCHinese(48, 16, "ÏòÓÒ");
			 else if(check_parameter.Usart2_check_dir==4)		OLED_ShowCHinese(48, 16, "ÓÒÏÂ");
			 else if(check_parameter.Usart2_check_dir==5)		OLED_ShowCHinese(48, 16, "Ïòºó");
			 else if(check_parameter.Usart2_check_dir==6)		OLED_ShowCHinese(48, 16, "×óÏÂ");
			 else if(check_parameter.Usart2_check_dir==7)		OLED_ShowCHinese(48, 16, "Ïò×ó");
			 else if(check_parameter.Usart2_check_dir==8)		OLED_ShowCHinese(48, 16, "×óÉÏ");
			 else if(check_parameter.Usart2_check_dir==9)		OLED_ShowCHinese(48, 16, "Í£Ö¹");
			 OLED_ShowCHinese(24, 32, "Ä£Ê½£ºÀ¶ÑÀ");
			 OLED_ShowString(104, 50, "7/8");
		 }
		 if(check_parameter.Process==18)
		 {
				OLED_ShowCHinese(8, 00, "Ê¹ÓÃÊý¾ÝÏßÁ¬½Ó");
			  OLED_ShowCHinese(20, 16, "ÖÐ¼äµÄ");
			  OLED_ShowString16(68, 16, "USB");
				OLED_ShowCHinese(92,16, "¿Ú");
				OLED_ShowCHinese(20, 32, "Ä£Ê½£º´®¿Ú");
			  OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(check_parameter.Process==19)
		 {
			 OLED_ShowCHinese(00, 00, "·¢ËÍ");
			 OLED_ShowString16(36, 00, "#");
			 OLED_ShowCHinese(48, 00, "½áÎ²µÄÊý¾Ý");
			 
			 OLED_ShowCHinese(8, 16, "²¨ÌØÂÊ£º");
			 OLED_ShowString16(72, 16, "115200");
//			 OLED_ShowCHinese(00, 00, "ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½");
//			 OLED_ShowString16(64, 00, "WHEELTEC");
//			 OLED_ShowCHinese(00, 16, "ï¿½ï¿½ï¿½Õµï¿½");
//			 OLED_ShowString16(48, 16, "ASCII:");
//			 OLED_ShowNumber(96, 18, (USART3->DR), 4, 12);		//ï¿½Ú´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ï·ï¿½ï¿½Íµï¿½ï¿½ï¿½ï¿½Ö·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½Asciiï¿½ï¿½ï¿½ï¿½Ê¾
			 OLED_ShowCHinese(20, 32, "Ä£Ê½£º´®¿Ú");
			 OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(check_parameter.Process==20)
		 {
				OLED_ShowCHinese(00, 00, "¼´½«ÍË³öÖÊ¼ìÄ£Ê½");
			  OLED_ShowCHinese(00, 16, "Çë×ª¶¯µçÎ»Æ÷»Øµ½");
			  OLED_ShowCHinese(12, 32, "µ±Ç°³µÐÍ£º");
			  switch(Car_Mode_Show)
			  {
					case Mec_Car:       OLED_ShowString16(92,32,"Mec "); break; 
					case Omni_Car:      OLED_ShowString16(92,32,"Omni"); break; 
					case Akm_Car:       OLED_ShowString16(92,32,"Akm "); break; 
					case Diff_Car:      OLED_ShowString16(92,32,"Diff"); break; 
					case FourWheel_Car: OLED_ShowString16(92,32,"4WD "); break; 
					case Tank_Car:      OLED_ShowString16(92,32,"Tank"); break; 
					case Mec_Car_V550:  OLED_ShowString(92,32,"MecV"); break;
					case FourWheel_Car_V550: OLED_ShowString(92,32,"4WDV"); break;
			  }
				OLED_ShowString(104, 50, "8/8");
		 }
		 OLED_ShowCHinese12(00, 50, "µ¥»÷¼ÌÐøË«»÷ÍË³ö");
		 OLED_Refresh_Gram();
	 }
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½APPï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½Ú²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½ï¿½  Öµï¿½ï¿½ï¿½ï¿½
**************************************************************************/
int voltage_to_percentage(float voltage) {
    const float V_MIN = 9.5f;  // ï¿½ï¿½Ð¡ï¿½ï¿½Ñ¹ (0%)
    const float V_MAX = 12.2f; // ï¿½ï¿½ï¿½ï¿½Ñ¹ (100%)
    
    // È·ï¿½ï¿½ï¿½ï¿½Ñ¹ï¿½ï¿½ï¿½ï¿½Ð§ï¿½ï¿½Î§ï¿½ï¿½
    if (voltage < V_MIN) return 0;
    if (voltage > V_MAX) return 100;
    
    // ï¿½ï¿½ï¿½Ô²ï¿½Öµï¿½ï¿½ï¿½ï¿½Ù·Ö±È²ï¿½È¡ï¿½ï¿½
    int percentage = (int)(((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0f + 0.5f); // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	
	if( percentage > 100 ) percentage = 100;
	if( percentage <= 0 ) percentage = 0;
	
    return percentage;
}

void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //ï¿½Ôµï¿½Øµï¿½Ñ¹ï¿½ï¿½ï¿½ï¿½ï¿½É°Ù·Ö±ï¿½ï¿½ï¿½Ê½
	Voltage_Show = voltage_to_percentage(Voltage);
//	 Voltage_Show=(Voltage*1000-10000)/27;
//	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //ï¿½ï¿½ï¿½ï¿½ï¿½Ù¶Èµï¿½Î»×ªï¿½ï¿½Îª0.01m/sï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½APPï¿½ï¿½Ê¾
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //ï¿½ï¿½ï¿½Ú½ï¿½ï¿½ï¿½ï¿½Ó¡APPï¿½ï¿½ï¿½Ýºï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //ï¿½ï¿½ï¿½Í²ï¿½ï¿½ï¿½ï¿½ï¿½APPï¿½ï¿½APPï¿½Úµï¿½ï¿½Ô½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾
		 printf("{C%d:%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI,LineDiffParam);
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0) 
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //ï¿½ï¿½ï¿½Í²ï¿½ï¿½ï¿½ï¿½ï¿½APPï¿½ï¿½APPï¿½ï¿½ï¿½ï¿½Ò³ï¿½ï¿½Ê¾
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)imu.gyro.z);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //ï¿½ï¿½ï¿½Í²ï¿½ï¿½ï¿½ï¿½ï¿½APPï¿½ï¿½APPï¿½Ú²ï¿½ï¿½Î½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¾
		 printf("{B%d:%d:%d}$",(int)imu.gyro.x,(int)imu.gyro.y,(int)imu.gyro.z);
	 }
	 if(Debug_Flag)
	 {
		 if(IP_Frame_Valid)//ipï¿½ï¿½ï¿½ï¿½ï¿½Ñ¹Ø±ï¿½
			printf("{#IP:%d.%d.%d.%d}$",(u8)Received_IP[0],(u8)Received_IP[1],(u8)Received_IP[2],(u8)Received_IP[3]);//ï¿½Ï·ï¿½IP
		 else printf("{#IP:unknow..}$");//ipÎ´Öª
		 Debug_Flag=0;
	 }
}

float base_vol=11.5f;
#define VOL_COUNT 100
float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};
	
	/*----------- ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ -----------*/
	static u8 once=1;
	if(once)
	{
		once=0;
		for(i=0;i<VOL_COUNT;i++)
			Speed_Buf[i]=base_vol;
	}
	/*-------------------------------*/
	
    for(i = 1 ; i<VOL_COUNT; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[VOL_COUNT - 1] =data;

    for(i = 0 ; i < VOL_COUNT; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
    return Filter_Speed;
}
