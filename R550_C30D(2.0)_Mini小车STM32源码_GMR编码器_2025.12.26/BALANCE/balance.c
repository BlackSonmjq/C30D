#include "balance.h"

//int Time_count=0; //Time variable //???????

// Robot mode is wrong to detect flag bits
//???????????????????¶À
int robot_mode_check_flag=0; 

short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //????????????     

//========== PWM????????? ==========//
u8 start_check_flag = 0;//????????????PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //????????PWM
u8 clear_done_once = 0; //????????¶À
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

uint32_t LineDiffParam = 50;//??????

static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check=0;
	
	if(*flag==1)
	{
		*flag = 0;
		
		Set_Pwm(0,0,0,0,0); 
		
		check = 1;
		taskENTER_CRITICAL();//????FLash??????Ì‡?????????
		
		int32_t buf[4]={0};
		buf[0] = *((int32_t*)&RC_Velocity);
		buf[1] = *((int32_t*)&Velocity_KP);
		buf[2] = *((int32_t*)&Velocity_KI);
		buf[3] = LineDiffParam;
		check += Write_Flash( (u32*)buf , 4);
		
		taskEXIT_CRITICAL();//??????
		
		//?????ß’????,check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0);//????°¿??0??????
	if( read!=0xffffffff ) RC_Velocity = *((float*)&read);
	
	read = Read_Flash(1);//????°¿??1??????
	if( read!=0xffffffff ) Velocity_KP = *((float*)&read);
	
	read = Read_Flash(2);//????°¿??2??????
	if( read!=0xffffffff ) Velocity_KI = *((float*)&read);
	
	read = Read_Flash(3);
	if( read!=0xffffffff ) LineDiffParam = read;
	
	//?????????,????
	if( RC_Velocity < 0 || RC_Velocity > 10000 )
		RC_Velocity = 500;
	
	//????????,????
	if( LineDiffParam  > 100 )
		LineDiffParam = 50;
	
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
????????????????????????????????????????????
????????X??Y??Z??????????????
????  ?????
**************************************************************************/
//?????????
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //??????,???50~100???1.0~1.2?????????
	{
		if( diffparam>=50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //??????,50~0???1.0~1.2?????????
	{
		if( diffparam<=50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//???????????,?????1.
}


void Drive_Motor(float Vx,float Vy,float Vz)
{
	float amplitude=3.5; //Wheel target speed limit //?????????????

	Vx=target_limit_float(Vx,-amplitude,amplitude);
	Vy=target_limit_float(Vy,-amplitude,amplitude);
	Vz=target_limit_float(Vz,-amplitude,amplitude);
	
	//Speed smoothing is enabled when moving the omnidirectional trolley
	//??????ß≥?????????????????
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
	{
		if(Allow_Recharge==0)
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //???????????????????
		else
			smooth_control.VX=Vx,     
			smooth_control.VY=Vy,
			smooth_control.VZ=Vz;

		//Get the smoothed data 
		//?????????????????			
		Vx=smooth_control.VX;     
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;
	}
		
	//??????????
	float LeftWheelDiff = wheelCoefficient(LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam,0);
	
	//Mecanum wheel car
	//????????ß≥??
	if (Car_Mode==Mec_Car||Car_Mode==Mec_Car_V550) 
	{
		//Inverse kinematics //???????
		MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);

		//Wheel (motor) target speed limit //????(???)?????????
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		
		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=LeftWheelDiff;
		MOTOR_C.Target*=RightWheelDiff;
		MOTOR_D.Target*=RightWheelDiff;
	} 
		
	//Omni car
	//?????ß≥??
	else if (Car_Mode==Omni_Car) 
	{
		//Inverse kinematics //???????
		MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
		MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;

		//Wheel (motor) target speed limit //????(???)?????????
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=0;	//Out of use //???????
		

		MOTOR_B.Target*=LeftWheelDiff;
		MOTOR_C.Target*=RightWheelDiff;
	}
		
	//Ackermann structure car
	//??????ß≥??
	else if (Car_Mode==Akm_Car) 
	{
		//Ackerman car specific related variables //??????ß≥???????????
		float R, Ratio=636.56, AngleR, Angle_Servo;
		
		// For Ackerman small car, Vz represents the front wheel steering Angle
		//?????????ß≥??Vz??????????????
		AngleR=Vz;
		R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
		
		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//???????????(???????????????)????¶À??rad
		AngleR=target_limit_float(AngleR,-0.49f,0.32f);
		
		//Inverse kinematics //???????
		if(AngleR!=0)
		{
			MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
			MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
		}
		else 
		{
			MOTOR_A.Target = Vx;
			MOTOR_B.Target = Vx;
		}
		// The PWM value of the servo controls the steering Angle of the front wheel
		//???PWM??????????????????
		Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
		Servo=SERVO_INIT + (Angle_Servo - 1.572f)*Ratio;

		//Wheel (motor) target speed limit //????(???)?????????
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //???????
		MOTOR_D.Target=0; //Out of use //???????
		Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //???PWM????
		
		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=RightWheelDiff;
	}
		
	//Differential car
	//????ß≥??
	else if (Car_Mode==Diff_Car) 
	{
		//Inverse kinematics //???????
		MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //????????????????
		MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //????????????????

		//Wheel (motor) target speed limit //????(???)?????????
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //???????
		MOTOR_D.Target=0; //Out of use //???????
		
		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=RightWheelDiff;
	}
		
	//FourWheel car
	//??????
	else if(Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550) 
	{	
		//Inverse kinematics //???????
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //????????????????
		MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //????????????????
		MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //????????????????
		MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //????????????????
				
		//Wheel (motor) target speed limit //????(???)?????????
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 

		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=LeftWheelDiff;
		MOTOR_C.Target*=RightWheelDiff;
		MOTOR_D.Target*=RightWheelDiff;
	}

	//Tank Car
	//?????
	else if (Car_Mode==Tank_Car) 
	{
		//Inverse kinematics //???????
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //????????????????
		MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //????????????????

		//Wheel (motor) target speed limit //????(???)?????????
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=0; //Out of use //???????
		MOTOR_D.Target=0; //Out of use //???????
		
		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=RightWheelDiff;
	}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
?????????FreeRTOS??????????????????
??????????
????  ?????
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
		// This task runs at a frequency of 100Hz (10ms control once)
		//????????100Hz????????ßµ?10ms??????¶≤?
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

		//Time count is no longer needed after 30 seconds
		//????????30????????
		if(SysVal.Time_count<3000) SysVal.Time_count++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//???????????????????????????????¶À?????¶À
		Get_Velocity_Form_Encoder();   
		
		//Click the user button to update the gyroscope zero
		//????????????????????????
		Key(); 
			
		if( Allow_Recharge==1 )
			if( Get_Charging_HardWare==0 ) Allow_Recharge=0,Find_Charging_HardWare();
		
//			command_lost_count++;
//			if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0)
//				Move_X=0,Move_Y=0,Move_Z=0;
		if(Allow_Recharge==1)
		{
			if(Get_Charging_HardWare==1)
			{   //??????????????????????????ßﬁ??
				charger_check++;
				if( charger_check>RATE_100_HZ) charger_check=RATE_100_HZ+1,Allow_Recharge=0,RED_STATE=0,Recharge_Red_Move_X = 0,Recharge_Red_Move_Y = 0,Recharge_Red_Move_Z = 0;
			}
			//???????????????????ßﬂ??????????????????????¶À??????????????
			if      (nav_walk==1 && RED_STATE==0) Drive_Motor(Recharge_UP_Move_X,0,Recharge_UP_Move_Z);
			//??????????????????????????????????????
			else if (RED_STATE!=0) nav_walk = 0,Drive_Motor(Recharge_Red_Move_X,0,Recharge_Red_Move_Z);
			//?????ß‹???????ß≥?????
			if (nav_walk==0&&RED_STATE==0) Drive_Motor(0,0,0);
		}
		else
		{			
			if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //????APP???????
			else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //??????????????
			else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //????PS2???????????

			//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
			//without additional processing
			//CAN??????1??????3(ROS)??????5?????????????????????????????
			else                      Drive_Motor(Move_X, Move_Y, Move_Z);
		}

		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
		//and the software failure flag is 0
		//??????????????????????????????ON??¶À???????????????¶À?0
		if(Turn_Off(Voltage)==0||(Allow_Recharge&&EN&&!Flag_Stop)) 
		{ 			
			//Speed closed-loop control to calculate the PWM value of each motor, 
			//PWM represents the actual wheel speed					 
			//????????????????PWM???PWM??????????????
			MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
			MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

			Limit_Pwm(16700);

			//????????????PWM????????????
			auto_pwm_clear();
			
			//Set different PWM control polarity according to different car models
			//??????ß≥????????®∞????PWM???????
			switch(Car_Mode)
			{
				case Mec_Car:case Mec_Car_V550:
					Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //????????ß≥??
				case Omni_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //?????ß≥??
				case Akm_Car:       Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //??????ß≥??
				case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //???????ß≥??
				case FourWheel_Car:case FourWheel_Car_V550:
					Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //?????? 
				case Tank_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //?????
			}
		}
		//If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
		//???Turn_Off(Voltage)??????1????????????ß≥???????????PWM??????0
		else	Set_Pwm(0,0,0,0,0); 
		
		//Flashß’??
		if( 1 == FlashParam_Save(&FlashWriteFlag) )
		{
			Buzzer_count=0;
		}
	}  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
??????????????PWM?????????????????????
????????PWM
????  ?????
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//????????????
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//????????????	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//????????????	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//????????????
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//???????
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
?????????????PWM? 
???????????
????  ?????
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
????????????????
???????????
????  ?????
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
?????????????????????????????????????¶À??
???????????
????  ???????????????1??????????0????
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC2=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
???????????????
????????long int
????  ???unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e??k??-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e??k??-e(k-1)]+Ki*e(k)

??????????????PI??????
???????????????????(??????)????????
????  ??????PWM
????????????PID??? 
pwm+=Kp[e??k??-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)??????????? 
e(k-1)????????¶≈????  ??????? 
pwm???????????
????????????????????ó®????PI????
pwm+=Kp[e??k??-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //???????
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //???????????? 
	
	//???PWM???¶À????¶À?1???????????PWM
	if( start_clear ) 
	{
		//PWM?????????????????ß≥??????????????????????????
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//???????????????¶À??4??????????4??bit???
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //???????
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //???????????? 
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
	}
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //???????
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //???????????? 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<2;
		else clear_state &= ~(1<<2);
	}
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	
	 Bias=Target-Encoder; //Calculate the deviation //???????
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //???????????? 
	
	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car || Car_Mode==Omni_Car ) Pwm = 0;
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//4?????????????????????????
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
???????????APP???????2???????????????ß’???
??????????
????  ?????
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550) //The omnidirectional wheel moving trolley can move laterally //????????ß≥????????ß‹??????
	{
	 switch(Flag_Direction)  //Handle direction control commands //???????????????
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //?????????????????????????
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //?????  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //?????
		 else 		               Move_Z=0;                       //stop           //??
	 }
	}	
	else //Non-omnidirectional moving trolley //????????ß≥??
	{
	 switch(Flag_Direction) //Handle direction control commands //???????????????
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //????? 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //?????	
	}
	
	//Z-axis data conversion //Z?????????
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//????????ß≥??????????????
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //?????????????????????
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //??¶À?????mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//?????????????????????????
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
???????????PS2?????????????ß’???
??????????
????  ?????
**************************************************************************/
#include "xbox360_gamepad.h"
#include "WiredPS2_gamepad.h"
//xbox360?????????????????
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//????start????
	if( keyid == Xbox360KEY_Menu && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	if( gamepad_brand == Xbox360 )
	{
		//????????
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			RC_Velocity -= 50;
		else if( keyid == Xbox360KEY_RB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			RC_Velocity += 50;
		
		if( RC_Velocity < 0 ) RC_Velocity = 0;
	}
	else if(  gamepad_brand == PS2_USB_Wiredless )
	{
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			RC_Velocity += 50;
		else if( keyid == Xbox360_PaddingBit && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK ) )
			RC_Velocity -= 50;
		if( RC_Velocity < 0 ) RC_Velocity = 0;
	}
	
	
	//??????????
	if( keyid == Xbox360KEY_SELECT && event == GamePadKeyEvent_LONGCLICK )
	{
		if( GamePadInterface->Vib_EN )
		{
			GamePadInterface->SetVibration(0,127);
			vTaskDelay(50);
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
		}
		else
		{
			GamePadInterface->Vib_EN = !GamePadInterface->Vib_EN;
			vTaskDelay(50);
			GamePadInterface->SetVibration(0,127);
		}	
	}
}

//????USB??????????
void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//????start????
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	//????????
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity += 50;
	
	if( RC_Velocity < 0 ) RC_Velocity = 0;
}

//????PS2??????????,??USB??
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//????start????
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;
	
	//????????
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity+= 50;
	
	if( RC_Velocity < 0 ) RC_Velocity = 0;
}


//???????Åâ??
static uint8_t map_to_vib(float x) {
    // ???????¶∂???????? [0.2, 1.2] ??
    if (x < 0.1f) return 0;
    if (x > 1.2f) x = 1.2f;

    // ???????
    float result = 255.0f * (x - 0.1f) / 1.1f;

    // ???????????? uint8_t
    return (uint8_t)(result + 0.5f);
}


void PS2_control(void)
{
	float LX=127,LY=127,RX=127;
	float ThrottleTri = 255;
	
	//??????
	LY = GamePadInterface->LY - 127;
	
	//???????
	LX = 127 - GamePadInterface->LX;
	
	//??????
	RX = 127 - GamePadInterface->RX;
	
	//????ß≥???????
	if( fabs(LY)<20 ) LY = 0;
	if( fabs(LX)<20 ) LX = 0;
	if( fabs(RX)<20 ) RX = 0;
	
	//???xbox360??????????????????????????????
	if( gamepad_brand == Xbox360 )
	{
		//??????????,?????????
		if( (int)LY == 0 )
		{
			if( GamePadInterface->LT == 0 && GamePadInterface->RT != 0 )
				ThrottleTri =  GamePadInterface->RT, LY = 127;
			else if( GamePadInterface->LT != 0 && GamePadInterface->RT == 0 )
				ThrottleTri =  -GamePadInterface->LT,LY = 127;
			else
				ThrottleTri = 0;
		}
	}
	
	//???usb???????,????????????????????
	else if( gamepad_brand == PS2_USB_Wired ||  gamepad_brand == PS2_USB_WiredV2 )
	{
		if( fabs(RX)<0.0001f )
		{
			if( GamePadInterface->getKeyState(PS2KEY_4PINK) )
				RX = 127;
			else if( GamePadInterface->getKeyState(PS2KEY_2RED) )
				RX = -127;
		}
	}
	
	  //Handle PS2 controller control commands
	  //??PS2?????????????ß’???
	
	Move_X = (LY/127.0f) * RC_Velocity * (ThrottleTri/255.0f);
	Move_Y = (LX/127.0f) * RC_Velocity;
	Move_Z = (PI/2) * (RX/127.0f) * ( RC_Velocity/500.0f );
	
//		Move_X=LX*RC_Velocity/128; 
//		Move_Y=LY*RC_Velocity/128; 
//		Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z?????????
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //????????ß≥??????????????
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //?????????????????????
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //??¶À?????mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
		//Control target value is obtained and kinematics analysis is performed
	  //?????????????????????????
		Drive_Motor(Move_X,Move_Y,Move_Z);		

	//??????????????????????
	#include "bsp_gamepad.h"
	
	//Z???????ßÿ??????
	float now_z = imu.accel.z/1671.84f;
	static float last_z = 0;
	float strength = fabs(last_z - now_z);
	
	//????????
	if( strength>0.1f && SysVal.Time_count>CONTROL_DELAY)
	{
		if( GamePadInterface->SetVibration!=NULL )
			GamePadInterface->SetVibration(map_to_vib(strength),0);
	}
	last_z = now_z;		
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
?????????????????????????ß’???
??????????
????  ?????
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //??????????????1??????????????
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //????????????ß≥???????

	  //limiter //???
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //?????????????????????
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //????????????????????????????????????????????????????ß≥????????????PWM?????????
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//??????????????/??????
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//????????????????????
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //???????
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//????????????????ß’???
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z?????????
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //????????ß≥??????????????
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //?????????????????????
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //??¶À?????mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //??????????????1??????????????
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //?????????????????????????
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
?????????????????????????????????
??????????
????  ?????
**************************************************************************/
void Key(void)
{	
    u8 tmp;

    //????????????
    tmp=KEY_Scan(RATE_100_HZ,0);
		if(Check==0)
		{
    //???? ?? ???????????????°„??????????????
    if(tmp==single_click )
	{
		Allow_Recharge=!Allow_Recharge;
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}		

    //??? ?? ?????????????????,??????????
    else if(tmp==double_click) 
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //???? ?ß›????
    else if(tmp==long_click )
    {
        oled_refresh_flag=1;
        oled_page++;
        if(oled_page>OLED_MAX_Page-1) oled_page=0;
    }
	
	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
???????????????????????????????????¶Àm/s
??????????
????  ?????
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //?????????????????
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//?????????
	float LeftWheelDiff = wheelCoefficient(LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam,0);
	
	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//??????ß≥?????????????????????
		switch(Car_Mode)
		{
			case Mec_Car:case Mec_Car_V550:
			case FourWheel_Car:case FourWheel_Car_V550:
                Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Akm_Car:case Diff_Car:case Tank_Car:
				Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Omni_Car:    
				Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//??????????????????????????¶Àm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		
		if( Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550 || Car_Mode == FourWheel_Car || Car_Mode == FourWheel_Car_V550)
		{
			MOTOR_A.Encoder /= LeftWheelDiff; MOTOR_B.Encoder /= LeftWheelDiff;
			MOTOR_C.Encoder /= RightWheelDiff; MOTOR_D.Encoder /= RightWheelDiff;
		}
		else if( Car_Mode==Diff_Car || Car_Mode== Tank_Car || Car_Mode == Akm_Car )
		{
			MOTOR_A.Encoder /= LeftWheelDiff; MOTOR_B.Encoder /= RightWheelDiff;
		}
		else if( Car_Mode==Omni_Car )
		{
			MOTOR_B.Encoder /= LeftWheelDiff; MOTOR_C.Encoder /= RightWheelDiff;
		}
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
??????????????????????????????
??????????????????
????  ?????
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;
	
	if(PS2_ON_Flag)
	{
		step=0.05;
	}
	else
	{
		step=0.01;
	}
	
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
??????????????????????????
??????????????
????  ???????????????
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
??????????????¶À????????????????????????????????????????
??????????
????  ?????
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//???????6?¶Õ????????????ßÿ?????????????????	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

//PWM????????
void auto_pwm_clear(void)
{
	//ß≥??????????ßÿ?
	float y_accle = (float)(imu.accel.y/1671.84f);//Y??????????
	float z_accle = (float)(imu.accel.z/1671.84f);//Z??????????
	float diff;
	
	//????Y??Z??????????????????9.8?????ß≥????????
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM???????
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f )
	{
		start_check_flag = 1;//?????????PWM
		wait_clear_times = 0;//??¶À?????
		start_clear = 0;     //??¶À??????
		
		
		//????ß“??????????¶À
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //???????????0??0????????? 2.5 ????ß≥??????ß“??????????pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//ß≥??????????????????pwm?????ß≥????ß“???????????????
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//???????pwm
				else clear_done_once = 1;//ß≥????ß“????????????????
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//????????????????????????pwm??????????????10?????????
	if( clear_done_once )
	{
		//ß≥?????????????????????????????ß≥????ß“??????
		if( diff > 8.8f )
		{
			//????????pwm??¶À???????????
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//???????pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}

