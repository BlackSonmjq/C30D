#include "control.h"	
#include "filter.h"	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

u8 Flag_Target,Flag_Change;                             //��ر�־λ
u8 temp1;                                               //��ʱ����
float Voltage_Count,Voltage_All; 											  //��ѹ������ر���
float Gyro_K=0.004;     				  											//�����Ǳ���ϵ��
int j;
#define a_PARAMETER          (0.275f)   
#define T 0.320f    //0.145f
#define L 0.315f    //0.17f
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����Y���ٶȺͽǶ�
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vy,float angle)
{
        Target_A   = Vy*(1+T*tan(angle)/2/L);
        Target_B   = Vy*(1-T*tan(angle)/2/L);
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ�����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;                                                      //���LINE5�ϵ��жϱ�־λ  		
		   Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }
		  if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
			{
					if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)  memcpy(rxbuf,Urxbuf,8*sizeof(u8));	//��������˴��ڿ��Ʊ�־λ�����봮�ڿ���ģʽ
					Read_DMP();                                                           //===������̬		
			  	Key();//ɨ�谴���仯	
			    return 0;	                                               
			}                                                                     	 //===10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
			UA_Encoder=Read_Encoder(2);                                          //===��ȡ��������ֵ		
			Encoder_A=UA_Encoder/25;
			Position_A+=Encoder_A;                                                 //===���ֵõ��ٶ�   
			UB_Encoder=-Read_Encoder(3);                                          //===��ȡ��������ֵ		
			Encoder_B=UB_Encoder/25;
			Position_B+=Encoder_B;                                                 //===���ֵõ��ٶ�   
	  	Read_DMP();                                                            	//===������̬	
  		Led_Flash(100);                                                       	 //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬	
			Voltage_All+=Get_battery_volt();                                      	 //��β����ۻ�
			if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	
		  if(PS2_KEY==4)PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;									
		  if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1) CAN_N_Usart_Control();    //�ӵ����ڻ���CANң�ؽ���ָ��֮��ʹ��CAN�ʹ��ڿ�������
			if(RC_Velocity>0&&RC_Velocity<15)  RC_Velocity=15;                   //������������ٷ�������
			if(Turn_Off(Voltage)==0)               //===�����ص�ѹ�������쳣
			 { 			 	
				 if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)      Get_RC(Run_Flag);//===���ں�CAN���ƶ�δʹ�ܣ����������ң��ָ
				 Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===�ٶȱջ����Ƽ�����A����PWM
				 Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===�ٶȱջ����Ƽ�����B����PWM
				 Xianfu_Pwm(6900);                     //===PWM�޷�
				 Set_Pwm(-Motor_A,-Motor_B);     //===��ֵ��PWM�Ĵ���  
			 }
			 else	Set_Pwm(0,0);    //===��ֵ��PWM�Ĵ��� 
	 }
	 return 0;	 
} 


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b)
{
	  
   	if(motor_a<0)			INA2=1,			INA1=0;
			else 	          INA2=0,			INA1=1;
		PWMA=myabs(motor_a);
	
		if(motor_b<0)			INB2=1,			INB1=0;
		else 	            INB2=0,			INB1=1;
		PWMB=myabs(motor_b);
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;
}
/**************************************************************************
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//λ�ÿ���ģʽ�У�A����������ٶ�
		if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //λ�ÿ���ģʽ�У�A����������ٶ�
	  if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//λ�ÿ���ģʽ�У�B����������ٶ�
		if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//λ�ÿ���ģʽ�У�B����������ٶ�
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double(100); 
	if(tmp==2)Flag_Show=!Flag_Show;//˫��������ʾģʽ                  
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<1110)//��ص�ѹ���͹رյ��
			{	                                                
      temp=1;      
      PWMA=0;
      PWMB=0;					
      }
			else
      temp=0;
      return temp;			
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}

/**************************************************************************
�������ܣ�ͨ������ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC(u8 mode)
{
//	  float step=0.3f;  //�����ٶȿ��Ʋ���ֵ��
	  if(mode==0)//�ٶ�
		{	
				 switch(Flag_Direction)   //�������
				 { 
				 case 1:      Move_Y=RC_Velocity;  	 	 Angle=0;        break;
				 case 2:      Move_Y=RC_Velocity;  	 	 Angle=PI/4;   	 break;
				 case 3:      Move_Y=0;      				 	 Angle=0;   	   break;
				 case 4:      Move_Y=-RC_Velocity;  	 Angle=-PI/4;    break;
				 case 5:      Move_Y=-RC_Velocity;  	 Angle=0;        break;
				 case 6:      Move_Y=-RC_Velocity;  	 Angle=PI/4;     break;
				 case 7:      Move_Y=0;     	 			 	 Angle=0;        break;
				 case 8:      Move_Y=+RC_Velocity; 	 	 Angle=-PI/4;    break; 
				 default:     Move_Y=0;                Angle=0;        break;
			 } 
//				if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
//				if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
//				if(Angle<-RC_Velocity) Angle=-RC_Velocity;	
//				if(Angle>RC_Velocity)  Angle=RC_Velocity;	 
	 }
		 Kinematic_Analysis(Move_Y,Angle);//�õ�����Ŀ��ֵ�������˶�ѧ����
}

/**************************************************************************
�������ܣ�����CAN���ߴ��ڿ���ָ����д���
��ڲ�������
����  ֵ����
**************************************************************************/
void CAN_N_Usart_Control(void)
{
//			int flag_1, flag_2;
			int RX,LY;
		  int Yuzhi=20;
			if(CAN_ON_Flag==1||Usart_ON_Flag==1) 
			{
//			 if(rxbuf[1])flag_1=1;  else flag_1=-1;  //�������λ
//			 if(rxbuf[3])flag_2=1;  else flag_2=-1;  //�������λ
//			 Target_A=flag_1*rxbuf[0];
//			 Target_B=flag_2*rxbuf[1];
			 if(rxbuf[1]==0)Move_Y=rxbuf[0]; //ʶ���˶�����
			 else           Move_Y=-rxbuf[0]; //�ٶ�
			 Angle=(rxbuf[2]-90)*PI/180;   //�ǶȻ�ȡ
			}
			else if (PS2_ON_Flag==1)
	    {
	     RX=PS2_RX-128;
			 LY=PS2_LY-128;
			 if(RX>-Yuzhi&&RX<Yuzhi)RX=0; //������������
			 if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		   Angle= RX*PI/4/120;
		   Move_Y=-LY/2.84;	 
		  //if(Move_Y<0)Angle=-Angle;	  
	    }
			Kinematic_Analysis(Move_Y,Angle);//�õ�����Ŀ��ֵ�������˶�ѧ����
}
