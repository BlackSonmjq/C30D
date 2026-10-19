#include "check.h"

//�����Լ���ؽṹ��
Check_parameter check_parameter;		
//����������ֵ
u32 Buzzer_count1 = 0;

void Check_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	check_parameter.Motor_forward = 300;
	check_parameter.Motor_retreat = 500;
	while(1)
	{
		// This task runs at a frequency of 100Hz (10ms control once)
		//��������100Hz��Ƶ�����У�10ms����һ�Σ�
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		//Time count is no longer needed after 30 seconds
		//ʱ�������30�������Ҫ
		if(SysVal.Time_count<3000) SysVal.Time_count++;
		
		Buzzer_count1++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
		Get_Velocity_Form_Encoder();
		//Click the user button to proceed to the next step
		//Double-click to exit the self-check mode
		//�����û�����������һ����˫���˳��Լ�ģʽ
		Check_Key();
		switch(check_parameter.Process)
		{
			case 3:				//����Լ�
				if(check_parameter.Motor_forward > 0)				check_parameter.Motor_forward--,Full_rotation = 16799;		//��ת
				else if(check_parameter.Motor_retreat > 0)	check_parameter.Motor_retreat--,Full_rotation = -16799;		//��ת
				switch(Car_Mode)
				{
					case Mec_Car:case Mec_Car_V550: 
						Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation); break; 									//Mecanum wheel car       //�����ķ��С��
					case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation); break; //Omni car                //ȫ����С��
					case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation); break; //Ackermann structure car //������С��
					case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation); break; //Differential car        //���ֲ���С��
					case FourWheel_Car:case FourWheel_Car_V550:
						Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation); break; 									//FourWheel car           //������
					case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation); break; //Tank Car                //�Ĵ���
				} 
				if(!(check_parameter.Motor_forward > 0) && !(check_parameter.Motor_retreat > 0))				Set_Pwm(0,0,0,0);
				break;
			case 4:				//�˳�����Լ죬���Ƶ��ͣ��
				Set_Pwm(0,0,0,0,0);
				break;
			case 6:				// A4988 pins are reserved; legacy servo self-test is skipped.
				break;
			case 7:
				//�����ת
				if(check_parameter.Servo_Dir[0]==0 && check_parameter.Servo_count[0]<2500)
					check_parameter.Servo_count[0] = check_parameter.Servo_count[0] + 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[0]==0 && check_parameter.Servo_count[0]>=2500)
					check_parameter.Servo_Dir[0] = 1;
				//�����ת
				if(check_parameter.Servo_Dir[0]==1 && check_parameter.Servo_count[0]>500)
					check_parameter.Servo_count[0] = check_parameter.Servo_count[0] - 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[0]==1 && check_parameter.Servo_count[0]<=500)
					check_parameter.Servo_Dir[0] = 2;
				//����ռ�ձȿ��ƶ��ת��
				TIM12->CCR2=check_parameter.Servo_count[0];
				break;
			case 8:
				//�����һ�����û��ת���������һ���������֤�����һ���������λ
				if((check_parameter.Servo_Dir[0]!=2))				
					check_parameter.Servo_count[0]=500,TIM12->CCR2=check_parameter.Servo_count[0];
				//�����ת
				if(check_parameter.Servo_Dir[1]==0 && check_parameter.Servo_count[1]<2500)
					check_parameter.Servo_count[1] = check_parameter.Servo_count[1] + 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[1]==0 && check_parameter.Servo_count[1]>=2500)
					check_parameter.Servo_Dir[1] = 1;
				//�����ת
				if(check_parameter.Servo_Dir[1]==1 && check_parameter.Servo_count[1]>500)
					check_parameter.Servo_count[1] = check_parameter.Servo_count[1] - 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[1]==1 && check_parameter.Servo_count[1]<=500)
					check_parameter.Servo_Dir[1] = 2;
				//����ռ�ձȿ��ƶ��ת��
				TIM12->CCR1=check_parameter.Servo_count[1];
				break;
			case 9:
				//�����һ�����û��ת���������һ���������֤�����һ���������λ
				if((check_parameter.Servo_Dir[1]!=2))				
					check_parameter.Servo_count[1]=500,TIM12->CCR1=check_parameter.Servo_count[1];
				//�����ת
				if(check_parameter.Servo_Dir[2]==0 && check_parameter.Servo_count[2]<2500)
					check_parameter.Servo_count[2] = check_parameter.Servo_count[2] + 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[2]==0 && check_parameter.Servo_count[2]>=2500)
					check_parameter.Servo_Dir[2] = 1;
				//�����ת
				if(check_parameter.Servo_Dir[2]==1 && check_parameter.Servo_count[2]>500)
					check_parameter.Servo_count[2] = check_parameter.Servo_count[2] - 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[2]==1 && check_parameter.Servo_count[2]<=500)
					check_parameter.Servo_Dir[2] = 2;
				//����ռ�ձȿ��ƶ��ת��
				TIM8->CCR4=check_parameter.Servo_count[2];
				break;
			case 10:
				//�����һ�����û��ת���������һ���������֤�����һ���������λ
				if((check_parameter.Servo_Dir[2]!=2))				
					check_parameter.Servo_count[2]=500,TIM8->CCR4=check_parameter.Servo_count[2];
				//�����ת
				if(check_parameter.Servo_Dir[3]==0 && check_parameter.Servo_count[3]<2500)
					check_parameter.Servo_count[3] = check_parameter.Servo_count[3] + 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[3]==0 && check_parameter.Servo_count[3]>=2500)
					check_parameter.Servo_Dir[3] = 1;
				//�����ת
				if(check_parameter.Servo_Dir[3]==1 && check_parameter.Servo_count[3]>500)
					check_parameter.Servo_count[3] = check_parameter.Servo_count[3] - 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[3]==1 && check_parameter.Servo_count[3]<=500)
					check_parameter.Servo_Dir[3] = 2;
				//����ռ�ձȿ��ƶ��ת��
				TIM8->CCR3=check_parameter.Servo_count[3];
				break;
			case 11:
				//�����һ�����û��ת���������һ���������֤�����һ���������λ
				if((check_parameter.Servo_Dir[3]!=2))				
					check_parameter.Servo_count[3]=500,TIM8->CCR3=check_parameter.Servo_count[3];
				//�����ת
				if(check_parameter.Servo_Dir[4]==0 && check_parameter.Servo_count[4]<2500)
					check_parameter.Servo_count[4] = check_parameter.Servo_count[4] + 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[4]==0 && check_parameter.Servo_count[4]>=2500)
					check_parameter.Servo_Dir[4] = 1;
				//�����ת
				if(check_parameter.Servo_Dir[4]==1 && check_parameter.Servo_count[4]>500)
					check_parameter.Servo_count[4] = check_parameter.Servo_count[4] - 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[4]==1 && check_parameter.Servo_count[4]<=500)
					check_parameter.Servo_Dir[4] = 2;
				//����ռ�ձȿ��ƶ��ת��
				TIM8->CCR2=check_parameter.Servo_count[4];
				break;
			case 12:
				//�����һ�����û��ת���������һ���������֤�����һ���������λ
				if((check_parameter.Servo_Dir[4]!=2))				
					check_parameter.Servo_count[4]=500,TIM8->CCR2=check_parameter.Servo_count[4];
				//�����ת
				if(check_parameter.Servo_Dir[5]==0 && check_parameter.Servo_count[5]<2500)
					check_parameter.Servo_count[5] = check_parameter.Servo_count[5] + 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[5]==0 && check_parameter.Servo_count[5]>=2500)
					check_parameter.Servo_Dir[5] = 1;
				//�����ת
				if(check_parameter.Servo_Dir[5]==1 && check_parameter.Servo_count[5]>500)
					check_parameter.Servo_count[5] = check_parameter.Servo_count[5] - 5;
				//��ת�������л���־λ
				if(check_parameter.Servo_Dir[5]==1 && check_parameter.Servo_count[5]<=500)
					check_parameter.Servo_Dir[5] = 2;
				//����ռ�ձȿ��ƶ��ת��
				TIM8->CCR1=check_parameter.Servo_count[5];
				break;
			case 14:				//�������Լ����
				if((Buzzer_count1/100)%2)			Buzzer_Set(1);
				else													Buzzer_Set(0);
				break;
			case 15:				//�˳��������Լ�ʱ�رշ�����
				Buzzer_Set(0);
				break;
			case 19:				//����3�Լ죬��⵽��־λ�򻺴����ַ�
				if(check_parameter.Usart3_check_flag==1)
				{
					USART3_Return();
					check_parameter.Usart3_check_flag = 0;
					check_parameter.Usart3_rec_count = 0;
				}
				break;
			default:
				break;
		}
	}
}

void Check_Key(void)
{
	u8 tmp;

  //���������Ƶ��
  tmp=KEY_Scan(RATE_100_HZ,0);
	if(tmp==single_click)				//����
	{
		check_parameter.Process++;
		// Servo self-test stages 6-13 would reconfigure the A4988 pins.
		if(check_parameter.Process==6) check_parameter.Process=14;
		if(check_parameter.Process==21)			//�Լ�������˳�
		{
			delay_ms(50);
			NVIC_SystemReset();
		}
	}
	else if(tmp==double_click)
	{
		delay_ms(50);
		NVIC_SystemReset();
	}
}


