#include "check.h"

//定义自检相关结构体
Check_parameter check_parameter;		
//蜂鸣器计数值
u32 Buzzer_count1 = 0;

void Check_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	check_parameter.Motor_forward = 300;
	check_parameter.Motor_retreat = 500;
	while(1)
	{
		// This task runs at a frequency of 100Hz (10ms control once)
		//此任务以100Hz的频率运行（10ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		//Time count is no longer needed after 30 seconds
		//时间计数，30秒后不再需要
		if(SysVal.Time_count<3000) SysVal.Time_count++;
		
		Buzzer_count1++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//获取编码器数据，即车轮实时速度，并转换位国际单位
		Get_Velocity_Form_Encoder();
		//Click the user button to proceed to the next step
		//Double-click to exit the self-check mode
		//单击用户按键进入下一步，双击退出自检模式
		Check_Key();
		switch(check_parameter.Process)
		{
			case 3:				//电机自检
				if(check_parameter.Motor_forward > 0)				check_parameter.Motor_forward--,Full_rotation = 16799;		//正转
				else if(check_parameter.Motor_retreat > 0)	check_parameter.Motor_retreat--,Full_rotation = -16799;		//反转
				switch(Car_Mode)
				{
					case Mec_Car:case Mec_Car_V550: 
						Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation); break; 									//Mecanum wheel car       //麦克纳姆轮小车
					case Omni_Car:      Set_Pwm(-Full_rotation,  Full_rotation, -Full_rotation, Full_rotation); break; //Omni car                //全向轮小车
					case Akm_Car:       Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation); break; //Ackermann structure car //阿克曼小车
					case Diff_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation); break; //Differential car        //两轮差速小车
					case FourWheel_Car:case FourWheel_Car_V550:
						Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation); break; 									//FourWheel car           //四驱车 
					case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation); break; //Tank Car                //履带车
				} 
				if(!(check_parameter.Motor_forward > 0) && !(check_parameter.Motor_retreat > 0))				Set_Pwm(0,0,0,0);
				break;
			case 4:				//退出电机自检，控制电机停下
				Set_Pwm(0,0,0,0);
				break;
			case 14:				//蜂鸣器自检相关
				if((Buzzer_count1/100)%2)			Buzzer_Set(1);
				else													Buzzer_Set(0);
				break;
			case 15:				//退出蜂鸣器自检时关闭蜂鸣器
				Buzzer_Set(0);
				break;
			case 19:				//串口3自检，检测到标志位则缓存区字符
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

  //传入任务的频率
  tmp=KEY_Scan(RATE_100_HZ,0);
	if(tmp==single_click)				//单击
	{
		check_parameter.Process++;
		// 舵机自检步骤（6～13）已移除，跳过这些步骤以保护步进驱动引脚。
		if(check_parameter.Process==6) check_parameter.Process=14;
		if(check_parameter.Process==21)			//自检结束并退出
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
