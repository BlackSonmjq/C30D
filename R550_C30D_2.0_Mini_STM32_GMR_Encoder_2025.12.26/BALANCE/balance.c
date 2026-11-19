#include "balance.h"
#include "heading_control.h"
#include "wheel_output.h"
#include "stepper_task.h"
#include "route_config.h"
#include "route_io.h"
#include "chassis_odometry.h"
#include "vision_align.h"
#include "vision_strike.h"
#include "servo_task.h"
//int Time_count=0; //Time variable //计时变量

// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0;

short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);

/* 麦轮手动平移：左杆前后、右杆左右；进入手动时锁定当前 IMU 航向。 */
static u8 manual_heading_locked;
static float manual_heading_target;
/* Explicit visual stop: use wheel speed damping without stored drive effort. */
static u8 vision_park_braking;
static VisionAlign_State vision_align;
static uint32_t vision_manual_frames, vision_manual_seen;
static VisionPickPhase vision_pick_phase;
static uint8_t vision_hit_region;
static uint32_t vision_hit_count;
static uint32_t vision_lock_count;
#if VISION_PICK_ENABLE
static VisionStrike_State vision_pick;
static uint8_t vision_pick_id;
static uint8_t vision_pick_attempts;
static uint8_t vision_detection_timed_out;
static uint8_t vision_target_region;
static uint8_t vision_resume_pending;
static uint32_t vision_fruit_seen;
static VisionPickRange_State vision_pick_range;
static uint32_t vision_pick_phase_started;
static uint32_t vision_pick_wait_started;
#endif
static uint8_t vision_align_ready;
static uint16_t vision_align_blocked = 8192U;
uint8_t VisionAlign_GetPhase(void)
{ return vision_align.phase; }
uint8_t VisionAlign_ControlOwned(void)
{
    return vision_align.phase == VISION_ALIGN_ACQUIRE ||
        vision_align.phase == VISION_ALIGN_TRACK || vision_align.phase == VISION_ALIGN_LOST ||
        vision_pick_phase != PICK_IDLE;
}
static void VisionAlign_Drive(float forward, float left);
static uint8_t VisionAlign_ControlAllowed(void)
{
    return VISION_ALIGN_ENABLE && !APP_ON_Flag &&
        !Remote_ON_Flag && !CAN_ON_Flag && !Usart1_ON_Flag && !Usart5_ON_Flag &&
        !Allow_Recharge && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550);
}
static void VisionAlign_ReleaseControl(void)
{
    VisionAlign_Cancel(&vision_align);
    if (vision_pick_phase == PICK_WAIT || vision_pick_phase == PICK_HOLD)
        vision_pick_phase = PICK_IDLE;
#if VISION_PICK_ENABLE
    if (vision_pick_phase != PICK_WORK && vision_pick_phase != PICK_REPORT && vision_pick_phase != PICK_COOLDOWN)
        vision_target_region = 0U;
#endif
    vision_align_ready = 0U;
    vision_align_blocked = 8192U;
}
/* Small live snapshot for VOFA; no formatting or UART wait in control code. */
void VisionAlign_GetDiagnostics(VisionAlign_Diagnostics *data)
{
    uint32_t mask;
    ChassisOdomFrame pose;
    if (!data) return;
    mask = __get_PRIMASK();
    __disable_irq();
    ChassisOdom_Read(&pose);
    data->heading_locked = manual_heading_locked;
    data->imu_fresh = pose.valid &&
        (uint32_t)(xTaskGetTickCount() - pose.tick_ms) <= ROUTE_ODOM_MAX_GAP_MS;
    data->yaw_deg = pose.yaw_rad * 57.2957795f;
    data->yaw_target_deg = (manual_heading_locked ? manual_heading_target : pose.yaw_rad) * 57.2957795f;
    data->yaw_error_deg = data->yaw_target_deg - data->yaw_deg;
    if (data->yaw_error_deg > 180.0f) data->yaw_error_deg -= 360.0f;
    if (data->yaw_error_deg < -180.0f) data->yaw_error_deg += 360.0f;
    data->gyro_z_dps = (float)imu.gyro.z * HEADING_CONTROL_GYRO_FULL_SCALE_DPS / 32768.0f;
    data->heading_i_radps = HeadingControl_GetIntegralOutput();
    data->phase = VisionAlign_GetPhase();
    data->pick_phase = (uint8_t)vision_pick_phase;
    data->hit_region = vision_hit_region;
    data->hit_count = vision_hit_count;
    data->lock_count = vision_lock_count;
#if VISION_PICK_ENABLE
    data->locked_region = vision_target_region;
#else
    data->locked_region = 0U;
#endif
    data->locked = vision_pick_phase == PICK_WAIT || vision_pick_phase == PICK_HOLD || vision_pick_phase == PICK_WORK || vision_pick_phase == PICK_REPORT || vision_align.phase == VISION_ALIGN_ACQUIRE || vision_align.phase == VISION_ALIGN_TRACK ||
        (vision_align.phase == VISION_ALIGN_DONE &&
         (uint32_t)(xTaskGetTickCount() - vision_align.stopped_ms) < VISION_ALIGN_DONE_HOLD_MS);
    data->ly = GamePadInterface ? GamePadInterface->LY : 127U;
    data->rx = GamePadInterface ? GamePadInterface->RX : 127U;
    data->stick_active = fabsf((float)data->ly - 127.0f) >= 20.0f ||
                         fabsf((float)data->rx - 127.0f) >= 20.0f;
    data->ready = vision_align_ready;
    data->blocked = vision_align_blocked;
    data->vx_mps = Move_X;
    data->vy_mps = Move_Y;
    data->wz_radps = Move_Z;
    data->rc_velocity_mmps = RC_Velocity;
    data->wheel_target[0] = MOTOR_A.Target;
    data->wheel_feedback[0] = MOTOR_A.Encoder;
    data->wheel_pwm[0] = EN && !Flag_Stop && Voltage >= 10.0f ? MOTOR_A.Motor_Pwm : 0.0f;
    data->wheel_target[1] = MOTOR_B.Target;
    data->wheel_feedback[1] = MOTOR_B.Encoder;
    data->wheel_pwm[1] = EN && !Flag_Stop && Voltage >= 10.0f ? MOTOR_B.Motor_Pwm : 0.0f;
    data->wheel_target[2] = MOTOR_C.Target;
    data->wheel_feedback[2] = MOTOR_C.Encoder;
    data->wheel_pwm[2] = EN && !Flag_Stop && Voltage >= 10.0f ? MOTOR_C.Motor_Pwm : 0.0f;
    data->wheel_target[3] = MOTOR_D.Target;
    data->wheel_feedback[3] = MOTOR_D.Encoder;
    data->wheel_pwm[3] = EN && !Flag_Stop && Voltage >= 10.0f ? MOTOR_D.Motor_Pwm : 0.0f;
    data->control_flags = (PS2_ON_Flag ? 1U : 0U) | (Remote_ON_Flag ? 2U : 0U) |
        (APP_ON_Flag ? 4U : 0U) | (RouteIO_ControlOwned() ? 8U : 0U) |
        (EN ? 16U : 0U) | (Flag_Stop ? 32U : 0U) |
        (GamePadInterface && GamePadInterface->getKeyState ? 64U : 0U);
    __set_PRIMASK(mask);
}
static void ManualMecanum_Drive(float forward, float left, uint8_t automatic)
{
    ChassisOdomFrame pose;
    float angular = 0.0f;
    RouteLidarFrame lidar;
    float front_limit, rear_limit;
    ChassisOdom_Read(&pose);
    if (!EN || Flag_Stop || Voltage < 10.0f || SysVal.Time_count < CONTROL_DELAY ||
        !pose.valid || (uint32_t)(xTaskGetTickCount() - pose.tick_ms) > ROUTE_ODOM_MAX_GAP_MS) {
        manual_heading_locked = 0U;
        Move_X = Move_Y = Move_Z = 0.0f;
        Drive_Motor(0.0f, 0.0f, 0.0f);
        return;
    }
    /* Navigation uses REBUILD references; manual/visual motion keeps its 45 mm guard. */
    RouteIO_ReadLidar(&lidar);
    if (!RouteIO_ControlOwned() || automatic) {
        front_limit = lidar.fresh && lidar.mm[ROUTE_LIDAR_FRONT] > 45U ?
            (lidar.mm[ROUTE_LIDAR_FRONT] - 45U) * 0.002f : 0.0f;
        rear_limit = lidar.fresh && lidar.mm[ROUTE_LIDAR_BEHIND] > 45U ?
            (lidar.mm[ROUTE_LIDAR_BEHIND] - 45U) * (automatic ? 0.004f : 0.002f) : 0.0f;
        if (forward > front_limit) forward = front_limit;
        if (forward < -rear_limit) forward = -rear_limit;
        if (smooth_control.VX > front_limit) smooth_control.VX = front_limit;
        if (smooth_control.VX < -rear_limit) smooth_control.VX = -rear_limit;
    }
    if (!manual_heading_locked) {
        manual_heading_target = pose.yaw_rad;
        HeadingControl_Reset();
        manual_heading_locked = 1U;
    }
    if (HEADING_CONTROL_ENABLE)
        angular = HeadingControl_HoldYaw(manual_heading_target, pose.yaw_rad, imu.gyro.z);
    /* The IMU controller already bounds yaw effort and damps measured rate. */
    smooth_control.VZ = angular;
    /* On a manual reversal, discard the old direction's ramp. Smooth_control
     * still ramps up the new direction; do not spend cycles driving the old one. */
    if (forward * smooth_control.VX < 0.0f) smooth_control.VX = 0.0f;
    if (left * smooth_control.VY < 0.0f) smooth_control.VY = 0.0f;
    Move_X = forward;
    Move_Y = left;
    Move_Z = angular;
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

volatile u8 clear_state = 0x00;
/*------------------------------------*/

uint32_t LineDiffParam = 50;//纠偏系数

static uint8_t FlashParam_Save(uint8_t *flag)
{
	u8 check=0;

	if(*flag==1)
	{
		*flag = 0;

		Set_Pwm(0,0,0,0);

		check = 1;
		taskENTER_CRITICAL();//操作FLash进入临界，保证数据安全

		int32_t buf[4]={0};
		buf[0] = *((int32_t*)&RC_Velocity);
		buf[1] = *((int32_t*)&Velocity_KP);
		buf[2] = *((int32_t*)&Velocity_KI);
		buf[3] = LineDiffParam;
		check += Write_Flash( (u32*)buf , 4);

		taskEXIT_CRITICAL();//退出临界

		//若全部写入成功,check==1
	}

	return check;
}

void FlashParam_Read(void)
{
	int read;
	read = Read_Flash(0);//读取下标为0的数据
	if( read!=0xffffffff ) RC_Velocity = *((float*)&read);

	read = Read_Flash(1);//读取下标为1的数据
	if( read!=0xffffffff ) Velocity_KP = *((float*)&read);

	read = Read_Flash(2);//读取下标为2的数据
	if( read!=0xffffffff ) Velocity_KI = *((float*)&read);

	read = Read_Flash(3);
	if( read!=0xffffffff ) LineDiffParam = read;

	//异常速度数据,过滤
	if( !(RC_Velocity >= 0) || RC_Velocity > 10000 )
		RC_Velocity = MANUAL_RC_MAX_MMPS;
    /* Raise the previous 200/300 mm/s defaults stored in Flash as well. */
    if (RC_Velocity == 200.0f || RC_Velocity == 300.0f) RC_Velocity = MANUAL_RC_MAX_MMPS;
    RC_Velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);

	//纠偏系数异常,过滤
	if( LineDiffParam  > 100 )
		LineDiffParam = 50;

}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
//计算纠偏系数
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //左轮纠偏,对应50~100对应1.0~1.2倍的纠偏系数
	{
		if( diffparam>=50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //右轮纠偏,50~0对应1.0~1.2倍的纠偏系数
	{
		if( diffparam<=50 )
			return 1.0f + 0.004f*(50-diffparam);
	}

	return 1.0f;//不满足条件时,默认是1.
}



void Drive_Motor(float Vx,float Vy,float Vz)
{
	float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅

	Vx=target_limit_float(Vx,-amplitude,amplitude);
	Vy=target_limit_float(Vy,-amplitude,amplitude);
	Vz=target_limit_float(Vz,-amplitude,amplitude);

	//Speed smoothing is enabled when moving the omnidirectional trolley
	//全向移动小车才开启速度平滑处理
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
	{
		if(Allow_Recharge==0)
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
		else
			smooth_control.VX=Vx,
			smooth_control.VY=Vy,
			smooth_control.VZ=Vz;

		//Get the smoothed data
		//获取平滑处理后的数据
		Vx=smooth_control.VX;
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;
	}

	//纠偏系数计算
	float LeftWheelDiff = wheelCoefficient(LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam,0);

	//Mecanum wheel car
	//麦克纳姆轮小车
	if (Car_Mode==Mec_Car||Car_Mode==Mec_Car_V550)
	{
        /* Discard old translational PI once when handing over to yaw hold. */
        if (Vx == 0.0f && Vy == 0.0f && wheel_output_translation_active)
            wheel_output_reset_pi = 1;
        wheel_output_translation_active = Vx != 0.0f || Vy != 0.0f;
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
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
	//全向轮小车
	else if (Car_Mode==Omni_Car)
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
		MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
		MOTOR_D.Target=0;	//Out of use //没有使用到


		MOTOR_B.Target*=LeftWheelDiff;
		MOTOR_C.Target*=RightWheelDiff;
	}

	//Ackermann structure car
	//阿克曼小车
	else if (Car_Mode==Akm_Car)
	{
		//Ackerman car specific related variables //阿克曼小车专用相关变量
		float R, AngleR;

		// For Ackerman small car, Vz represents the front wheel steering Angle
		//对于阿克曼小车Vz代表右前轮转向角度
		AngleR=Vz;
		R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;

		// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
		//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
		AngleR=target_limit_float(AngleR,-0.49f,0.32f);

		//Inverse kinematics //运动学逆解
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
		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
		MOTOR_C.Target=0; //Out of use //没有使用到
		MOTOR_D.Target=0; //Out of use //没有使用到
		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=RightWheelDiff;
	}

	//Differential car
	//差速小车
	else if (Car_Mode==Diff_Car)
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude);
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude);
		MOTOR_C.Target=0; //Out of use //没有使用到
		MOTOR_D.Target=0; //Out of use //没有使用到

		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=RightWheelDiff;
	}

	//FourWheel car
	//四驱车
	else if(Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
		MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
		MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
		MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
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
	//履带车
	else if (Car_Mode==Tank_Car)
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude);
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude);
		MOTOR_C.Target=0; //Out of use //没有使用到
		MOTOR_D.Target=0; //Out of use //没有使用到

		MOTOR_A.Target*=LeftWheelDiff;
		MOTOR_B.Target*=RightWheelDiff;
	}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	float chassis_vz;
	u8 heading_control_active;
	u8 chassis_allowed, actuator_parked, wheel_output_active;
    IMU_HeadingFrame heading;
	static u8 imu_ready_beeped = 0;

    while(1)
    {
		// This task runs at a frequency of 100Hz (10ms control once)
		//此任务以100Hz的频率运行（10ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

		//Time count is no longer needed after 30 seconds
		//时间计数，30秒后不再需要
		if(SysVal.Time_count<3000) SysVal.Time_count++;
		//Get the encoder data, that is, the real time wheel speed,
		//and convert to transposition international units
		//获取编码器数据，即车轮实时速度，并转换位国际单位
        Get_Velocity_Form_Encoder();
        Imu_ReadHeading(&heading);
        if (ROUTE_DEMO_ENABLE || VISION_ALIGN_ENABLE) {
            /* Encoder 按固定 CONTROL_FREQUENCY 换算过速度；除以同一频率恢复
             * 本次计数对应的位移，平移里程不再重复乘实际采样时间。单位 mm。 */
            ChassisOdom_UpdateDelta((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) *
                                   (250.0f / CONTROL_FREQUENCY),
                                   (MOTOR_A.Encoder - MOTOR_B.Encoder + MOTOR_C.Encoder - MOTOR_D.Encoder) *
                                   (250.0f / CONTROL_FREQUENCY),
                              imu.gyro.z, (uint32_t)xTaskGetTickCount(),
                              SysVal.Time_count >= CONTROL_DELAY &&
                              (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550));
        }

		//Click the user button to update the gyroscope zero
		//单击用户按键更新陀螺仪零点
		Key();

		/* One short beep when startup IMU calibration has completed. */
		if((SysVal.Time_count >= CONTROL_DELAY) && heading.ready && (imu_ready_beeped == 0))
		{
			Buzzer_count = 0;
			imu_ready_beeped = 1;
		}

        vision_park_braking = 0U;
        if (RouteIO_ControlOwned() && PS2_ON_Flag && GamePadInterface &&
            (fabsf((float)GamePadInterface->LY - 127.0f) >= 20.0f ||
             fabsf((float)GamePadInterface->RX - 127.0f) >= 20.0f))
            RouteIO_RequestManual();
        if (APP_ON_Flag ||
            (!Remote_ON_Flag && !PS2_ON_Flag &&
             !VisionAlign_ControlAllowed()) ||
            !EN || Flag_Stop)
            manual_heading_locked = 0U;
        if (!VisionAlign_ControlAllowed() || !EN || Flag_Stop)
            VisionAlign_ReleaseControl();

        if (vision_pick_phase == PICK_WORK || vision_pick_phase == PICK_REPORT)
        {
            VisionAlign_Drive(0.0f, 0.0f);
        }
        else if (RouteIO_ControlOwned())
        {
            Allow_Recharge = 0;
            RouteIO_ApplyDrive();
        }
        else
        {
            if (Allow_Recharge == 1 && Get_Charging_HardWare == 0)
            {
                Allow_Recharge = 0;
                Find_Charging_HardWare();
            }


        /* Hold heading normally, including idle after startup calibration.
         * Ignore parked upper-actuator vibration; never block chassis commands.
         * Explicit turns establish a new heading reference afterward. */
        chassis_allowed = (Voltage >= 10) && EN && !Flag_Stop;
        actuator_parked = StepperTask_IsBusy() &&
            (Move_X == 0.0f) && (Move_Y == 0.0f) && (Move_Z == 0.0f);

		heading_control_active =
			(HEADING_CONTROL_ENABLE != 0) &&
            chassis_allowed &&
            !actuator_parked &&
            (Move_Z == 0.0f) &&
			(SysVal.Time_count >= CONTROL_DELAY) &&
			(Allow_Recharge == 0) &&
			(Car_Mode != Akm_Car) &&
			(APP_ON_Flag == 0) &&
			(Remote_ON_Flag == 0) &&
			(PS2_ON_Flag == 0) &&
			(CAN_ON_Flag == 0) &&
			(Usart1_ON_Flag == 0) &&
			(Usart5_ON_Flag == 0);

		if(heading_control_active == 0 && manual_heading_locked == 0U)
			HeadingControl_Reset();

//			command_lost_count++;
//			if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0)
//				Move_X=0,Move_Y=0,Move_Z=0;
		if(Allow_Recharge==1)
		{
			if(Get_Charging_HardWare==1)
			{   //存在回充装备时，对回充装备的状态进行检测
				charger_check++;
				if( charger_check>RATE_100_HZ) charger_check=RATE_100_HZ+1,Allow_Recharge=0,RED_STATE=0,Recharge_Red_Move_X = 0,Recharge_Red_Move_Y = 0,Recharge_Red_Move_Z = 0;
			}
			//如果开启了导航回充，同时没有接收到红外信号，接收来自上位机的的回充控制命令
			if      (nav_walk==1 && RED_STATE==0) Drive_Motor(Recharge_UP_Move_X,0,Recharge_UP_Move_Z);
			//接收到了红外信号，接收来自回充装备的回充控制命令
			else if (RED_STATE!=0) nav_walk = 0,Drive_Motor(Recharge_Red_Move_X,0,Recharge_Red_Move_Z);
			//防止没有红外信号时小车运动
			if (nav_walk==0&&RED_STATE==0) Drive_Motor(0,0,0);
		}
		else
		{
			if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
			else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令
            else if (VisionAlign_ControlAllowed()) VisionAlign_Drive(0.0f, 0.0f);

			//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed,
			//without additional processing
			//CAN、串口1、串口3(ROS)、串口5控制直接得到三轴目标速度，无须额外处理
			else
			{
				chassis_vz = Move_Z;
				if(heading_control_active)
					chassis_vz = HeadingControl_Update(imu.gyro.z);

				Drive_Motor(Move_X, Move_Y, chassis_vz);
			}
		}

        }

		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
		//and the software failure flag is 0
		//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
		if((Turn_Off(Voltage)==0||(!RouteIO_ControlOwned()&&Allow_Recharge&&EN&&!Flag_Stop)) &&
           heading.ready && (uint32_t)(xTaskGetTickCount()-heading.tick_ms)<100U)
		{
			//Speed closed-loop control to calculate the PWM value of each motor,
			//PWM represents the actual wheel speed
			//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
            wheel_output_active = !start_clear && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550);
            WheelOutput_UpdateStartup(0U, MOTOR_A.Target, MOTOR_A.Encoder, wheel_output_active);
            WheelOutput_UpdateStartup(1U, MOTOR_B.Target, MOTOR_B.Encoder, wheel_output_active);
            WheelOutput_UpdateStartup(2U, MOTOR_C.Target, MOTOR_C.Encoder, wheel_output_active);
            WheelOutput_UpdateStartup(3U, MOTOR_D.Target, MOTOR_D.Encoder, wheel_output_active);
            wheel_output_stopping = fabsf(MOTOR_A.Target) <= WHEEL_FEEDFORWARD_MIN_MPS &&
                                    fabsf(MOTOR_B.Target) <= WHEEL_FEEDFORWARD_MIN_MPS &&
                                    fabsf(MOTOR_C.Target) <= WHEEL_FEEDFORWARD_MIN_MPS &&
                                    fabsf(MOTOR_D.Target) <= WHEEL_FEEDFORWARD_MIN_MPS;
			MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
			MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

            MOTOR_A.Motor_Pwm = WheelOutput_Apply(0U, MOTOR_A.Motor_Pwm, MOTOR_A.Target, MOTOR_A.Encoder, wheel_output_active);
            MOTOR_B.Motor_Pwm = WheelOutput_Apply(1U, MOTOR_B.Motor_Pwm, MOTOR_B.Target, MOTOR_B.Encoder, wheel_output_active);
            MOTOR_C.Motor_Pwm = WheelOutput_Apply(2U, MOTOR_C.Motor_Pwm, MOTOR_C.Target, MOTOR_C.Encoder, wheel_output_active);
            MOTOR_D.Motor_Pwm = WheelOutput_Apply(3U, MOTOR_D.Motor_Pwm, MOTOR_D.Target, MOTOR_D.Encoder, wheel_output_active);
            wheel_output_reset_pi = 0;


			Limit_Pwm(16700);

			//检测是否需要清除PWM并自动执行清理
			auto_pwm_clear();

			//Set different PWM control polarity according to different car models
			//根据不同小车型号设置不同的PWM控制极性
			switch(Car_Mode)
			{
				case Mec_Car:case Mec_Car_V550:
					Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); break; //Mecanum wheel car       //麦克纳姆轮小车
				case Omni_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); break; //Omni car                //全向轮小车
				case Akm_Car:       Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); break; //Ackermann structure car //阿克曼小车
				case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); break; //Differential car        //两轮差速小车
				case FourWheel_Car:case FourWheel_Car_V550:
					Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); break; //FourWheel car           //四驱车
				case Tank_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); break; //Tank Car                //履带车
			}
		}
		//If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
		//如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
		else {
            WheelOutput_ResetStartup();
            Set_Pwm(0,0,0,0);
        }

		//Flash写入
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
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;

	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;

	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;

}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值
入口参数：幅值
返回  值：无
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
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
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
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
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
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 if (vision_park_braking || wheel_output_stopping || wheel_output_reset_pi) Pwm = Velocity_KP * Bias;
	 else Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
     Pwm = WheelOutput_ClampPi(0U, Pwm, Target, Encoder, !start_clear && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550));
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差

	//清除PWM标志位，该位为1时代表需要清除PWM
	if( start_clear )
	{
		//PWM逐渐递减的方式清除，减缓小车由于电机释放而造成轻微移动的影响
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;

		//若清除完毕，则标记标志位，4个电机分别用4个bit表示
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}

	 return Pwm;
}
int Incremental_PI_B (float Encoder,float Target)
{
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 if (vision_park_braking || wheel_output_stopping || wheel_output_reset_pi) Pwm = Velocity_KP * Bias;
	 else Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
     Pwm = WheelOutput_ClampPi(1U, Pwm, Target, Encoder, !start_clear && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550));
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差
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
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 if (vision_park_braking || wheel_output_stopping || wheel_output_reset_pi) Pwm = Velocity_KP * Bias;
	 else Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
     Pwm = WheelOutput_ClampPi(2U, Pwm, Target, Encoder, !start_clear && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550));
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差

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

	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 if (vision_park_braking || wheel_output_stopping || wheel_output_reset_pi) Pwm = Velocity_KP * Bias;
	 else Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
     Pwm = WheelOutput_ClampPi(3U, Pwm, Target, Encoder, !start_clear && (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550));
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差

	if(Car_Mode==Diff_Car || Car_Mode==Akm_Car || Car_Mode==Tank_Car || Car_Mode==Omni_Car ) Pwm = 0;
	if( start_clear )
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;

		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);

		//4个电机均清除完毕，则关闭清除任务
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	 return Pwm;
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
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
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
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
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转
	}

	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9;
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}

	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;

	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
#include "xbox360_gamepad.h"
#include "WiredPS2_gamepad.h"
//xbox360游戏手柄按键回调函数
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
    if (keyid == Xbox360KEY_Y && event == GamePadKeyEvent_SINGLECLICK)
        RouteIO_RequestStart();
	//按下start按键
	if( keyid == Xbox360KEY_Menu && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;

	if( gamepad_brand == Xbox360 )
	{
		//手柄加减速
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			RC_Velocity -= 50;
		else if( keyid == Xbox360KEY_RB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			RC_Velocity += 50;

		RC_Velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
	}
	else if(  gamepad_brand == PS2_USB_Wiredless )
	{
		if( keyid == Xbox360KEY_LB && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
			RC_Velocity += 50;
		else if( keyid == Xbox360_PaddingBit && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK ) )
			RC_Velocity -= 50;
		RC_Velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
	}


	//震动激活与取消
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

//有线USB手柄回调函数
void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
    if (keyid == PS2KEY_1GREEN && event == GamePadKeyEvent_SINGLECLICK)
        RouteIO_RequestStart();
	//按下start按键
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;

	//手柄加减速
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity += 50;

	RC_Velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
}

//经典PS2手柄回调函数,非USB款
void Classic_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	//按下start按键
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_SINGLECLICK )
		GamePadInterface->StartFlag = 1;

	//手柄加减速
	else if( keyid == PS2KEY_L2 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity -= 50;
	else if( keyid == PS2KEY_L1 && (event == GamePadKeyEvent_DOUBLECLICK || event == GamePadKeyEvent_SINGLECLICK )  )
		RC_Velocity+= 50;

	RC_Velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
}


//手柄震动映射函数
static uint8_t map_to_vib(float x) {
    // 检查输入范围，限制在 [0.2, 1.2] 内
    if (x < 0.1f) return 0;
    if (x > 1.2f) x = 1.2f;

    // 线性映射
    float result = 255.0f * (x - 0.1f) / 1.1f;

    // 四舍五入并转换为 uint8_t
    return (uint8_t)(result + 0.5f);
}


/* Automatic DX alignment is available without a controller.
 * Vision owns ACQUIRE/TRACK; manual input is only used outside an attempt. */
static void VisionAlign_HardStop(void)
{
    vision_park_braking = 1U;
    smooth_control.VX = smooth_control.VY = smooth_control.VZ = 0.0f;
    Move_X = Move_Y = Move_Z = 0.0f;
    /* Own tool motion inhibits chassis output, but does not invalidate IMU yaw. */
    if (!vision_align_ready && !(vision_pick_phase == PICK_WORK &&
        (vision_align_blocked & ~(512U | 1024U)) == 0U)) {
        manual_heading_locked = 0U;
        HeadingControl_Reset();
    }
    Drive_Motor(0.0f, 0.0f, 0.0f);
}
static void VisionAlign_HoldYaw(void)
{
    if (!vision_align_ready || vision_pick_phase == PICK_WORK) {
        VisionAlign_HardStop();
        return;
    }
    smooth_control.VX = smooth_control.VY = 0.0f;
    ManualMecanum_Drive(0.0f, 0.0f, 1U);
    if (Move_Z == 0.0f) vision_park_braking = 1U;
}
static void VisionAlign_Drive(float forward, float left)
{
#if VISION_ALIGN_ENABLE
    Camera_Data camera;
    RouteLidarFrame align_lidar;
    uint16_t rear_filtered;
    ChassisOdomFrame pose;
    VisionAlign_Output command;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint8_t ready;
#if VISION_PICK_ENABLE
    uint8_t previous_phase = vision_align.phase;
    uint8_t parked = vision_pick_phase == PICK_WAIT || vision_pick_phase == PICK_HOLD;
    uint8_t fruit_expired;
#endif
    uint8_t was_automatic = vision_align.phase == VISION_ALIGN_ACQUIRE || vision_align.phase == VISION_ALIGN_TRACK;
    uint8_t manual_active = PS2_ON_Flag && GamePadInterface && GamePadInterface->getKeyState &&
        (fabsf((float)GamePadInterface->LY - 127.0f) >= 20.0f ||
         fabsf((float)GamePadInterface->RX - 127.0f) >= 20.0f);
    Camera_GetData(&camera);
    ChassisOdom_Read(&pose);
    vision_align_blocked = (!VisionAlign_ControlAllowed() ? 8192U : 0U) | (Check != 0 ? 2U : 0U) |
        (!EN ? 4U : 0U) | (Flag_Stop ? 8U : 0U) | (!(Voltage >= 10.0f) ? 16U : 0U) |
        (SysVal.Time_count < CONTROL_DELAY ? 32U : 0U) | (!pose.valid ? 64U : 0U) |
        ((uint32_t)(now - pose.tick_ms) > ROUTE_ODOM_MAX_GAP_MS ? 128U : 0U) |
        (StepperTask_IsBusy() ? 512U : 0U) |
        (ServoTask_IsBusy() ? 1024U : 0U) | (USART_ServoControlBusy() ? 2048U : 0U) |
        (RouteIO_WorkReserved() ? 4096U : 0U);
    ready = vision_align_ready = vision_align_blocked == 0U;
#if VISION_PICK_ENABLE
    if (vision_pick_phase == PICK_WORK) {
        /* Ignore all vision decisions until safe return; consume frames without acting. */
        vision_align.last_frames = vision_manual_frames = camera.frames;
        vision_manual_seen = now;
        if (!StepperTask_IsBusy()) {
            StepperTask_Data tool;
            StepperTask_GetData(&tool);
            if (tool.state == STEPPER_RETURNED && tool.mango_id == vision_pick_id) {
                vision_resume_pending = 1U;
                vision_pick_phase_started = now;
                if (tool.result == MANGO_OK) {
                    vision_hit_region = vision_pick_id;
                    ++vision_hit_count;
                    vision_hit_report = vision_pick_id;
                    vision_target_region = 0U;
                    vision_pick_phase = PICK_REPORT;
                } else {
                    vision_pick_phase = PICK_COOLDOWN;
                    vision_pick_phase_started = now;
                }
            }
        }
        VisionAlign_HardStop();
        return;
    }
    if (vision_resume_pending) {
        if (camera.frames == vision_align.last_frames) {
            if ((uint32_t)(now - vision_pick_phase_started) >= VISION_ALIGN_FRESH_MS) {
                vision_resume_pending = 0U;
                goto detection_timeout;
            }
            VisionAlign_HoldYaw();
            return;
        }
        vision_resume_pending = 0U;
    }
    /* A fresh missing green fruit ends automatic ownership before the tool starts. */
    if (ready && camera.frames != vision_manual_frames && camera.green == 0U &&
        (parked || was_automatic || previous_phase == VISION_ALIGN_LOST)) {
        vision_target_region = 0U;
        vision_pick_attempts = 0U;
        vision_pick_phase = PICK_IDLE;
        vision_manual_frames = vision_align.last_frames = camera.frames;
        vision_manual_seen = now;
        VisionAlign_Cancel(&vision_align);
        vision_align.phase = VISION_ALIGN_WAIT;
        VisionAlign_HardStop();
        return;
    }
    /* Evaluate the raw locked number before substituting it into the local snapshot.
     * Cached frames, another fruit and simulated DX cannot extend this deadline. */
    fruit_expired = ready && vision_target_region && vision_pick_phase <= PICK_HOLD &&
        (parked || was_automatic || previous_phase == VISION_ALIGN_LOST) &&
        ((uint32_t)(now - vision_fruit_seen) >= VISION_PICK_FRUIT_GAP_MS ||
         (camera.frames == vision_manual_frames &&
          (uint32_t)(now - vision_manual_seen) >= VISION_ALIGN_FRESH_MS));
    /* First new numbered green observation owns this target until release/return. */
    if (vision_pick_phase != PICK_WORK &&
        ((camera.green == 0U && !parked) || (camera.frames == vision_manual_frames &&
         (uint32_t)(now - vision_manual_seen) >= VISION_ALIGN_FRESH_MS)))
        vision_target_region = 0U;
    if (ready && !vision_detection_timed_out && vision_pick_phase != PICK_WORK &&
        !vision_target_region && camera.frames && camera.frames != vision_manual_frames &&
        camera.green == 1U && camera.region >= 1U && camera.region <= 4U)
    {
        vision_target_region = camera.region;
        ++vision_lock_count;
    }
    if (vision_target_region && camera.green == 1U) {
        if (camera.frames != vision_manual_frames && camera.region == vision_target_region)
            vision_fruit_seen = now;
        camera.region = vision_target_region;
    }
#endif
    if (camera.frames != vision_manual_frames) {
        vision_manual_frames = camera.frames;
        vision_manual_seen = now;
    }
    /* Scale manual stick speed only; automatic alignment keeps its own tuning. */
    if (camera.frames && (camera.pole == 1U || camera.green == 1U) &&
        (uint32_t)(now - vision_manual_seen) < VISION_ALIGN_FRESH_MS) {
        float rc_speed = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
        float limit = MANUAL_RC_TARGET_MMPS * 0.001f;
        if (rc_speed > MANUAL_RC_TARGET_MMPS) {
            forward *= MANUAL_RC_TARGET_MMPS / rc_speed;
            left *= MANUAL_RC_TARGET_MMPS / rc_speed;
        }
        smooth_control.VX = target_limit_float(smooth_control.VX, -limit, limit);
        smooth_control.VY = target_limit_float(smooth_control.VY, -limit, limit);
    }
#if VISION_PICK_ENABLE
    /* A timed-out green episode cannot immediately steal manual control back. */
    if (vision_detection_timed_out) {
        vision_target_region = 0U;
        if (camera.frames != vision_align.last_frames && camera.green == 0U) {
            vision_detection_timed_out = 0U;
            vision_pick_attempts = 0U;
        }
        vision_align.last_frames = camera.frames;
        ManualMecanum_Drive(forward, left, 0U);
        return;
    }
    if (fruit_expired &&
        ((uint32_t)(now - vision_fruit_seen) >= VISION_PICK_FRUIT_GAP_MS ||
         (uint32_t)(now - vision_manual_seen) >= VISION_ALIGN_FRESH_MS))
        goto detection_timeout;
    /* A transient absence during the parked decision belongs to the same episode. */
    if (!parked && vision_pick_phase != PICK_WORK && camera.frames &&
        camera.frames != vision_align.last_frames && camera.green == 0U)
        vision_pick_attempts = 0U;
    if (vision_pick_phase == PICK_REPORT) {
        /* A fresh G=0 exits immediately. */
        if (camera.frames != vision_align.last_frames) {
            vision_align.last_frames = camera.frames;
            if (camera.green == 0U) {
                (void)VisionStrike_Update(&vision_pick, &camera, now, 0U);
                VisionAlign_Cancel(&vision_align);
                vision_align.phase = VISION_ALIGN_WAIT;
                vision_pick_phase = PICK_IDLE;
                ManualMecanum_Drive(forward, left, 0U);
                return;
            }
            if (camera.green == 1U && vision_pick_attempts > VISION_PICK_MAX_REPEATS)
                goto detection_timeout;
            if (camera.green == 1U && camera.region == 0U) {
                vision_detection_timed_out = 1U;
                vision_pick_phase = PICK_IDLE;
                VisionAlign_Cancel(&vision_align);
                vision_align.phase = VISION_ALIGN_WAIT;
            } else if (camera.pole == 1U && camera.green == 1U &&
                camera.region >= 1U && camera.region <= 4U &&
                vision_pick_attempts <= VISION_PICK_MAX_REPEATS) {
                /* More fruit: retain automatic ownership, but discard old
                 * confirmations before another alignment and stationary hold. */
                vision_pick.latched = 0U;
                VisionStrike_ResetEvidence(&vision_pick);
                vision_pick.last_frames = camera.frames;
                vision_pick.last_seen_ms = now;
                VisionAlign_Begin(&vision_align, now);
                vision_align.last_seen_ms = now;
                vision_pick_phase = PICK_IDLE;
            }
        }
        VisionAlign_HoldYaw();
        return;
    }
    if (vision_pick_phase == PICK_COOLDOWN) {
        /* Discard even the snapshot present at expiry; resume on a later frame. */
        vision_align.last_frames = vision_pick.last_frames = camera.frames;
        if ((uint32_t)(now - vision_pick_phase_started) >= VISION_PICK_COOLDOWN_MS) {
            /* Rearm only a safely returned failure, within the existing quota. */
            vision_pick_phase = PICK_IDLE;
            if (vision_pick_attempts <= VISION_PICK_MAX_REPEATS && camera.pole == 1U &&
                camera.green == 1U && camera.region >= 1U && camera.region <= 4U) {
                vision_pick.latched = 0U;
                VisionStrike_ResetEvidence(&vision_pick);
                vision_pick.last_seen_ms = now;
                VisionAlign_Begin(&vision_align, now);
                vision_align.last_seen_ms = now;
                VisionAlign_HardStop();
                return;
            }
            vision_detection_timed_out = 1U;
            VisionAlign_Cancel(&vision_align);
            vision_align.phase = VISION_ALIGN_WAIT;
            VisionAlign_HardStop();
            return;
        }
        ManualMecanum_Drive(forward, left, 0U);
        return;
    }
#endif
#if VISION_PICK_ENABLE
    if (!ready) vision_target_region = 0U;
    if (ready && !vision_target_region && vision_pick_phase == PICK_IDLE &&
        (vision_align.phase == VISION_ALIGN_IDLE || vision_align.phase == VISION_ALIGN_WAIT)) {
        (void)VisionStrike_Update(&vision_pick, &camera, now, 0U);
        ManualMecanum_Drive(forward, left, 0U);
        return;
    }
#endif
    RouteIO_ReadLidar(&align_lidar);
#if VISION_PICK_ENABLE
    rear_filtered = VisionPickRange_Update(&vision_pick_range,
        align_lidar.mm[ROUTE_LIDAR_BEHIND], align_lidar.sequence, align_lidar.fresh);
    /* Raw near-obstacle data always wins over the median. */
    if (align_lidar.mm[ROUTE_LIDAR_BEHIND] <= 45U)
        rear_filtered = align_lidar.mm[ROUTE_LIDAR_BEHIND];
    /* Resolve the old DX3/4 dead band with the original tracking effort.
     * Only new, same-side observations qualify; never turn B=0 into DX=0. */
    if (ready && vision_target_region &&
        (vision_align.phase == VISION_ALIGN_DONE ||
         (vision_align.phase == VISION_ALIGN_TRACK && vision_align.approaching))) {
        if (camera.frames != vision_align.last_frames) {
            int8_t sign = camera.pole == 1U && camera.green == 1U &&
                !VisionAlign_Centered(camera.offset) ? (camera.offset > 0 ? 1 : -1) : 0;
            /* +/-2 identifies a persistent rear error separately from lateral +/-1. */
            if (!sign && parked && camera.pole == 1U && camera.green == 1U &&
                align_lidar.fresh && rear_filtered && VisionAlign_Centered(camera.offset)) {
                if (rear_filtered > VISION_ALIGN_REAR_MM + VISION_ALIGN_REAR_TOLERANCE_MM) sign = 2;
                if (rear_filtered < 45U ||
                    rear_filtered < VISION_ALIGN_REAR_MM - VISION_ALIGN_REAR_TOLERANCE_MM) sign = -2;
            }
            if (!sign || sign != vision_align.restart_sign) vision_align.restart_frames = 0U;
            vision_align.restart_sign = sign;
            if (sign && ++vision_align.restart_frames >= VISION_ALIGN_RESTART_FRAMES) {
                vision_align.phase = VISION_ALIGN_TRACK;
                vision_align.approaching = sign == 2 || sign == -2;
                vision_align.zero_frames = vision_align.restart_frames = 0U;
                vision_pick_phase = PICK_IDLE;
            }
        }
    }
    /* Park through short visual gaps. Only real pole/green data can qualify the hold. */
    if ((vision_pick_phase == PICK_WAIT || vision_pick_phase == PICK_HOLD) && ready) {
        if (camera.frames != vision_align.last_frames) {
            vision_align.last_frames = camera.frames;
            vision_align.last_seen_ms = now;
        }
        command.owned = 1U;
        command.x_mps = command.y_mps = 0.0f;
    } else
#else
    rear_filtered = align_lidar.mm[ROUTE_LIDAR_BEHIND];
#endif
    command = VisionAlign_Update(&vision_align, &camera, now, manual_active, ready,
        rear_filtered, align_lidar.fresh && rear_filtered != 0U);
#if VISION_PICK_ENABLE
    /* An exhausted attempt quota cannot enter an unfinishable stationary hold. */
    if (vision_align.phase == VISION_ALIGN_DONE && vision_pick_attempts > VISION_PICK_MAX_REPEATS)
        vision_detection_timed_out = 1U;
    /* Alignment completion requires a locked target, not a new REGION reading. */
    if (ready && !parked && vision_align.phase == VISION_ALIGN_DONE &&
        (camera.pole != 1U || camera.green != 1U || !vision_target_region ||
         (uint32_t)(now - vision_align.last_seen_ms) >= VISION_ALIGN_FRESH_MS))
        vision_detection_timed_out = 1U;
    if (vision_align.phase == VISION_ALIGN_TIMEOUT ||
        (vision_align.phase == VISION_ALIGN_LOST &&
         (uint32_t)(now - vision_align.started_ms) >= VISION_ALIGN_TIMEOUT_MS))
        vision_detection_timed_out = 1U;
    if (vision_align.phase == VISION_ALIGN_DONE && previous_phase != VISION_ALIGN_DONE && !vision_pick.latched) {
        vision_pick_phase = PICK_WAIT;
        vision_pick_wait_started = now;
    }
    {
        uint8_t blocked = !ready || (!vision_pick.latched && (vision_align.phase != VISION_ALIGN_DONE ||
            !VisionAlign_Centered(camera.offset)));
        /* DX/range still gate motion and the hold; they do not erase number evidence. */
        (void)VisionStrike_Update(&vision_pick, &camera, now,
            !ready || (!vision_pick.latched && vision_align.phase != VISION_ALIGN_DONE));
        if (!ready || vision_align.phase != VISION_ALIGN_DONE)
            vision_pick_phase = PICK_IDLE;
        if (vision_pick_phase == PICK_WAIT || vision_pick_phase == PICK_HOLD) {
            uint8_t qualified;
            qualified = !blocked && camera.pole == 1U && camera.green == 1U && vision_target_region &&
                (uint32_t)(now - vision_align.last_seen_ms) < VISION_ALIGN_FRESH_MS &&
                align_lidar.fresh && align_lidar.mm[ROUTE_LIDAR_BEHIND] >= 45U &&
                align_lidar.mm[ROUTE_LIDAR_BEHIND] < VISION_PICK_REAR_MAX_MM &&
                rear_filtered >= VISION_ALIGN_REAR_MM - VISION_ALIGN_REAR_TOLERANCE_MM &&
                rear_filtered <= VISION_ALIGN_REAR_MM + VISION_ALIGN_REAR_TOLERANCE_MM &&
                !vision_hit_report &&
                fabsf(MOTOR_A.Encoder) <= VISION_PICK_STOP_MPS &&
                fabsf(MOTOR_B.Encoder) <= VISION_PICK_STOP_MPS &&
                fabsf(MOTOR_C.Encoder) <= VISION_PICK_STOP_MPS &&
                fabsf(MOTOR_D.Encoder) <= VISION_PICK_STOP_MPS;
            if (!qualified) vision_pick_phase = PICK_WAIT;
            switch (vision_pick_phase) {
            case PICK_WAIT:
                if ((uint32_t)(now - vision_pick_wait_started) >= VISION_PICK_WAIT_MS) {
                    vision_detection_timed_out = 1U;
                } else if (qualified) {
                    vision_pick_phase = PICK_HOLD;
                    vision_pick_phase_started = now;
                }
                break;
            case PICK_HOLD:
                if ((uint32_t)(now - vision_pick_wait_started) >= VISION_PICK_WAIT_MS) {
                    vision_detection_timed_out = 1U;
                } else if ((uint32_t)(now - vision_pick_phase_started) >= VISION_PICK_STOP_HOLD_MS &&
                    vision_target_region >= 1U && vision_target_region <= 4U &&
                    vision_pick_attempts <= VISION_PICK_MAX_REPEATS &&
                    StepperTask_RequestMango(vision_target_region)) {
                    ++vision_pick_attempts;
                    vision_pick_id = vision_target_region;
                    VisionStrike_Accept(&vision_pick);
                    VisionAlign_Cancel(&vision_align);
                    vision_pick_phase = PICK_WORK;
                }
                break;
            default: break;
            }
            if (vision_pick_phase != PICK_IDLE && !vision_detection_timed_out) {
                VisionAlign_HoldYaw();
                return;
            }
        }
    }
    if (vision_detection_timed_out) {
detection_timeout:
        vision_detection_timed_out = 1U;
        vision_align.last_frames = camera.frames;
        vision_target_region = 0U;
        vision_pick_phase = PICK_IDLE;
        VisionAlign_Cancel(&vision_align);
        vision_align.phase = VISION_ALIGN_WAIT;
        VisionAlign_HardStop();
        return;
    }
#endif

    if (!ready && (!manual_active || was_automatic)) {
        VisionAlign_HardStop();
        return;
    }
    if (command.owned && command.x_mps == 0.0f && command.y_mps == 0.0f) {
        VisionAlign_HoldYaw();
        return;
    }
    if (command.owned) {
        /* Brake residual longitudinal motion before driving the opposite way. */
        float forward_feedback = (MOTOR_A.Encoder + MOTOR_B.Encoder +
                                  MOTOR_C.Encoder + MOTOR_D.Encoder) * 0.25f;
        if ((command.x_mps > 0.0f && forward_feedback < -VISION_PICK_STOP_MPS) ||
            (command.x_mps < 0.0f && forward_feedback > VISION_PICK_STOP_MPS)) {
            VisionAlign_HoldYaw();
            return;
        }
        ManualMecanum_Drive(command.x_mps, command.y_mps, 1U);
        return;
    }
#endif
    ManualMecanum_Drive(forward, left, 0U);
}

void Chassis_NavigationDrive(float forward, float left, float yaw_target)
{
    if (!manual_heading_locked) HeadingControl_Reset();
    manual_heading_target = yaw_target;
    manual_heading_locked = 1U;
    VisionAlign_Drive(forward, left);
}

void PS2_control(void)
{
    if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550) {
        /* 左杆前后控制前后，右杆左右控制横移；取消摇杆自转和扳机替代。 */
        float rc_velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
        float forward = (float)GamePadInterface->LY - 127.0f;
        float left = 127.0f - (float)GamePadInterface->RX;
        if (fabsf(forward) < 20.0f) forward = 0.0f;
        if (fabsf(left) < 20.0f) left = 0.0f;
        VisionAlign_Drive(forward * rc_velocity / (127.0f * 1000.0f),
                           left * rc_velocity / (127.0f * 1000.0f));
        return;
    }

	float LX=127,LY=127,RX=127;
	float ThrottleTri = 255;

	//前进摇杆
	LY = GamePadInterface->LY - 127;

	//左右横移
	LX = 127 - GamePadInterface->LX;

	//顺逆时针
	RX = 127 - GamePadInterface->RX;

	//摇杆微小幅度过滤
	if( fabs(LY)<20 ) LY = 0;
	if( fabs(LX)<20 ) LX = 0;
	if( fabs(RX)<20 ) RX = 0;

	//针对xbox360手柄，扳机为模拟量时，允许使用扳机控制
	if( gamepad_brand == Xbox360 )
	{
		//前进摇杆无值时,采用扳机的值
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

	//针对usb有线手柄,在非模拟量模式下的摇杆值映射
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
	  //对PS2手柄控制命令进行处理

	Move_X = (LY/127.0f) * RC_Velocity * (ThrottleTri/255.0f);
	Move_Y = (LX/127.0f) * RC_Velocity;
	Move_Z = (PI/2) * (RX/127.0f) * ( RC_Velocity/500.0f );

//		Move_X=LX*RC_Velocity/128;
//		Move_Y=LY*RC_Velocity/128;
//		Move_Z=RY*(PI/2)/128;

	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;
		}

	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
		Move_X=Move_X/1000;
		Move_Y=Move_Y/1000;
		Move_Z=Move_Z;

		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);

	//根据加速度反应震动情况传达到手柄
	#include "bsp_gamepad.h"

	//Z轴数据判断震动强度
	float now_z = imu.accel.z/1671.84f;
	static float last_z = 0;
	float strength = fabs(last_z - now_z);

	//震动映射到手柄
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
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100;
    int Threshold=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity;
    if (Car_Mode == Mec_Car || Car_Mode == Mec_Car_V550) {
        /* 航模通道：CH2 为左杆前后，CH1 为右杆左右；CH3/CH4 不参与麦轮平移与自转。 */
        float rc_velocity = target_limit_float(RC_Velocity, 0.0f, MANUAL_RC_MAX_MMPS);
        int forward = target_limit_int(Remoter_Ch2, 1000, 2000) - 1500;
        int right = target_limit_int(Remoter_Ch1, 1000, 2000) - 1500;
        if (forward > -Threshold && forward < Threshold) forward = 0;
        if (right > -Threshold && right < Threshold) right = 0;
        if (thrice > 0U) {
            --thrice;
            manual_heading_locked = 0U;
            Move_X = Move_Y = Move_Z = 0.0f;
            Drive_Motor(0.0f, 0.0f, 0.0f);
            return;
        }
        ManualMecanum_Drive(forward * rc_velocity / 500000.0f,
                           -right * rc_velocity / 500000.0f, 0U);
        return;
    }

		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500;

	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation.
		//右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500;

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;

		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;

		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
    Move_X= LX*Remote_RCvelocity/500;
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;

		//Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==Mec_Car_V550)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==FourWheel_Car_V550)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}

	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
		Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;
		Move_Z=Move_Z;

	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;

		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
    u8 tmp;

    //传入任务的频率
    tmp=KEY_Scan(RATE_100_HZ,0);
		if(Check==0)
		{
        if (ROUTE_DEMO_ENABLE && (tmp == single_click || tmp == double_click))
        {
            /* The rebuild navigation task owns USER debounce and start/stop. */
            return;
        }

    //单击 或 手柄同时按下两边的下扳机，开启自动回充
    if(tmp==single_click )
	{
		Allow_Recharge=!Allow_Recharge;
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //双击 或 手柄同时按下两边的摇杆,更新陀螺仪
    else if(tmp==double_click)
	{
		ImuData_copy(&imu.Deviation_gyro,&imu.gyro);
        ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}

    //长按 切换页面
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
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr;
		OriginalEncoder.A=Read_Encoder(2);
		OriginalEncoder.B=Read_Encoder(3);
		OriginalEncoder.C=Read_Encoder(4);
		OriginalEncoder.D=Read_Encoder(5);

	//计算纠偏系数
	float LeftWheelDiff = wheelCoefficient(LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(LineDiffParam,0);

	//test_num=OriginalEncoder.B;

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
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
		//编码器原始数据转换为车轮速度，单位m/s
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
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
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

    float translation_step = step;
    /* Establish route wheel speed promptly; visual motion keeps its current ramp. */
    if (RouteIO_ControlOwned() && !VisionAlign_ControlOwned() && translation_step < 0.03f)
        translation_step = 0.03f;

	if	   (vx>0) 	smooth_control.VX+=translation_step;
	else if(vx<0)		smooth_control.VX-=translation_step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;

	if	   (vy>0)   smooth_control.VY+=translation_step;
	else if(vy<0)		smooth_control.VY-=translation_step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;

    /* 航向减速、停车和反向制动立即跟随指令；仅同向加速保留平滑，避免旧方向延迟。 */
    if (vz == 0.0f || vz * smooth_control.VZ < 0.0f || fabsf(vz) <= fabsf(smooth_control.VZ)) {
        smooth_control.VZ = vz;
    } else if (vz > smooth_control.VZ) {
        smooth_control.VZ += step;
    } else {
        smooth_control.VZ -= step;
    }

	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
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
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//如果连续6次接近满幅输出，判断为电机乱转，让电机失能
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;
}

//PWM消除函数
void auto_pwm_clear(void)
{
	//小车姿态简易判断
	float y_accle = (float)(imu.accel.y/1671.84f);//Y轴加速度实际值
	float z_accle = (float)(imu.accel.z/1671.84f);//Z轴加速度实际值
	float diff;

	//计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;

//	debug_show_diff = diff;

	//PWM消除检测
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f || MOTOR_D.Target != 0.0f )
	{
		start_check_flag = 1;//标记需要清空PWM
		wait_clear_times = 0;//复位清空计时
		start_clear = 0;     //复位清除标志


		//运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //当目标速度由非0变0时，开始计时 2.5 秒，若小车不在斜坡状态下，清空pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//开启清除pwm
				else clear_done_once = 1;//小车在斜坡上，标记已完成清除

				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if( clear_done_once )
	{
		//小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if( diff > 8.8f )
		{
			//完成清除后pwm再次积累，重新清除
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//开启清除pwm
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
