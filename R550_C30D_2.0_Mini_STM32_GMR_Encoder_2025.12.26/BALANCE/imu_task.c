#include "imu_task.h"

//imu数据结构体
IMU_DATA_t imu;

static long gyro_sum_x;
static long gyro_sum_y;
static long gyro_sum_z;
static long accel_sum_x;
static long accel_sum_y;
static long accel_sum_z;
static u16 imu_calibration_samples;

/* Average all stationary startup samples instead of using one noisy sample. */
static void ImuStartupCalibration_Update(void)
{
    gyro_sum_x += imu.gyro.x;
    gyro_sum_y += imu.gyro.y;
    gyro_sum_z += imu.gyro.z;
    accel_sum_x += imu.accel.x;
    accel_sum_y += imu.accel.y;
    accel_sum_z += imu.accel.z;
    imu_calibration_samples++;

    imu.Deviation_gyro.x = (short)(gyro_sum_x / imu_calibration_samples);
    imu.Deviation_gyro.y = (short)(gyro_sum_y / imu_calibration_samples);
    imu.Deviation_gyro.z = (short)(gyro_sum_z / imu_calibration_samples);
    imu.Deviation_accel.x = (short)(accel_sum_x / imu_calibration_samples);
    imu.Deviation_accel.y = (short)(accel_sum_y / imu_calibration_samples);
    imu.Deviation_accel.z = (short)(accel_sum_z / imu_calibration_samples);
}

void MPU6050_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //此任务以100Hz的频率运行
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));
        //Get acceleration sensor data
        MPU6050_Get_Accelscope();

        //Get gyroscope data
        MPU6050_Get_Gyroscope(); //得到陀螺仪数据

        //Average the stationary IMU samples collected during startup.
        //开机静止期间，对IMU零偏进行多次平均
        if(SysVal.Time_count<CONTROL_DELAY)
            ImuStartupCalibration_Update();

    }
}


void ICM20948_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 100Hz
        //此任务以100Hz的频率运行
        vTaskDelayUntil(&lastWakeTime, F2T(IMU_TASK_RATE));

        //Get acceleration sensor data
        ICM20948_Get_Accel(); //得到加速度传感器数据

        //Get gyroscope data
        ICM20948_Get_Gyroscope(); //得到陀螺仪数据

        //Average the stationary IMU samples collected during startup.
        //开机静止期间，对IMU零偏进行多次平均
        if(SysVal.Time_count<CONTROL_DELAY)
            ImuStartupCalibration_Update();

#if 0 // 未使用磁力计数据,不开启采集
        static u8 mag_count=0;
        //Get magnetometer data
        mag_count++;
        if(mag_count>=13)	 //磁力计最大采样速率为8hz，因此每进入这个函数13次才采样一次磁力计信息
        {
            invMSMagRead(&magnet[0], &magnet[1], &magnet[2]); //得到磁力计数据
            mag_count=0;
        }
#endif


    }
}


/**************************************************************************
函数功能：复制imu的数值
入口参数：需要赋值的结构体变量，被复制的结构体变量
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val)
{
	req_val->x = copied_val -> x;
	req_val->y = copied_val -> y;
	req_val->z = copied_val -> z;
}


