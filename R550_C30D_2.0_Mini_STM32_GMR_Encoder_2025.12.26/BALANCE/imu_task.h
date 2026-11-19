#ifndef __IMU_TASK_H
#define __IMU_TASK_H
#include "system.h"

#include "task_priority.h"
#define IMU_TASK_PRIO      TASK_PRIORITY_IMU
#define IMU_STK_SIZE       256
#define IMU_TASK_RATE      RATE_100_HZ

typedef struct{
	short x;
	short y;
	short z;
}IMU_BASE_t;

//imu三轴数据,包含偏差值和原始值
typedef struct{
	IMU_BASE_t Deviation_accel;//偏差值,用于纠正误差.该值采集后不会发生改变
	IMU_BASE_t Deviation_gyro;
	IMU_BASE_t Original_accel;
	IMU_BASE_t Original_gyro;
	IMU_BASE_t gyro;
	IMU_BASE_t accel;
}IMU_DATA_t;

extern IMU_DATA_t imu;
typedef struct {
    float yaw_rad, rate_rad_s;
    uint32_t tick_ms;
    uint8_t ready;
} IMU_HeadingFrame;
void Imu_ReadHeading(IMU_HeadingFrame *out);

//对外接口
void MPU6050_task(void *pvParameters);
void ICM20948_task(void *pvParameters);

void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val);

#endif
