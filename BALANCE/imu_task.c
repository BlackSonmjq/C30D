#include "imu_task.h"
#include "nav_stm32.h"
#include "nav_config.h"

IMU_DATA_t imu;
static int16_t read16(uint8_t address,uint8_t high,uint8_t low)
{
    return (int16_t)(((uint16_t)I2C_ReadOneByte(address,high)<<8)|I2C_ReadOneByte(address,low));
}
static float wheel_max(void)
{
    float m=float_abs(MOTOR_A.Encoder);
    if(float_abs(MOTOR_B.Encoder)>m) m=float_abs(MOTOR_B.Encoder);
    if(float_abs(MOTOR_C.Encoder)>m) m=float_abs(MOTOR_C.Encoder);
    if(float_abs(MOTOR_D.Encoder)>m) m=float_abs(MOTOR_D.Encoder);
    return m;
}
static void sample_task(int icm)
{
    u32 wake=getSysTickCnt(),last=NavBoard_Millis(),now;
    int32_t sums[3]={0,0,0};
    unsigned count=0;
    uint8_t calibrated=0;
    float bias[3]={0,0,0},yaw=0,dt,rate;
    while(1) {
        int valid,ready;
        int16_t gx,gy,gz;
        vTaskDelayUntil(&wake,F2T(IMU_TASK_RATE));
        now=NavBoard_Millis(); dt=(float)(uint32_t)(now-last)*0.001f; last=now;
        valid=icm?(ICM20948_getDeviceID()==REG_VAL_WIA):
                  (MPU6050_getDeviceID()==MPU6050_DEFAULT_ADDRESS);
        if(!valid) {
            if(!calibrated) { count=0; memset(sums,0,sizeof(sums)); }
            NavBoard_PublishImu(yaw,0.0f,0); continue;
        }
        if(icm) {
            gx=read16(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_H,REG_ADD_GYRO_XOUT_L);
            gy=read16(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_H,REG_ADD_GYRO_YOUT_L);
            gz=read16(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_H,REG_ADD_GYRO_ZOUT_L);
            imu.accel.x=read16(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H,REG_ADD_ACCEL_XOUT_L);
            imu.accel.y=read16(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_H,REG_ADD_ACCEL_YOUT_L);
            imu.accel.z=read16(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_H,REG_ADD_ACCEL_ZOUT_L);
        } else {
            gx=read16(devAddr,MPU6050_RA_GYRO_XOUT_H,MPU6050_RA_GYRO_XOUT_L);
            gy=read16(devAddr,MPU6050_RA_GYRO_YOUT_H,MPU6050_RA_GYRO_YOUT_L);
            gz=read16(devAddr,MPU6050_RA_GYRO_ZOUT_H,MPU6050_RA_GYRO_ZOUT_L);
            imu.accel.x=read16(devAddr,MPU6050_RA_ACCEL_XOUT_H,MPU6050_RA_ACCEL_XOUT_L);
            imu.accel.y=read16(devAddr,MPU6050_RA_ACCEL_YOUT_H,MPU6050_RA_ACCEL_YOUT_L);
            imu.accel.z=read16(devAddr,MPU6050_RA_ACCEL_ZOUT_H,MPU6050_RA_ACCEL_ZOUT_L);
        }
        imu.Original_gyro.x=gx; imu.Original_gyro.y=gy; imu.Original_gyro.z=gz;
        if(!calibrated) {
            /* Only a stationary startup average; never recalibrate at a waypoint. */
            if(wheel_max()>NAV_STOP_WHEEL_MPS || abs(gx)>1000 || abs(gy)>1000 || abs(gz)>1000 ||
               abs(imu.accel.z)<12000 || abs(imu.accel.z)>20000) {
                count=0; memset(sums,0,sizeof(sums));
            } else {
                sums[0]+=gx; sums[1]+=gy; sums[2]+=gz;
                if(++count>=NAV_IMU_CAL_SAMPLES) {
                    unsigned i;
                    for(i=0;i<3;i++) bias[i]=(float)sums[i]/(float)count;
                    imu.Deviation_gyro.x=(short)bias[0]; imu.Deviation_gyro.y=(short)bias[1];
                    imu.Deviation_gyro.z=(short)bias[2]; calibrated=1; yaw=0;
                }
            }
        }
        imu.gyro.x=(short)((float)gx-bias[0]); imu.gyro.y=(short)((float)gy-bias[1]);
        imu.gyro.z=(short)((float)gz-bias[2]);
        ready=calibrated && dt>0.0f && dt<0.1f;
        rate=0.0f;
        if(ready) {
            rate=NAV_GYRO_SIGN*((float)gz-bias[2])/NAV_GYRO_LSB_PER_DPS*0.0174532925f;
            yaw+=rate*dt;
            if(yaw>3.14159265f) yaw-=6.28318531f;
            if(yaw< -3.14159265f) yaw+=6.28318531f;
        }
        Led_Count=calibrated?300:1;
        NavBoard_PublishImu(yaw,rate,ready);
    }
}
void MPU6050_task(void *p) { (void)p; sample_task(0); }
void ICM20948_task(void *p) { (void)p; sample_task(1); }
void ImuData_copy(IMU_BASE_t *out,const IMU_BASE_t *in) { *out=*in; }
