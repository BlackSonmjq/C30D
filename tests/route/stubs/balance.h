#ifndef ROUTE_TEST_BALANCE_H
#define ROUTE_TEST_BALANCE_H
#include "FreeRTOS.h"
#include "task.h"
enum { Mec_Car=0, Mec_Car_V550=6, CONTROL_DELAY=1000 };
typedef struct { uint32_t Time_count; } TestSystem;
typedef struct { float VX,VY,VZ; } TestSmooth;
typedef struct { float Encoder,Target; int Motor_Pwm; } TestMotor;
typedef struct { struct { short z; } gyro; } TestImu;
extern TestSystem SysVal;
extern TestSmooth smooth_control;
extern TestMotor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
extern TestImu imu;
extern uint8_t Car_Mode,EN,Flag_Stop;
extern float Voltage;
void Drive_Motor(float vx,float vy,float vz);
#endif
