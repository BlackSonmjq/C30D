/* 后面由 runner 追加实际的编码器换算函数和 Balance_task 中的里程调用。 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "balance.h"
#include "chassis_odometry.h"
enum { Omni_Car=1,Akm_Car=2,Diff_Car=3,FourWheel_Car=4,Tank_Car=5,FourWheel_Car_V550=7 };
#define CONTROL_FREQUENCY 100
typedef struct { int A,B,C,D; } RawEncoder;
TestSystem SysVal={CONTROL_DELAY};
TestSmooth smooth_control;
TestMotor MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;
TestImu imu;
uint8_t Car_Mode=Mec_Car,EN=1,Flag_Stop;
float Voltage=12;
static RawEncoder OriginalEncoder;
static unsigned LineDiffParam=50;
static float Wheel_perimeter=0.075f*3.14159265358979323846f;
static float Encoder_precision=4.0f*500.0f*30.0f;
static int counts[4];
static uint32_t clock_ms;
TickType_t xTaskGetTickCount(void){return clock_ms;}
static int Read_Encoder(unsigned timer){int value=counts[timer-2];counts[timer-2]=0;return value;}
static void production_odom_call(void);
void Get_Velocity_Form_Encoder(void);
int main(void)
{
    const uint32_t gaps[]={10,20,50};
    for(unsigned i=0;i<sizeof(gaps)/sizeof(gaps[0]);++i){
        ChassisOdomFrame p;
        float expected=120.0f*Wheel_perimeter/Encoder_precision*1000.0f;
        ChassisOdom_Reset(0);clock_ms=10;
        Get_Velocity_Form_Encoder();production_odom_call();
        /* 原始计数经底盘 C/D 极性处理后应为 + - + -，即纯车身左移。 */
        counts[0]=120;counts[1]=-120;counts[2]=-120;counts[3]=120;
        clock_ms+=gaps[i];Get_Velocity_Form_Encoder();production_odom_call();
        ChassisOdom_Read(&p);
        if(!p.valid||fabsf(p.y_mm-expected)>0.00001f||fabsf(p.x_mm)>0.00001f){
            fprintf(stderr,"Production bridge failed at %lu ms: %.6f / %.6f mm\n",
                    (unsigned long)gaps[i],p.y_mm,expected);return 1;
        }
        /* 同一时刻再次读取已清零计数，不得重复累计上一份位移。 */
        Get_Velocity_Form_Encoder();production_odom_call();ChassisOdom_Read(&p);
        if(fabsf(p.y_mm-expected)>0.00001f)return 1;
    }
    puts("Production encoder polarity/scaling and Balance_task displacement bridge passed at 10/20/50 ms.");
    return 0;
}
