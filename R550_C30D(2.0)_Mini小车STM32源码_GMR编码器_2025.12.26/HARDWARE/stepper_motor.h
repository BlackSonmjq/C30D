#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "a4988.h"

#define STEPPER_MOTOR_ARR_MIN 1999u
#define STEPPER_MOTOR_ARR_MAX 19999u

/* 初始化 A4988 引脚以及两路 STEP 脉冲定时器。 */
void StepperMotor_Init(void);

/* 停止指定电机的 STEP 脉冲并将 STEP 拉低。
   不改变外部 EN 电平，因此电机仍可能保持力矩。 */
void StepperMotor_Disable(u8 motor);

/* 初始化后设置指定电机的目标匀速 ARR。定时器每 1 us 计数一次，
   脉冲频率约为 1000000/(ARR+1) Hz。允许 1999～19999（约 500～50 Hz）。
   运行中修改会从后续脉冲逐步生效；成功返回 1，参数无效返回 0。 */
u8 StepperMotor_SetARR(u8 motor, u16 arr);

/* 按带符号脉冲数阻塞运行：正数使 DIR 为高，负数使 DIR 为低。
   达到目标脉冲数后自动停止 PWM；返回已完成的脉冲周期数。
   参数无效或脉冲数为 0 时返回 0。同一路电机不可并发调用。 */
u32 StepperMotor_MovePulses(u8 motor, s32 signed_pulses);

#endif
