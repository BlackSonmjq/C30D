#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "a4988.h"

/* 初始化两路 A4988 的 GPIO 和 STEP 定时器，上电不自动输出脉冲。 */
void StepperMotor_Init(void);

/* 停止 STEP 并拉低输出，保留实际已输出的脉冲计数。
 * 此函数不控制外接 EN，因此驱动使能时电机仍可保持力矩。 */
void StepperMotor_Disable(u8 motor);

/* 初始化后设置单路目标巡航 ARR：计数周期 1 us，名义脉冲频率为
 * 1000000/(ARR+1) Hz；参数为非零 16 位数，不额外限制软件速度。
 * 运动中修改会在后续脉冲逐步生效，实际频率包含中断重启耗时。
 * 成功返回 1，电机编号无效或 ARR 为 0 时返回 0。 */
u8 StepperMotor_SetSpeed(u8 motor, u16 arr);

/* 按有符号脉冲数阻塞运动：正数 DIR 高、负数 DIR 低，到数自动停止。
 * 返回实际计数值，参数无效或脉冲数为 0 时返回 0。
 * 定时器中断驱动脉冲，调用任务通过休眠等待；必须在开中断且非临界区
 * 的任务上下文调用，不能在 ISR 调用，也不能并发控制同一电机。 */
u32 StepperMotor_MovePulses(u8 motor, s32 signed_pulses);

/* 非阻塞运动会重新开始计数，定长运动由定时器中断到数停止。
 * 连续运动 forward=1 时 DIR 高、0 时 DIR 低；同一电机只能有一个控制方。
 * 获取最终计数前先调用 Disable，以计入停止瞬间已输出的上升沿。
 * 计数为指令 STEP 边沿，包含细分，不能检测机械失步；u32 在 2^32 回绕。 */
u8 StepperMotor_StartMove(u8 motor, s32 signed_pulses);
u8 StepperMotor_StartContinuous(u8 motor, u8 forward);
u32 StepperMotor_GetCompletedPulses(u8 motor);
/* 查询定时器是否仍在输出本次运动的脉冲。 */
u8 StepperMotor_IsRunning(u8 motor);
/* 每轮新任务开始时停止电机并清零旧计数，避免零步击打复用上一轮计数。 */
void StepperMotor_ResetCompletedPulses(u8 motor);

/* 定时器中断内部派发接口，禁止调用 FreeRTOS API。 */
void StepperMotor_UpdateIRQ(u8 motor);

#endif
