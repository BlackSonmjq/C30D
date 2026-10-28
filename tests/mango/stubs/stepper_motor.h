#ifndef MOCK_STEPPER_MOTOR_H
#define MOCK_STEPPER_MOTOR_H
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int32_t s32;
#define A4988_MOTOR_Left 1U
#define A4988_MOTOR_Right 2U
void StepperMotor_Disable(u8 motor);
u8 StepperMotor_SetSpeed(u8 motor, u16 arr);
u8 StepperMotor_StartMove(u8 motor, s32 pulses);
u32 StepperMotor_GetCompletedPulses(u8 motor);
u8 StepperMotor_IsRunning(u8 motor);
void StepperMotor_ResetCompletedPulses(u8 motor);
#endif
