#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "a4988.h"

/* Initializes the A4988 GPIOs and the two STEP PWM timers. */
void StepperMotor_Init(void);

/* Stops STEP PWM and drives STEP low. The external A4988 EN is unchanged,
   so the motor can still have holding torque. */
void StepperMotor_Disable(u8 motor);

/* Set the target cruising ARR for one motor after Init. Timer tick = 1 us,
   so nominal pulse rate is 1000000/(ARR+1) Hz. No software speed limits;
   ARR is still a 16-bit value. Use a nonzero ARR for valid STEP pulses.
   Takes effect on subsequent pulses, including during an active move;
   returns 1 on success or 0 for an invalid motor or zero ARR. */
u8 StepperMotor_SetSpeed(u8 motor, u16 arr);

/* Blocking move by signed pulse count. Positive sets DIR high; negative
   sets DIR low. Automatically stops PWM after the requested pulse count.
   Returns the number of completed pulse periods (0 for invalid/zero input).
   Timer interrupts drive pulses; the calling RTOS task sleeps while waiting.
   Call only with interrupts enabled, outside critical sections/ISRs.
   Do not call concurrently for the same motor. */
u32 StepperMotor_MovePulses(u8 motor, s32 signed_pulses);

/* Internal timer ISR dispatch. Must not call FreeRTOS APIs. */
void StepperMotor_UpdateIRQ(u8 motor);

#endif
