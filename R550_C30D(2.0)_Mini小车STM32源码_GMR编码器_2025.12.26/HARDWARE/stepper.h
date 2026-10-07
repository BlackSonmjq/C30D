#ifndef __STEPPER_H
#define __STEPPER_H
#include "sys.h"

typedef enum
{
	STEPPER_1 = 0, /* PC8 / TIM8_CH3 */
	STEPPER_2      /* PB15 / TIM12_CH2 */
} StepperChannel;

void stepper_init(void);
/* speed_hz is steps/second (16..20000); even timer periods preserve 50% duty. */
/* 0 pulse_count stops; returns 0 for an invalid channel or speed. */
u8 stepper_run(StepperChannel channel, u32 pulse_count, u32 speed_hz);
void stepper_stop(StepperChannel channel);

#endif
