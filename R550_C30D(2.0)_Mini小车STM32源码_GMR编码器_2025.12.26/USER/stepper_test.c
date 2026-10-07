#include "stepper_test.h"
#include "stepper_motor.h"
#include "delay.h"

/* One motor-shaft revolution for a 1.8-degree motor at 1/16 microstepping.
   MS1/MS2/MS3 must be configured on the driver hardware. */
#define STEPPER_TEST_FULL_STEPS_PER_REV 200
#define STEPPER_TEST_MICROSTEPS 16
#define STEPPER_TEST_PULSES (STEPPER_TEST_FULL_STEPS_PER_REV * STEPPER_TEST_MICROSTEPS)
#define STEPPER_TEST_ARR 49u

#define STEPPER_TEST_CYCLE_PAUSE_MS 1000

static void Stepper_TestMotor(u8 motor)
{
    StepperMotor_SetSpeed(motor, STEPPER_TEST_ARR);
    StepperMotor_MovePulses(motor, STEPPER_TEST_PULSES);
    StepperMotor_Disable(motor);
}

void Stepper_TestLoop(void)
{
    delay_init(168);
    StepperMotor_Init();
    while (1) {
        Stepper_TestMotor(A4988_MOTOR_Left);
        delay_ms(STEPPER_TEST_CYCLE_PAUSE_MS);
    }
}
