#include "stepper_test.h"
#include "stepper_motor.h"
#include "delay.h"

#define STEPPER_TEST_PULSES 800
#define STEPPER_TEST_ARR 4999u
#define STEPPER_TEST_PAUSE_MS 500
#define STEPPER_TEST_CYCLE_PAUSE_MS 1000

static void Stepper_TestMotor(u8 motor)
{
    /* 用脉冲数的正负号选择方向。到达目标后会自动停止 PWM；
       再调用停止函数，确保 STEP 保持低电平。 */
    StepperMotor_MovePulses(motor, STEPPER_TEST_PULSES);
    StepperMotor_Disable(motor);
    delay_ms(STEPPER_TEST_PAUSE_MS);

    StepperMotor_MovePulses(motor, -STEPPER_TEST_PULSES);
    StepperMotor_Disable(motor);
    delay_ms(STEPPER_TEST_PAUSE_MS);
}

void Stepper_TestLoop(void)
{
    delay_init(168);
    StepperMotor_Init();
    StepperMotor_SetSpeed(A4988_MOTOR_Left, STEPPER_TEST_ARR);
    StepperMotor_SetSpeed(A4988_MOTOR_Right, STEPPER_TEST_ARR);

    while (1) {
        Stepper_TestMotor(A4988_MOTOR_Left);
        Stepper_TestMotor(A4988_MOTOR_Right);
        delay_ms(STEPPER_TEST_CYCLE_PAUSE_MS);
    }
}
