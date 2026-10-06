#include "stepper_motor.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

#define STEPPER_PULSE_HIGH_US   5u
#define STEPPER_START_PERIOD_US 20000u
#define STEPPER_RUN_PERIOD_US   5000u
#define STEPPER_RAMP_PULSES     200u
#define STEPPER_MAX_PERIOD_CHANGE_US 150u

static volatile u8 stepper_running[2];
static volatile u32 stepper_target_period_us[2];

static TIM_TypeDef *StepperMotor_Timer(u8 motor)
{
    if (motor == A4988_MOTOR_Left) return TIM12;
    if (motor == A4988_MOTOR_Right) return TIM8;
    return 0;
}

static void StepperMotor_StepPinMode(u8 motor, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    GPIO_TypeDef *port = (motor == A4988_MOTOR_Left) ? GPIOB : GPIOC;

    gpio.GPIO_Pin = (motor == A4988_MOTOR_Left) ? GPIO_Pin_15 : GPIO_Pin_8;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(port, &gpio);
}

static void StepperMotor_InitTimer(TIM_TypeDef *timer, u16 prescaler)
{
    TIM_TimeBaseInitTypeDef base;
    TIM_OCInitTypeDef output;

    TIM_DeInit(timer);
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = prescaler;
    base.TIM_Period = STEPPER_START_PERIOD_US - 1u;
    TIM_TimeBaseInit(timer, &base);

    TIM_OCStructInit(&output);
    output.TIM_OCMode = TIM_OCMode_PWM2;
    output.TIM_OutputState = TIM_OutputState_Enable;
    output.TIM_Pulse = STEPPER_START_PERIOD_US - STEPPER_PULSE_HIGH_US;
    output.TIM_OCPolarity = TIM_OCPolarity_High;
    output.TIM_OCIdleState = TIM_OCIdleState_Reset;
    if (timer == TIM12) TIM_OC2Init(timer, &output);
    else                TIM_OC3Init(timer, &output);

    TIM_SelectOnePulseMode(timer, TIM_OPMode_Single);
    TIM_Cmd(timer, DISABLE);
    if (timer == TIM8) TIM_CtrlPWMOutputs(timer, DISABLE);
}

void StepperMotor_Init(void)
{
    A4988_Init();
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    /* 工程系统时钟为 168 MHz；TIM12 为 84 MHz，TIM8 为 168 MHz。
       以下预分频值使两个定时器均以 1 MHz 计数，即每计数一次为 1 us。 */
    StepperMotor_InitTimer(TIM12, 83);
    StepperMotor_InitTimer(TIM8, 167);
    stepper_running[0] = 0;
    stepper_running[1] = 0;
    stepper_target_period_us[0] = STEPPER_RUN_PERIOD_US;
    stepper_target_period_us[1] = STEPPER_RUN_PERIOD_US;
}

void StepperMotor_Disable(u8 motor)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    GPIO_TypeDef *port;
    u16 pin;

    if (timer == 0) return;
    stepper_running[motor - 1u] = 0;
    TIM_Cmd(timer, DISABLE);
    if (timer == TIM8) TIM_CtrlPWMOutputs(timer, DISABLE);
    TIM_CCxCmd(timer, (timer == TIM12) ? TIM_Channel_2 : TIM_Channel_3,
               TIM_CCx_Disable);

    port = (motor == A4988_MOTOR_Left) ? GPIOB : GPIOC;
    pin = (motor == A4988_MOTOR_Left) ? GPIO_Pin_15 : GPIO_Pin_8;
    GPIO_ResetBits(port, pin);
    StepperMotor_StepPinMode(motor, GPIO_Mode_OUT);
}

u8 StepperMotor_SetSpeed(u8 motor, u16 arr)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    u32 period;

    if (timer == 0 || arr < STEPPER_MOTOR_ARR_MIN ||
        arr > STEPPER_MOTOR_ARR_MAX) return 0;

    period = (u32)arr + 1u;
    stepper_target_period_us[motor - 1u] = period;
    if (!stepper_running[motor - 1u]) {
        TIM_SetAutoreload(timer, arr);
        if (timer == TIM12)
            TIM_SetCompare2(timer, period - STEPPER_PULSE_HIGH_US);
        else
            TIM_SetCompare3(timer, period - STEPPER_PULSE_HIGH_US);
    }
    return 1;
}

u32 StepperMotor_MovePulses(u8 motor, s32 signed_pulses)//
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    u32 count, completed, ramp, period, target, desired;

    if (timer == 0) return 0;
    StepperMotor_Disable(motor);
    if (signed_pulses == 0) return 0;

    /* 单独处理有符号最小值，避免直接取负造成溢出。 */
    count = (signed_pulses < 0) ?
        (u32)(-(signed_pulses + 1)) + 1u : (u32)signed_pulses;
    ramp = (count / 2u < STEPPER_RAMP_PULSES) ?
        count / 2u : STEPPER_RAMP_PULSES;
    A4988_SetDir(motor, signed_pulses > 0);
    StepperMotor_StepPinMode(motor, GPIO_Mode_AF);
    TIM_CCxCmd(timer, (timer == TIM12) ? TIM_Channel_2 : TIM_Channel_3,
               TIM_CCx_Enable);
    if (timer == TIM8) TIM_CtrlPWMOutputs(timer, ENABLE);
    stepper_running[motor - 1u] = 1;
    period = STEPPER_START_PERIOD_US;

    for (completed = 0; completed < count && stepper_running[motor - 1u];
         completed++) {
        target = stepper_target_period_us[motor - 1u];
        if (ramp && completed < ramp) {
            desired = STEPPER_START_PERIOD_US -
                (STEPPER_START_PERIOD_US - target) *
                completed / ramp;
        } else if (ramp && completed >= count - ramp) {
            desired = target +
                (STEPPER_START_PERIOD_US - target) *
                (completed - (count - ramp)) / ramp;
        } else {
            desired = target;
        }

        /* 运行中修改 ARR 时，限制每个脉冲的周期变化，避免速度突变。 */
        if (desired > period + STEPPER_MAX_PERIOD_CHANGE_US)
            period += STEPPER_MAX_PERIOD_CHANGE_US;
        else if (desired + STEPPER_MAX_PERIOD_CHANGE_US < period)
            period -= STEPPER_MAX_PERIOD_CHANGE_US;
        else
            period = desired;

        TIM_SetAutoreload(timer, period - 1u);
        if (timer == TIM12)
            TIM_SetCompare2(timer, period - STEPPER_PULSE_HIGH_US);
        else
            TIM_SetCompare3(timer, period - STEPPER_PULSE_HIGH_US);
        TIM_GenerateEvent(timer, TIM_EventSource_Update);
        TIM_ClearFlag(timer, TIM_FLAG_Update);
        TIM_SetCounter(timer, 0);
        TIM_Cmd(timer, ENABLE);

        /* PWM2 在周期末尾 5 us 输出高电平；更新事件使输出回到低电平，
           单脉冲模式随后自动停止定时器。 */
        while (TIM_GetFlagStatus(timer, TIM_FLAG_Update) == RESET) {
            if (!stepper_running[motor - 1u]) {
                StepperMotor_Disable(motor);
                return completed;
            }
        }
    }

    StepperMotor_Disable(motor);
    return completed;
}
