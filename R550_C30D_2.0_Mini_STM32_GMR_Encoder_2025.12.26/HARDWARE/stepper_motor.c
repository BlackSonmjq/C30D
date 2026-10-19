#include "stepper_motor.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "FreeRTOS.h"
#include "task.h"

#define STEPPER_START_PERIOD_US 20000u
#define STEPPER_RUN_PERIOD_US   5000u
#define STEPPER_RAMP_PULSES     200u
#define STEPPER_MAX_PERIOD_CHANGE_US 150u

static volatile u8 stepper_running[2];
static volatile u32 stepper_target_period_us[2];
static volatile u32 stepper_completed[2];
static u32 stepper_count[2], stepper_ramp[2], stepper_period[2];
static void StepperMotor_StartNext(u8 motor);

/* Priority 4 is above the FreeRTOS syscall threshold (5).
   These interrupts MUST NOT call any FreeRTOS API. */
static void StepperMotor_InitIRQ(IRQn_Type irq)
{
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = (u8)irq;
    nvic.NVIC_IRQChannelPreemptionPriority = 4;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_ClearPendingIRQ(irq);
    NVIC_Init(&nvic);
}

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
    output.TIM_Pulse = STEPPER_START_PERIOD_US / 2u;
    output.TIM_OCPolarity = TIM_OCPolarity_High;
    output.TIM_OCIdleState = TIM_OCIdleState_Reset;
    if (timer == TIM12) TIM_OC2Init(timer, &output);
    else                TIM_OC3Init(timer, &output);

    TIM_SelectOnePulseMode(timer, TIM_OPMode_Single);
    /* Software UG loads the prescaler but must not count as a pulse. */
    TIM_UpdateRequestConfig(timer, TIM_UpdateSource_Regular);
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
    StepperMotor_InitIRQ(TIM8_BRK_TIM12_IRQn);
    StepperMotor_InitIRQ(TIM8_UP_TIM13_IRQn);
}

void StepperMotor_Disable(u8 motor)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    GPIO_TypeDef *port;
    u16 pin;
    u32 primask;

    if (timer == 0) return;
    primask = __get_PRIMASK();
    __disable_irq();
    TIM_ITConfig(timer, TIM_IT_Update, DISABLE);
    TIM_Cmd(timer, DISABLE);
    if (timer == TIM8) TIM_CtrlPWMOutputs(timer, DISABLE);
    TIM_CCxCmd(timer, (timer == TIM12) ? TIM_Channel_2 : TIM_Channel_3,
               TIM_CCx_Disable);

    port = (motor == A4988_MOTOR_Left) ? GPIOB : GPIOC;
    pin = (motor == A4988_MOTOR_Left) ? GPIO_Pin_15 : GPIO_Pin_8;
    GPIO_ResetBits(port, pin);
    StepperMotor_StepPinMode(motor, GPIO_Mode_OUT);
    TIM_ClearFlag(timer, TIM_FLAG_Update);
    stepper_running[motor - 1u] = 0;
    __set_PRIMASK(primask);
}

u8 StepperMotor_SetSpeed(u8 motor, u16 arr)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    u32 period;

    if (timer == 0 || arr == 0) return 0;

    period = (u32)arr + 1u;
    stepper_target_period_us[motor - 1u] = period;
    if (!stepper_running[motor - 1u]) {
        TIM_SetAutoreload(timer, arr);
        if (timer == TIM12)
            TIM_SetCompare2(timer, period / 2u);
        else
            TIM_SetCompare3(timer, period / 2u);
    }
    return 1;
}

/* Each next pulse is restarted in the timer ISR, not by a schedulable task.
   OPM also prevents extra pulses if an interrupt is delayed. */
static void StepperMotor_StartNext(u8 motor)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    u32 index = motor - 1u;
    u32 completed = stepper_completed[index];
    u32 count = stepper_count[index], ramp = stepper_ramp[index];
    u32 period = stepper_period[index];
    u32 target = stepper_target_period_us[index];
    u32 start_period = (target > STEPPER_START_PERIOD_US) ?
        target : STEPPER_START_PERIOD_US;
    u32 desired;

    if (ramp && completed < ramp) {
        desired = start_period - (start_period - target) * completed / ramp;
    } else if (ramp && completed >= count - ramp) {
        desired = target + (start_period - target) *
            (completed - (count - ramp)) / ramp;
    } else {
        desired = target;
    }
    if (desired > period + STEPPER_MAX_PERIOD_CHANGE_US)
        period += STEPPER_MAX_PERIOD_CHANGE_US;
    else if (desired + STEPPER_MAX_PERIOD_CHANGE_US < period)
        period -= STEPPER_MAX_PERIOD_CHANGE_US;
    else
        period = desired;

    stepper_period[index] = period;
    TIM_SetAutoreload(timer, period - 1u);
    if (timer == TIM12) TIM_SetCompare2(timer, period / 2u);
    else               TIM_SetCompare3(timer, period / 2u);
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
}

/* Called only from the existing shared timer interrupt vectors. */
void StepperMotor_UpdateIRQ(u8 motor)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    u32 index = motor - 1u;

    if (timer == 0 || TIM_GetITStatus(timer, TIM_IT_Update) == RESET) return;
    TIM_ClearITPendingBit(timer, TIM_IT_Update);
    if (!stepper_running[index]) return;
    ++stepper_completed[index];
    if (stepper_completed[index] >= stepper_count[index])
        StepperMotor_Disable(motor);
    else
        StepperMotor_StartNext(motor);
}

u32 StepperMotor_MovePulses(u8 motor, s32 signed_pulses)
{
    TIM_TypeDef *timer = StepperMotor_Timer(motor);
    u32 index, count, target, primask;

    if (timer == 0) return 0;
    StepperMotor_Disable(motor);
    if (signed_pulses == 0) return 0;
    index = motor - 1u;
    count = (signed_pulses < 0) ?
        (u32)(-(signed_pulses + 1)) + 1u : (u32)signed_pulses;

    primask = __get_PRIMASK();
    __disable_irq();
    stepper_completed[index] = 0;
    stepper_count[index] = count;
    stepper_ramp[index] = (count / 2u < STEPPER_RAMP_PULSES) ?
        count / 2u : STEPPER_RAMP_PULSES;
    target = stepper_target_period_us[index];
    stepper_period[index] = (target > STEPPER_START_PERIOD_US) ?
        target : STEPPER_START_PERIOD_US;
    A4988_SetDir(motor, signed_pulses > 0);
    /* Reset CNT while the STEP pin is still a low GPIO, avoiding a
       spurious rising edge when restarting an externally stopped move. */
    TIM_SetCounter(timer, 0);
    TIM_ClearFlag(timer, TIM_FLAG_Update);
    StepperMotor_StepPinMode(motor, GPIO_Mode_AF);
    TIM_CCxCmd(timer, (timer == TIM12) ? TIM_Channel_2 : TIM_Channel_3,
               TIM_CCx_Enable);
    if (timer == TIM8) TIM_CtrlPWMOutputs(timer, ENABLE);
    stepper_running[index] = 1;
    TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
    StepperMotor_StartNext(motor);
    __set_PRIMASK(primask);

    while (stepper_running[index]) {
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
            vTaskDelay(1);
        /* Before the scheduler starts, timer interrupts still drive STEP. */
    }
    return stepper_completed[index];
}
