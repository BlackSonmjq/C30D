#include "stepper.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

#define STEPPER_TIMER_TICK_HZ 1000000UL
#define STEPPER_MAX_SPEED_HZ 20000UL
#define STEPPER_STEP1_TIMER_HZ 168000000UL
#define STEPPER_STEP2_TIMER_HZ 84000000UL

static volatile u32 stepper1_remaining_pulses;
static volatile u32 stepper2_remaining_pulses;

static TIM_TypeDef *stepper_timer(StepperChannel channel)
{
	return channel == STEPPER_1 ? TIM8 : TIM12;
}

static u8 stepper_channel_valid(StepperChannel channel)
{
	return channel == STEPPER_1 || channel == STEPPER_2;
}

static volatile u32 *stepper_remaining_pulses(StepperChannel channel)
{
	return channel == STEPPER_1 ?
		&stepper1_remaining_pulses : &stepper2_remaining_pulses;
}

static void set_stepper_compare(StepperChannel channel, uint16_t value)
{
	if (channel == STEPPER_1)
		TIM_SetCompare3(TIM8, value);
	else
		TIM_SetCompare2(TIM12, value);
}

static void set_stepper_output(StepperChannel channel, FunctionalState state)
{
	if (channel == STEPPER_1)
		TIM_CCxCmd(TIM8, TIM_Channel_3, state);
	else
		TIM_CCxCmd(TIM12, TIM_Channel_2, state);
}

static void configure_stepper_timebase(StepperChannel channel, uint16_t period)
{
	TIM_TypeDef *timer = stepper_timer(channel);

	TIM_Cmd(timer, DISABLE);
	TIM_SetAutoreload(timer, period);
	TIM_SetCounter(timer, 0);
	TIM_GenerateEvent(timer, TIM_EventSource_Update);
	TIM_ClearITPendingBit(timer, TIM_IT_Update);
}

static void disable_stepper(StepperChannel channel)
{
	TIM_TypeDef *timer;
	volatile u32 *remaining;

	if (!stepper_channel_valid(channel))
		return;

	timer = stepper_timer(channel);
	remaining = stepper_remaining_pulses(channel);
	*remaining = 0;

	if (channel == STEPPER_1)
	{
		TIM_ITConfig(timer, TIM_IT_Update, DISABLE);
		TIM_ITConfig(timer, TIM_IT_CC3, DISABLE);
		TIM_ClearITPendingBit(timer, TIM_IT_Update);
		TIM_ClearITPendingBit(timer, TIM_IT_CC3);
	}
	else
	{
		TIM_ITConfig(timer, TIM_IT_Update, DISABLE);
		TIM_ITConfig(timer, TIM_IT_CC2, DISABLE);
		TIM_ClearITPendingBit(timer, TIM_IT_Update);
		TIM_ClearITPendingBit(timer, TIM_IT_CC2);
	}

	TIM_Cmd(timer, DISABLE);
	set_stepper_output(channel, DISABLE);
	set_stepper_compare(channel, 0);
	TIM_SetCounter(timer, 0);
}

static u8 calculate_period(u32 speed_hz, uint16_t *period, uint16_t *pulse)
{
	uint64_t ticks;

	if (speed_hz == 0 || speed_hz > STEPPER_MAX_SPEED_HZ)
		return 0;

	ticks = ((uint64_t)STEPPER_TIMER_TICK_HZ + speed_hz / 2) / speed_hz;
	if (ticks & 1)
	{
		/* An even number of 1 us ticks gives an exact 50% duty cycle. */
		uint64_t lower_ticks = ticks - 1;
		uint64_t upper_ticks = ticks + 1;
		uint64_t lower_error;
		uint64_t upper_error;

		if (upper_ticks > 65536)
			ticks = lower_ticks;
		else
		{
			lower_error = (uint64_t)STEPPER_TIMER_TICK_HZ -
				(uint64_t)speed_hz * lower_ticks;
			upper_error = (uint64_t)speed_hz * upper_ticks -
				STEPPER_TIMER_TICK_HZ;
			ticks = lower_error * upper_ticks <= upper_error * lower_ticks ?
				lower_ticks : upper_ticks;
		}
	}
	if (ticks < 2 || ticks > 65536)
		return 0;

	*period = (uint16_t)(ticks - 1);
	*pulse = (uint16_t)(ticks / 2);
	return 1;
}

void stepper_init(void)
{
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef timebase;
	TIM_OCInitTypeDef output_compare;
	NVIC_InitTypeDef nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;

	gpio.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &gpio);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);

	gpio.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &gpio);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);

	TIM_TimeBaseStructInit(&timebase);
	timebase.TIM_Prescaler = (STEPPER_STEP1_TIMER_HZ / STEPPER_TIMER_TICK_HZ) - 1;
	timebase.TIM_Period = 65535;
	timebase.TIM_CounterMode = TIM_CounterMode_Up;
	timebase.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM8, &timebase);

	timebase.TIM_Prescaler = (STEPPER_STEP2_TIMER_HZ / STEPPER_TIMER_TICK_HZ) - 1;
	TIM_TimeBaseInit(TIM12, &timebase);

	TIM_OCStructInit(&output_compare);
	output_compare.TIM_OCMode = TIM_OCMode_PWM1;
	output_compare.TIM_OutputState = TIM_OutputState_Enable;
	output_compare.TIM_Pulse = 0;
	output_compare.TIM_OCPolarity = TIM_OCPolarity_High;
	output_compare.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC3Init(TIM8, &output_compare);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM12, &output_compare);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_ARRPreloadConfig(TIM12, ENABLE);

	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM_CCxCmd(TIM8, TIM_Channel_3, DISABLE);
	TIM_CCxCmd(TIM12, TIM_Channel_2, DISABLE);

	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
	NVIC_Init(&nvic);
}

u8 stepper_run(StepperChannel channel, u32 pulse_count, u32 speed_hz)
{
	TIM_TypeDef *timer;
	volatile u32 *remaining;
	uint16_t period;
	uint16_t pulse;

	if (!stepper_channel_valid(channel))
		return 0;
	if (pulse_count == 0)
	{
		disable_stepper(channel);
		return 1;
	}
	if (!calculate_period(speed_hz, &period, &pulse))
		return 0;

	disable_stepper(channel);
	timer = stepper_timer(channel);
	remaining = stepper_remaining_pulses(channel);
	configure_stepper_timebase(channel, period);
	set_stepper_compare(channel, pulse);
	TIM_GenerateEvent(timer, TIM_EventSource_Update);
	TIM_ClearITPendingBit(timer, TIM_IT_Update);
	*remaining = pulse_count;

	if (channel == STEPPER_1)
	{
		if (pulse_count == 1)
			TIM_ITConfig(timer, TIM_IT_CC3, ENABLE);
		else
			TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
	}
	else
	{
		if (pulse_count == 1)
			TIM_ITConfig(timer, TIM_IT_CC2, ENABLE);
		else
			TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
	}

	set_stepper_output(channel, ENABLE);
	TIM_Cmd(timer, ENABLE);
	return 1;
}

void stepper_stop(StepperChannel channel)
{
	disable_stepper(channel);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
		if (stepper1_remaining_pulses > 1)
		{
			stepper1_remaining_pulses--;
			if (stepper1_remaining_pulses == 1)
			{
				TIM_ITConfig(TIM8, TIM_IT_Update, DISABLE);
				TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
			}
		}
	}
}

void TIM8_CC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
		disable_stepper(STEPPER_1);
	}
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
		if (stepper2_remaining_pulses > 1)
		{
			stepper2_remaining_pulses--;
			if (stepper2_remaining_pulses == 1)
			{
				TIM_ITConfig(TIM12, TIM_IT_Update, DISABLE);
				TIM_ITConfig(TIM12, TIM_IT_CC2, ENABLE);
			}
		}
	}
	if (TIM_GetITStatus(TIM12, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM12, TIM_IT_CC2);
		disable_stepper(STEPPER_2);
	}
}
