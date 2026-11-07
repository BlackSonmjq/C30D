#include "pushrod.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

#define PUSHROD_PINS (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)
#define PUSHROD_TIMER_HZ 10000UL /* 100 us/count; 5000 ms fits a 16-bit timer. */

static const uint16_t s_pins[PUSHROD_COUNT] =
{
    GPIO_Pin_0,
    GPIO_Pin_1,
    GPIO_Pin_2,
    GPIO_Pin_3
};

static uint8_t s_initialized = 0U;
static volatile uint8_t s_timed_id;
static volatile uint8_t s_timed_done;

/* Caller holds PRIMASK, or is the sole TIM6 interrupt handler. */
static void Pushrod_DisarmTimer(void)
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->DIER = 0;
    TIM6->SR = 0;
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}

void Pushrod_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_ClocksTypeDef clocks;
    NVIC_InitTypeDef irq;
    uint32_t timer_clock;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_initialized = 0U;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    /* TIM6 is reserved for pushrod cutoff. APB1 timer clock doubles when
     * APB1 is divided; current board clock gives 84 MHz / 8400 = 10 kHz. */
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_clock *= 2U;
    TIM6->CR1 = TIM_CR1_OPM | TIM_CR1_URS;
    TIM6->CR2 = 0;
    TIM6->PSC = timer_clock / PUSHROD_TIMER_HZ - 1U;
    TIM6->ARR = 0xFFFFU;
    TIM6->CNT = 0;
    TIM6->EGR = TIM_EGR_UG;
    Pushrod_DisarmTimer();
    s_timed_id = s_timed_done = 0;

    /* Priority 0 can preempt RTOS critical sections. The IRQ uses no RTOS API. */
    irq.NVIC_IRQChannel = TIM6_DAC_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 0;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);

    GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio);

    s_initialized = 1U;
    __set_PRIMASK(primask);
}

static Pushrod_Result Pushrod_Set(uint8_t id, uint8_t started)
{
    uint32_t primask;

    if (id < 1U || id > PUSHROD_COUNT)
    {
        return PUSHROD_INVALID_ARGUMENT;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_initialized == 0U)
    {
        __set_PRIMASK(primask);
        return PUSHROD_NOT_INITIALIZED;
    }

    if (started != 0U)
    {
        if (s_timed_id != 0U && !s_timed_done) {
            __set_PRIMASK(primask);
            return PUSHROD_BUSY;
        }
        s_timed_id = s_timed_done = 0;
        GPIO_SetBits(GPIOC, s_pins[id - 1U]);
    }
    else
    {
        if (s_timed_id == id) {
            Pushrod_DisarmTimer();
            s_timed_id = s_timed_done = 0;
        }
        GPIO_ResetBits(GPIOC, s_pins[id - 1U]);
    }
    __set_PRIMASK(primask);

    return PUSHROD_OK;
}

Pushrod_Result Pushrod_Start(uint8_t id)
{
    return Pushrod_Set(id, 1U);
}

Pushrod_Result Pushrod_Stop(uint8_t id)
{
    return Pushrod_Set(id, 0U);
}

Pushrod_Result Pushrod_StartTimed(uint8_t id, uint16_t duration_ms)
{
    uint32_t primask;
    if (id < 1U || id > PUSHROD_COUNT || duration_ms == 0U || duration_ms > 6553U)
        return PUSHROD_INVALID_ARGUMENT;
    primask = __get_PRIMASK();
    __disable_irq();
    if (!s_initialized) {
        __set_PRIMASK(primask);
        return PUSHROD_NOT_INITIALIZED;
    }
    if (s_timed_id != 0U && !s_timed_done) {
        __set_PRIMASK(primask);
        return PUSHROD_BUSY;
    }
    Pushrod_DisarmTimer();
    GPIO_ResetBits(GPIOC, PUSHROD_PINS);
    s_timed_id = id;
    s_timed_done = 0;
    TIM6->ARR = (uint32_t)duration_ms * (PUSHROD_TIMER_HZ / 1000U) - 1U;
    TIM6->CNT = 0;
    TIM6->EGR = TIM_EGR_UG; /* Load PSC and restart its divider for every run. */
    TIM6->SR = 0;          /* UG must never be mistaken for a timeout. */
    TIM6->DIER = TIM_DIER_UIE;
    GPIO_SetBits(GPIOC, s_pins[id - 1U]);
    TIM6->CR1 |= TIM_CR1_CEN;
    __set_PRIMASK(primask);
    return PUSHROD_OK;
}

uint8_t Pushrod_TimedDone(uint8_t id)
{
    return id != 0U && id == s_timed_id && s_timed_done;
}

void TIM6_DAC_IRQHandler(void)
{
    if ((TIM6->SR & TIM_SR_UIF) && (TIM6->DIER & TIM_DIER_UIE)) {
        /* Cut power before publishing completion, even if tasks are delayed. */
        if (s_timed_id >= 1U && s_timed_id <= PUSHROD_COUNT) {
            GPIO_ResetBits(GPIOC, s_pins[s_timed_id - 1U]);
            s_timed_done = 1;
        }
        Pushrod_DisarmTimer();
    }
}
