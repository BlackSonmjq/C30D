#include "pushrod.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

static const uint16_t s_pins[PUSHROD_COUNT] =
{
    GPIO_Pin_0,
    GPIO_Pin_1,
    GPIO_Pin_2,
    GPIO_Pin_3
};

static uint8_t s_initialized = 0U;

void Pushrod_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_initialized = 0U;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

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
        GPIO_SetBits(GPIOC, s_pins[id - 1U]);
    }
    else
    {
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
