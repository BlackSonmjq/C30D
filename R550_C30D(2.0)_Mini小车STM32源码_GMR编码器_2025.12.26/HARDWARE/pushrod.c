#include "pushrod.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

typedef struct
{
    GPIO_TypeDef *port_a;
    uint16_t pin_a;
    GPIO_TypeDef *port_b;
    uint16_t pin_b;
} Pushrod_Pins;

/* 引脚映射只在此处维护。修改后须重新核对硬件连接和时钟配置。 */
static const Pushrod_Pins s_pins[PUSHROD_COUNT] =
{
    {GPIOC, GPIO_Pin_0,  GPIOC, GPIO_Pin_1},
    {GPIOC, GPIO_Pin_2,  GPIOC, GPIO_Pin_3},
    {GPIOC, GPIO_Pin_5,  GPIOC, GPIO_Pin_10},
    {GPIOC, GPIO_Pin_11, GPIOD, GPIO_Pin_15}
};

static uint8_t s_initialized = 0U;
static Pushrod_Direction s_direction[PUSHROD_COUNT];

/* 先将两个输入都拉低，再输出单一方向；从不主动输出 IA=IB=1。 */
static void Pushrod_Write(uint8_t index, Pushrod_Direction direction)
{
    const Pushrod_Pins *pins = &s_pins[index];

    GPIO_ResetBits(pins->port_a, pins->pin_a);
    GPIO_ResetBits(pins->port_b, pins->pin_b);
    if (direction == PUSHROD_FORWARD)
    {
        GPIO_SetBits(pins->port_a, pins->pin_a);
    }
    else if (direction == PUSHROD_REVERSE)
    {
        GPIO_SetBits(pins->port_b, pins->pin_b);
    }
    s_direction[index] = direction;
}

void Pushrod_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_initialized = 0U;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC |
                          RCC_AHB1Periph_GPIOD, ENABLE);

    /* 在改为输出之前，先把输出锁存器清零，避免初始化时发出运动脉冲。 */
    for (i = 0U; i < PUSHROD_COUNT; ++i)
    {
        Pushrod_Write(i, PUSHROD_STOP);
    }

    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;

    /* 只初始化选中的 8 个引脚，不改动同一端口的其他引脚。 */
    for (i = 0U; i < PUSHROD_COUNT; ++i)
    {
        gpio.GPIO_Pin = s_pins[i].pin_a;
        GPIO_Init(s_pins[i].port_a, &gpio);
        gpio.GPIO_Pin = s_pins[i].pin_b;
        GPIO_Init(s_pins[i].port_b, &gpio);
    }
    s_initialized = 1U;
    __set_PRIMASK(primask);
}

Pushrod_Result Pushrod_Control(uint8_t id, Pushrod_Direction direction)
{
    uint8_t index;
    uint32_t primask;
    Pushrod_Result result = PUSHROD_OK;

    if (id < 1U || id > PUSHROD_COUNT ||
        (direction != PUSHROD_STOP && direction != PUSHROD_FORWARD &&
         direction != PUSHROD_REVERSE))
    {
        return PUSHROD_INVALID_ARGUMENT;
    }

    index = (uint8_t)(id - 1U);
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_initialized == 0U)
    {
        result = PUSHROD_NOT_INITIALIZED;
    }
    else if (direction != PUSHROD_STOP &&
             s_direction[index] != PUSHROD_STOP &&
             direction != s_direction[index])
    {
        /* 拒绝直接反接运动：只停止，不自动启动相反方向。
         * 这里只防止单次命令直接换向，不判断机械是否真正静止。
         * 调用方应停止发运动命令，等推杆停稳，再发一次新的命令。
         */
        Pushrod_Write(index, PUSHROD_STOP);
        result = PUSHROD_REVERSAL_BLOCKED;
    }
    else if (direction != s_direction[index])
    {
        /* 相同方向的重复命令不切换电平，避免重复调用产生窄脉冲。 */
        Pushrod_Write(index, direction);
    }
    __set_PRIMASK(primask);
    return result;
}

Pushrod_Result Pushrod_Forward(uint8_t id)
{
    return Pushrod_Control(id, PUSHROD_FORWARD);
}

Pushrod_Result Pushrod_Reverse(uint8_t id)
{
    return Pushrod_Control(id, PUSHROD_REVERSE);
}

Pushrod_Result Pushrod_Stop(uint8_t id)
{
    return Pushrod_Control(id, PUSHROD_STOP);
}

void Pushrod_StopAll(void)
{
    uint8_t i;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (s_initialized != 0U)
    {
        for (i = 0U; i < PUSHROD_COUNT; ++i)
        {
            Pushrod_Write(i, PUSHROD_STOP);
        }
    }
    __set_PRIMASK(primask);
}
