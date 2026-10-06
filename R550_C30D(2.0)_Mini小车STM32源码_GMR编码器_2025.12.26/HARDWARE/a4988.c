#include "a4988.h"
#include "delay.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void A4988_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    /* Load safe output levels before changing the pins from input to output. */
    GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_SetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_9);
    GPIO_ResetBits(GPIOC, GPIO_Pin_7 | GPIO_Pin_8);

    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOC, &gpio);
}

void A4988_SetDir(u8 motor, u8 high)
{
    GPIO_TypeDef *port;
    u16 pin;

    if (motor == A4988_MOTOR_1) {
        port = GPIOB;
        pin = GPIO_Pin_14;
    } else if (motor == A4988_MOTOR_2) {
        port = GPIOC;
        pin = GPIO_Pin_7;
    } else {
        return;
    }

    if (high) GPIO_SetBits(port, pin);
    else      GPIO_ResetBits(port, pin);
}

void A4988_Enable(u8 motor, u8 enable)
{
    u16 pin;

    if (motor == A4988_MOTOR_1)      pin = GPIO_Pin_6;
    else if (motor == A4988_MOTOR_2) pin = GPIO_Pin_9;
    else return;

    /* A4988 EN is active low. */
    if (enable) GPIO_ResetBits(GPIOC, pin);
    else        GPIO_SetBits(GPIOC, pin);
}

void A4988_Step(u8 motor)
{
    GPIO_TypeDef *port;
    u16 pin;

    if (motor == A4988_MOTOR_1) {
        port = GPIOB;
        pin = GPIO_Pin_15;
    } else if (motor == A4988_MOTOR_2) {
        port = GPIOC;
        pin = GPIO_Pin_8;
    } else {
        return;
    }

    /* Allow DIR/EN setup and keep both STEP levels above the 1 us minimum. */
    delay_us(2);
    GPIO_SetBits(port, pin);
    delay_us(2);
    GPIO_ResetBits(port, pin);
    delay_us(2);
}
