#include "servo.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <stdio.h>

/* Finite polling budget per byte/TC; independent of RTOS tick and interrupts.
 * This is a fault escape bound, not a calibrated wall-clock delay.
 */
#ifndef SERVO_POLL_LIMIT
#define SERVO_POLL_LIMIT 1000000UL
#endif
#define SERVO_COMMAND_LIMIT 63U
static uint8_t initialized;
static volatile uint8_t sending;
static volatile uint8_t rx_length, frame_length;
static volatile char rx_buffer[SERVO_FRAME_CAPACITY];

static uint8_t valid_id(uint8_t id)
{
    return id >= 1U && id <= SERVO_COUNT;
}

void Servo_ClearRx(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    rx_length = 0;
    frame_length = 0;
    __set_PRIMASK(mask);
}

void Servo_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef irq;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    USART_Cmd(UART4, DISABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_Init(GPIOC, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200;
    uart.USART_Mode = USART_Mode_Rx;
    USART_Init(UART4, &uart);
    USART_HalfDuplexCmd(UART4, ENABLE);
    Servo_ClearRx();
    sending = 0;
    irq.NVIC_IRQChannel = UART4_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 6;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
    initialized = 1;
}

static uint8_t wait_flag(uint16_t flag)
{
    uint32_t remaining = SERVO_POLL_LIMIT;
    while (USART_GetFlagStatus(UART4, flag) == RESET) {
        if (--remaining == 0U) return 0;
    }
    return 1;
}

Servo_Result Servo_SendCommand(const char *command)
{
    uint32_t mask;
    uint16_t length = 0, i;
    Servo_Result result = SERVO_OK;
    volatile uint32_t discard;
    if (command == NULL || __get_IPSR() != 0U) return SERVO_INVALID_ARGUMENT;
    while (length <= SERVO_COMMAND_LIMIT && command[length] != '\0') ++length;
    if (length < 2U || length > SERVO_COMMAND_LIMIT ||
        command[0] != '#' || command[length - 1U] != '!') return SERVO_INVALID_ARGUMENT;
    if (!initialized) return SERVO_NOT_INITIALIZED;
    mask = __get_PRIMASK();
    __disable_irq();
    if (sending) {
        __set_PRIMASK(mask);
        return SERVO_BUSY;
    }
    sending = 1;
    /* Switch atomically relative to UART IRQ. Discard any partial old reply. */
    USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
    UART4->CR1 &= ~USART_CR1_RE;
    discard = UART4->SR;
    discard = UART4->DR;
    (void)discard;
    rx_length = 0;
    UART4->CR1 |= USART_CR1_TE;
    __set_PRIMASK(mask);
    for (i = 0; i < length - 1U; ++i) {
        if (!wait_flag(USART_FLAG_TXE)) {
            result = SERVO_TX_TIMEOUT;
            break;
        }
        USART_SendData(UART4, (uint8_t)command[i]);
    }
    if (result == SERVO_OK && !wait_flag(USART_FLAG_TXE)) result = SERVO_TX_TIMEOUT;
    /* Protect the final byte and direction change against task preemption:
     * otherwise a fast reply can be lost while RE is still disabled.
     * Normally about one byte (87 us); a UART fault uses the finite poll bound.
     */
    mask = __get_PRIMASK();
    __disable_irq();
    if (result == SERVO_OK) USART_SendData(UART4, (uint8_t)command[length - 1U]);
    /* TXE is not enough: the final stop bit must leave the shift register. */
    if (!wait_flag(USART_FLAG_TC)) result = SERVO_TX_TIMEOUT;
    UART4->CR1 &= ~USART_CR1_TE;
    UART4->CR1 |= USART_CR1_RE;
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    sending = 0;
    __set_PRIMASK(mask);
    return result;
}

Servo_Result Servo_SetID(uint8_t old_id, uint8_t new_id)
{
    char command[20];
    if (old_id == 255U || !valid_id(new_id)) return SERVO_INVALID_ARGUMENT;
    snprintf(command, sizeof(command), "#%03uPID%03u!", (unsigned)old_id, (unsigned)new_id);
    return Servo_SendCommand(command);
}

Servo_Result Servo_SetMode(uint8_t id, uint8_t mode)
{
    char command[20];
    if (!valid_id(id) || mode < 1U || mode > 4U) return SERVO_INVALID_ARGUMENT;
    snprintf(command, sizeof(command), "#%03uPMOD%u!", (unsigned)id, (unsigned)mode);
    return Servo_SendCommand(command);
}

Servo_Result Servo_MovePulse(uint8_t id, uint16_t pulse, uint16_t time_ms)
{
    char command[24];
    if (!valid_id(id) || pulse < 500U || pulse > 2500U || time_ms > 9999U)
        return SERVO_INVALID_ARGUMENT;
    snprintf(command, sizeof(command), "#%03uP%04uT%04u!",
             (unsigned)id, (unsigned)pulse, (unsigned)time_ms);
    return Servo_SendCommand(command);
}

Servo_Result Servo_MoveAngle(uint8_t id, uint16_t degrees, uint16_t time_ms)
{
    if (degrees > 270U) return SERVO_INVALID_ARGUMENT;
    /* 270-degree modes 1/2 only; caller selects the intended direction. */
    return Servo_MovePulse(id, (uint16_t)(500U + ((uint32_t)degrees * 2000U + 135U) / 270U), time_ms);
}

Servo_Result Servo_Home(uint8_t id, uint16_t time_ms)
{
    return Servo_MovePulse(id, SERVO_HOME_PULSE, time_ms);
}

Servo_Result Servo_Twist(uint8_t id, uint16_t time_ms)
{
    return Servo_MovePulse(id, SERVO_HALF_TURN_PULSE, time_ms);
}

Servo_Result Servo_Stop(uint8_t id)
{
    char command[16];
    if (!valid_id(id)) return SERVO_INVALID_ARGUMENT;
    snprintf(command, sizeof(command), "#%03uPDST!", (unsigned)id);
    return Servo_SendCommand(command);
}

Servo_Result Servo_ReadPosition(uint8_t id)
{
    char command[16];
    if (!valid_id(id)) return SERVO_INVALID_ARGUMENT;
    snprintf(command, sizeof(command), "#%03uPRAD!", (unsigned)id);
    return Servo_SendCommand(command);
}

void Servo_RxByteHandler(uint8_t byte)
{
    if (frame_length != 0U) return;
    if (byte == '#') {
        rx_length = 0;
    } else if (rx_length == 0U) {
        return;
    }
    if (rx_length >= SERVO_FRAME_CAPACITY - 1U) {
        rx_length = 0;
        return;
    }
    rx_buffer[rx_length++] = (char)byte;
    if (byte == '!') {
        rx_buffer[rx_length] = '\0';
        frame_length = rx_length;
        rx_length = 0;
    }
}

Servo_Result Servo_GetFrame(char *buffer, uint16_t capacity, uint16_t *length)
{
    uint32_t mask;
    uint8_t i, count;
    Servo_Result result;
    if (buffer == NULL || length == NULL) return SERVO_INVALID_ARGUMENT;
    mask = __get_PRIMASK();
    __disable_irq();
    count = frame_length;
    *length = count;
    if (count == 0U) result = SERVO_NO_FRAME;
    else if (capacity <= count) result = SERVO_BUFFER_TOO_SMALL;
    else {
        for (i = 0; i <= count; ++i) buffer[i] = rx_buffer[i];
        frame_length = 0;
        result = SERVO_OK;
    }
    __set_PRIMASK(mask);
    return result;
}

void UART4_IRQHandler(void)
{
    uint32_t status = UART4->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)UART4->DR; /* SR then DR clears receive errors. */
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) rx_length = 0;
        else if (status & USART_SR_RXNE) Servo_RxByteHandler(byte);
    }
}
