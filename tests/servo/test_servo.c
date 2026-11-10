#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SERVO_POLL_LIMIT 100U
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/servo.c"

USART_TypeDef mock_uart;
static char tx[128];
static unsigned tx_size, checks, tc_checks, tc_delay, fail_flag, nested, isr;
static uint32_t irq_mask;
static unsigned rx_irq_enabled, nvic_enabled, cleared_pending;
static uint8_t pc10_high = 1;
#define CHECK(e) do { ++checks; if (!(e)) { printf("FAIL line %u: %s\n", (unsigned)__LINE__, #e); exit(1); } } while (0)
uint32_t mock_primask(void) { return irq_mask; }
void mock_disable_irq(void) { irq_mask = 1; }
void mock_set_primask(uint32_t value) { irq_mask = value; }
uint32_t mock_ipsr(void) { return isr; }
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio) {
    CHECK(port == GPIOC && gpio->GPIO_Pin == GPIO_Pin_10);
    CHECK(gpio->GPIO_Mode == GPIO_Mode_AF && gpio->GPIO_OType == GPIO_OType_OD);
}
void GPIO_PinAFConfig(void *port, uint16_t source, uint8_t af) { CHECK(port == GPIOC && source == 10 && af == GPIO_AF_UART4); }
uint8_t GPIO_ReadInputDataBit(void *port, uint16_t pin) {
    CHECK(port == GPIOC && pin == GPIO_Pin_10); return pc10_high;
}
void RCC_AHB1PeriphClockCmd(uint32_t p, FunctionalState e) { CHECK(p == RCC_AHB1Periph_GPIOC && e == ENABLE); }
void RCC_APB1PeriphClockCmd(uint32_t p, FunctionalState e) { CHECK(p == RCC_APB1Periph_UART4 && e == ENABLE); }
void NVIC_Init(NVIC_InitTypeDef *irq) {
    CHECK(irq->NVIC_IRQChannel == UART4_IRQn);
    nvic_enabled = irq->NVIC_IRQChannelCmd == ENABLE;
}
void NVIC_ClearPendingIRQ(int irq) { CHECK(irq == UART4_IRQn); ++cleared_pending; }
void USART_StructInit(USART_InitTypeDef *uart) { memset(uart, 0, sizeof(*uart)); }
void USART_Init(USART_TypeDef *p, USART_InitTypeDef *uart) { CHECK(uart->USART_BaudRate == 115200); p->CR1 = uart->USART_Mode; }
void USART_DeInit(USART_TypeDef *p) { CHECK(p == UART4); memset(p, 0, sizeof(*p)); rx_irq_enabled = 0; }
void USART_Cmd(USART_TypeDef *p, FunctionalState e) { CHECK(p == UART4); (void)e; }
void USART_HalfDuplexCmd(USART_TypeDef *p, FunctionalState e) { CHECK(p == UART4 && e == ENABLE); }
void USART_ITConfig(USART_TypeDef *p, uint16_t i, FunctionalState e) {
    CHECK(p == UART4 && i == USART_IT_RXNE);
    rx_irq_enabled = e == ENABLE;
}
FlagStatus USART_GetFlagStatus(USART_TypeDef *p, uint16_t f) {
    if (f == USART_FLAG_TC) {
        ++tc_checks;
        if (tc_delay) { --tc_delay; return RESET; }
    }
    return f == fail_flag ? RESET : SET;
}
void USART_SendData(USART_TypeDef *p, uint16_t data) {
    CHECK((p->CR1 & (USART_CR1_TE | USART_CR1_RE)) == USART_CR1_TE);
    CHECK(tx_size < sizeof(tx) - 1);
    tx[tx_size++] = (char)data;
    tx[tx_size] = 0;
    if (nested) { nested = 0; CHECK(Servo_Home(2, 100) == SERVO_BUSY); }
}
static void expect(const char *command) {
    CHECK(strcmp(tx, command) == 0);
    CHECK((UART4->CR1 & (USART_CR1_TE | USART_CR1_RE)) == (receiving ? USART_CR1_RE : 0U));
    CHECK(rx_irq_enabled == receiving);
    CHECK(!sending && !irq_mask);
    tx_size = 0; tx[0] = 0;
}
static void receive(const char *frame) {
    while (*frame) {
        if (rx_irq_enabled && (UART4->CR1 & USART_CR1_RE)) {
            UART4->SR = USART_SR_RXNE;
            UART4->DR = (uint8_t)*frame;
            UART4_IRQHandler();
        }
        ++frame;
    }
}
static void arm_reply(void) {
    CHECK(Servo_ReadPosition(1) == SERVO_OK); expect("#001PRAD!");
    CHECK(receiving && rx_irq_enabled);
}
static void check_idle(void) {
    CHECK(!receiving && !rx_irq_enabled);
    CHECK(!(UART4->CR1 & (USART_CR1_TE | USART_CR1_RE)));
}
int main(void) {
    char reply[64], too_long[70]; uint16_t length; unsigned before;
    Servo_Diagnostics diag;
    CHECK(Servo_StartRxProbe() == SERVO_NOT_INITIALIZED);
    CHECK(Servo_Home(1, 1500) == SERVO_NOT_INITIALIZED);
    UART4->CR1 = 0xFFFFFFFFU; /* Simulate peripheral state left by a bootloader. */
    Servo_Init(); CHECK(tx_size == 0 && nvic_enabled && cleared_pending == 1);
    check_idle();
    receive("noise#OK!"); CHECK(frame_length == 0); check_idle();
    CHECK(Servo_StartRxProbe() == SERVO_OK && tx_size == 0);
    CHECK(receiving && rx_irq_enabled);
    CHECK(Servo_Home(1, 1000) == SERVO_BUSY && tx_size == 0);
    CHECK(Servo_StartRxProbe() == SERVO_BUSY);
    Servo_GetDiagnostics(&diag);
    CHECK(diag.pc10_high == 1 && diag.receiving == 1);
    CHECK(diag.irq_count == 0 && diag.byte_count == 0 && diag.error_count == 0);
    receive("x#OK!");
    Servo_GetDiagnostics(&diag); check_idle();
    CHECK(diag.irq_count == 5 && diag.byte_count == 5 && diag.error_flags == 0);
    CHECK(Servo_StartRxProbe() == SERVO_BUSY); /* Probe owns the reply until cancelled. */
    Servo_ClearRx();
    CHECK(Servo_StartRxProbe() == SERVO_OK && tx_size == 0);
    receive("#p");
    UART4->SR = USART_SR_FE | USART_SR_RXNE; UART4->DR = 0; UART4_IRQHandler();
    pc10_high = 0;
    Servo_GetDiagnostics(&diag); check_idle();
    CHECK(diag.irq_count == 3 && diag.byte_count == 2 && diag.error_count == 1);
    CHECK(diag.error_flags == USART_SR_FE && diag.last_status == (USART_SR_FE | USART_SR_RXNE));
    CHECK(diag.pc10_high == 0 && diag.receiving == 0);
    CHECK(Servo_Stop(1) == SERVO_BUSY && tx_size == 0);
    Servo_ClearRx(); pc10_high = 1;
    CHECK(Servo_StartRxProbe() == SERVO_OK);
    Servo_GetDiagnostics(&diag);
    CHECK(diag.irq_count == 0 && diag.byte_count == 0 && diag.error_count == 0 && diag.error_flags == 0);
    Servo_ClearRx(); check_idle(); /* Quiet receive window cancelled by caller. */
    isr = 1; CHECK(Servo_StartRxProbe() == SERVO_INVALID_ARGUMENT); isr = 0;
    for (uint8_t id = 1; id <= 4; ++id) {
        char expected[24];
        CHECK(Servo_Twist(id, 1500) == SERVO_OK);
        sprintf(expected, "#%03uP1833T1500!", (unsigned)id); expect(expected);
    }
    nested = 1; tc_delay = 3; before = tc_checks;
    CHECK(Servo_Home(1, 1000) == SERVO_OK); expect("#001P0500T1000!"); CHECK(tc_checks - before == 4);
    CHECK(Servo_MoveAngle(2, 180, 9999) == SERVO_OK); expect("#002P1833T9999!");
    CHECK(Servo_MoveAngle(3, 270, 20) == SERVO_OK); expect("#003P2500T0020!");
    CHECK(Servo_SetMode(4, 1) == SERVO_OK); expect("#004PMOD1!");
    CHECK(Servo_SetID(0, 3) == SERVO_OK); expect("#000PID003!");
    CHECK(Servo_Stop(1) == SERVO_OK); expect("#001PDST!");
    CHECK(Servo_ReadPosition(4) == SERVO_OK); expect("#004PRAD!");
    CHECK(Servo_StartRxProbe() == SERVO_BUSY); /* Normal reply reception cannot be interrupted. */
    CHECK(Servo_Twist(0, 1) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_Twist(5, 1) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_MovePulse(1, 499, 10) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_MovePulse(1, 2501, 10) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_Home(1, 10000) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_MoveAngle(1, 271, 1) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_SetMode(1, 5) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_SetID(255, 1) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_SendCommand(NULL) == SERVO_INVALID_ARGUMENT);
    CHECK(Servo_SendCommand("bad") == SERVO_INVALID_ARGUMENT);
    isr = 1; CHECK(Servo_Stop(1) == SERVO_INVALID_ARGUMENT); isr = 0;
    fail_flag = USART_FLAG_TXE; CHECK(Servo_Home(1, 1) == SERVO_TX_TIMEOUT); expect("");
    fail_flag = USART_FLAG_TC; CHECK(Servo_Stop(1) == SERVO_TX_TIMEOUT); expect("#001PDST!"); fail_flag = 0;
    check_idle();
    arm_reply();
    receive("noise#001P1833!");
    check_idle();
    CHECK(Servo_GetFrame(reply, 3, &length) == SERVO_BUFFER_TOO_SMALL && length == 10);
    receive("#OK!"); /* Pending frame must not be overwritten. */
    CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_OK && strcmp(reply, "#001P1833!") == 0);
    CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_NO_FRAME && length == 0);
    arm_reply(); receive("#bad#OK!"); CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_OK && strcmp(reply, "#OK!") == 0);
    arm_reply();
    memset(too_long, 'x', sizeof(too_long)); too_long[0] = '#'; too_long[69] = 0; receive(too_long);
    check_idle(); CHECK(frame_length == 0);
    arm_reply(); receive("#OK!"); CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_OK);
    for (unsigned error = USART_SR_PE; error <= USART_SR_ORE; error <<= 1) {
        arm_reply(); receive("#partial");
        UART4->SR = error | USART_SR_RXNE; UART4->DR = '!'; UART4_IRQHandler();
        CHECK(rx_length == 0 && frame_length == 0); check_idle();
        receive("#OK!"); CHECK(frame_length == 0);
    }
    arm_reply();
    for (unsigned noise = 0; noise < 3U * SERVO_FRAME_CAPACITY; ++noise) receive("x");
    check_idle(); CHECK(frame_length == 0);
    arm_reply();
    for (unsigned noise = 0; noise < 3U * SERVO_FRAME_CAPACITY; ++noise) receive("#");
    check_idle(); CHECK(frame_length == 0);
    arm_reply();
    UART4->SR = USART_SR_RXNE;
    UART4->DR = '#'; UART4_IRQHandler(); UART4->DR = 'O'; UART4_IRQHandler();
    UART4->DR = 'K'; UART4_IRQHandler(); UART4->DR = '!'; UART4_IRQHandler();
    CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_OK && strcmp(reply, "#OK!") == 0);
    check_idle();
    arm_reply(); receive("#partial"); Servo_ClearRx(); check_idle();
    receive("#OK!"); CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_NO_FRAME);
    arm_reply(); receive("#OK!"); Servo_ClearRx(); CHECK(Servo_GetFrame(reply, sizeof(reply), &length) == SERVO_NO_FRAME);
    printf("PASS: servo transport, four IDs, half-turn mapping, TC/timeout, parser/error boundaries (%u checks)\n", checks);
    return 0;
}
