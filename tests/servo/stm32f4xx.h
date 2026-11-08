#ifndef SERVO_TEST_MCU_H
#define SERVO_TEST_MCU_H
#include <stdint.h>
typedef enum {RESET = 0, SET = 1} FlagStatus;
typedef enum {DISABLE = 0, ENABLE = 1} FunctionalState;
typedef struct {volatile uint32_t SR, DR, CR1;} USART_TypeDef;
extern USART_TypeDef mock_uart;
#define UART4 (&mock_uart)
#define UART4_IRQn 52
#define USART_CR1_RE 4U
#define USART_CR1_TE 8U
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_RXNE 32U
uint32_t mock_primask(void);
void mock_disable_irq(void);
void mock_set_primask(uint32_t value);
uint32_t mock_ipsr(void);
#define __get_PRIMASK mock_primask
#define __disable_irq mock_disable_irq
#define __set_PRIMASK mock_set_primask
#define __get_IPSR mock_ipsr
#endif
