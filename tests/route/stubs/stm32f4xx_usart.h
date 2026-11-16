#ifndef ROUTE_TEST_USART_H
#define ROUTE_TEST_USART_H
#include <stdint.h>
typedef struct { uint32_t SR, DR; } USART_TypeDef;
extern USART_TypeDef route_test_usart1;
extern USART_TypeDef route_test_usart2;
extern USART_TypeDef route_test_usart3;
#define USART1 (&route_test_usart1)
#define USART2 (&route_test_usart2)
#define USART3 (&route_test_usart3)
#define USART_IT_TXE 0x80U
#define USART_IT_RXNE 0x20U
#define ENABLE 1
#define DISABLE 0
#define RESET 0
void USART_ITConfig(USART_TypeDef *uart, uint16_t interrupt, int enabled);
int USART_GetITStatus(USART_TypeDef *uart, uint16_t interrupt);
void USART_SendData(USART_TypeDef *uart, uint16_t byte);
#endif
