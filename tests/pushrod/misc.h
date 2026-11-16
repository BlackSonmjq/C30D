#include "stm32f4xx.h"
typedef struct {
    unsigned int NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    unsigned int NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
void NVIC_Init(NVIC_InitTypeDef *irq);
