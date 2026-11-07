#include "stm32f4xx.h"
#define ENABLE 1U
#define RCC_AHB1Periph_GPIOC 4U
#define RCC_APB1Periph_TIM6 16U
typedef struct { uint32_t PCLK1_Frequency, HCLK_Frequency; } RCC_ClocksTypeDef;
void RCC_GetClocksFreq(RCC_ClocksTypeDef *clocks);
void RCC_AHB1PeriphClockCmd(uint32_t peripheral, unsigned int enabled);
void RCC_APB1PeriphClockCmd(uint32_t peripheral, unsigned int enabled);
