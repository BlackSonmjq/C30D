#include "stm32f4xx.h"
#define GPIOC ((void *)3)
#define GPIO_Pin_0 1U
#define GPIO_Pin_1 2U
#define GPIO_Pin_2 4U
#define GPIO_Pin_3 8U
#define GPIO_Mode_OUT 1U
#define GPIO_OType_PP 0U
#define GPIO_Speed_2MHz 0U
#define GPIO_PuPd_DOWN 2U
typedef struct { uint32_t GPIO_Pin, GPIO_Mode, GPIO_OType, GPIO_Speed, GPIO_PuPd; } GPIO_InitTypeDef;
void GPIO_StructInit(GPIO_InitTypeDef *gpio);
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio);
void GPIO_ResetBits(void *port, uint16_t mask);
void GPIO_SetBits(void *port, uint16_t mask);
