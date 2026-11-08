#include "stm32f4xx.h"
#define GPIOC ((void *)3)
#define GPIO_Pin_10 (1U << 10)
#define GPIO_PinSource10 10U
#define GPIO_AF_UART4 8U
#define GPIO_Mode_AF 2U
#define GPIO_OType_OD 1U
#define GPIO_PuPd_UP 1U
#define GPIO_Speed_50MHz 2U
typedef struct {uint32_t GPIO_Pin, GPIO_Mode, GPIO_OType, GPIO_PuPd, GPIO_Speed;} GPIO_InitTypeDef;
void GPIO_StructInit(GPIO_InitTypeDef *gpio);
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio);
void GPIO_PinAFConfig(void *port, uint16_t source, uint8_t af);
