#include <stdint.h>
#define GPIO_Pin_2 (1U << 2)
#define GPIO_Pin_3 (1U << 3)
#define GPIO_Pin_6 (1U << 6)
#define GPIO_Pin_7 (1U << 7)
#define GPIOA ((void *)1)
#define GPIO_Mode_OUT 1U
#define GPIO_OType_PP 0U
#define GPIO_PuPd_DOWN 2U
#define GPIO_Speed_2MHz 0U
typedef struct { uint16_t GPIO_Pin; uint32_t GPIO_Mode, GPIO_OType, GPIO_PuPd, GPIO_Speed; } GPIO_InitTypeDef;
void GPIO_ResetBits(void *port, uint16_t mask);
void GPIO_SetBits(void *port, uint16_t mask);
void GPIO_StructInit(GPIO_InitTypeDef *gpio);
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio);
