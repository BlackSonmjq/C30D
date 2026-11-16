#ifndef TEST_STM32F4XX_H
#define TEST_STM32F4XX_H
#include <stdint.h>
typedef struct { volatile uint32_t CR1, CR2, DIER, SR, PSC, ARR, CNT, EGR; } TIM_TypeDef;
extern TIM_TypeDef test_tim6;
#define TIM6 (&test_tim6)
#define TIM6_DAC_IRQn 54
#define TIM_CR1_CEN 1U
#define TIM_CR1_URS 4U
#define TIM_CR1_OPM 8U
#define TIM_EGR_UG 1U
#define TIM_DIER_UIE 1U
#define TIM_SR_UIF 1U
extern uint32_t test_primask;
#define __get_PRIMASK() test_primask
#define __disable_irq() (test_primask = 1)
#define __set_PRIMASK(value) (test_primask = (value))
void NVIC_ClearPendingIRQ(unsigned int irq);
#endif
