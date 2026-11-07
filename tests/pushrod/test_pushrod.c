/* Real pushrod driver with mocked MMIO; no task is scheduled for timer expiry. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../R550_C30D_2.0_Mini_STM32_GMR_Encoder_2025.12.26/HARDWARE/pushrod.c"

TIM_TypeDef test_tim6;
uint32_t test_primask;
static uint16_t levels;
static unsigned int checks, clock_mode, pending_clears;
#define CHECK(expr) do { ++checks; if(!(expr)) { printf("FAIL line %u: %s\n", __LINE__, #expr); exit(1); } } while(0)
void NVIC_ClearPendingIRQ(unsigned int irq) {
    CHECK(irq == TIM6_DAC_IRQn); ++pending_clears;
}
void NVIC_Init(NVIC_InitTypeDef *irq) {
    CHECK(irq->NVIC_IRQChannel == TIM6_DAC_IRQn);
    CHECK(irq->NVIC_IRQChannelPreemptionPriority == 0);
    CHECK(irq->NVIC_IRQChannelCmd == ENABLE);
}
void RCC_GetClocksFreq(RCC_ClocksTypeDef *clocks) {
    clocks->HCLK_Frequency = clock_mode ? 42000000 : 168000000;
    clocks->PCLK1_Frequency = 42000000;
}
void RCC_AHB1PeriphClockCmd(uint32_t peripheral, unsigned int enabled) {
    CHECK(peripheral == RCC_AHB1Periph_GPIOC && enabled == ENABLE);
}
void RCC_APB1PeriphClockCmd(uint32_t peripheral, unsigned int enabled) {
    CHECK(peripheral == RCC_APB1Periph_TIM6 && enabled == ENABLE);
}
void GPIO_StructInit(GPIO_InitTypeDef *gpio) { memset(gpio, 0, sizeof(*gpio)); }
void GPIO_Init(void *port, GPIO_InitTypeDef *gpio) {
    CHECK(port == GPIOC && gpio->GPIO_Pin == 15 && gpio->GPIO_Mode == GPIO_Mode_OUT);
    CHECK(!levels);
}
void GPIO_ResetBits(void *port, uint16_t mask) {
    CHECK(port == GPIOC); levels &= ~mask;
}
void GPIO_SetBits(void *port, uint16_t mask) {
    CHECK(port == GPIOC && test_primask == 1);
    CHECK(!levels && !(TIM6->CR1 & TIM_CR1_CEN));
    levels |= mask;
}
static void expire(void) {
    /* Peripheral one-pulse mode clears CEN at the update; IRQ cuts GPIO. */
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->SR |= TIM_SR_UIF;
    TIM6_DAC_IRQHandler();
}
int main(void) {
    unsigned int id;
    CHECK(Pushrod_StartTimed(1, 5000) == PUSHROD_NOT_INITIALIZED);
    Pushrod_Init();
    CHECK(TIM6->PSC == 8399 && (TIM6->CR1 & TIM_CR1_OPM));
    CHECK(!levels && !test_primask);
    CHECK(Pushrod_StartTimed(0, 5000) == PUSHROD_INVALID_ARGUMENT);
    CHECK(Pushrod_StartTimed(5, 5000) == PUSHROD_INVALID_ARGUMENT);
    CHECK(Pushrod_StartTimed(1, 0) == PUSHROD_INVALID_ARGUMENT);
    CHECK(Pushrod_StartTimed(1, 6554) == PUSHROD_INVALID_ARGUMENT);
    for(id = 1; id <= 4; ++id) {
        CHECK(Pushrod_StartTimed(id, 5000) == PUSHROD_OK);
        CHECK(levels == (1U << (id - 1)) && !Pushrod_TimedDone(id));
        CHECK(TIM6->ARR == 49999 && TIM6->CNT == 0);
        CHECK((TIM6->ARR + 1) * (TIM6->PSC + 1) == 420000000);
        CHECK((TIM6->CR1 & TIM_CR1_CEN) && TIM6->DIER == TIM_DIER_UIE);
        CHECK(Pushrod_StartTimed(id, 5000) == PUSHROD_BUSY);
        CHECK(Pushrod_Start(id) == PUSHROD_BUSY);
        TIM6_DAC_IRQHandler(); /* No update: must not stop early. */
        CHECK(levels && !Pushrod_TimedDone(id));
        expire(); /* No task poll, no FreeRTOS tick, yet output must go low. */
        CHECK(!levels && Pushrod_TimedDone(id));
        CHECK(!Pushrod_TimedDone(0) && !Pushrod_TimedDone(5));
        CHECK(!(TIM6->CR1 & TIM_CR1_CEN) && !TIM6->DIER && !TIM6->SR);
        CHECK(Pushrod_Stop(id) == PUSHROD_OK && !Pushrod_TimedDone(id));
    }
    CHECK(Pushrod_StartTimed(1, 5000) == PUSHROD_OK);
    TIM6->SR = TIM_SR_UIF; /* Cancel with an IRQ pending. */
    CHECK(Pushrod_Stop(1) == PUSHROD_OK);
    TIM6_DAC_IRQHandler();
    CHECK(!levels && !Pushrod_TimedDone(1) && !TIM6->DIER);
    CHECK(Pushrod_StartTimed(4, 5000) == PUSHROD_OK);
    TIM6_DAC_IRQHandler(); /* Old canceled event must not end the new run. */
    CHECK(levels == 8 && !Pushrod_TimedDone(4));
    CHECK(Pushrod_Stop(2) == PUSHROD_OK && (TIM6->CR1 & TIM_CR1_CEN));
    expire();
    CHECK(!levels && Pushrod_TimedDone(4));
    /* Reinitialize with APB1 undivided; do not incorrectly double its clock. */
    clock_mode = 1;
    Pushrod_Init();
    CHECK(TIM6->PSC == 4199 && !Pushrod_TimedDone(4));
    test_primask = 1;
    CHECK(Pushrod_StartTimed(2, 1) == PUSHROD_OK && test_primask == 1);
    CHECK(TIM6->ARR == 9);
    CHECK(Pushrod_Stop(2) == PUSHROD_OK && test_primask == 1);
    CHECK(pending_clears > 10);
    printf("PASS: pushrod hardware cutoff, cancel/restart and clock setup; %u checks\n", checks);
    return 0;
}
