#define RCC_AHB1Periph_GPIOC 4U
#define RCC_AHB1Periph_DMA1 0x200000U
#define RCC_APB1Periph_UART4 0x80000U
void RCC_AHB1PeriphClockCmd(uint32_t peripheral, FunctionalState enabled);
void RCC_APB1PeriphClockCmd(uint32_t peripheral, FunctionalState enabled);
