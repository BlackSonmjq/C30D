typedef struct {uint32_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority, NVIC_IRQChannelSubPriority; FunctionalState NVIC_IRQChannelCmd;} NVIC_InitTypeDef;
void NVIC_Init(NVIC_InitTypeDef *irq);
void NVIC_ClearPendingIRQ(int irq);
