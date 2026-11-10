#define USART_Mode_Rx USART_CR1_RE
#define USART_IT_RXNE 32U
#define USART_FLAG_TXE 128U
#define USART_FLAG_TC 64U
typedef struct {uint32_t USART_BaudRate, USART_Mode;} USART_InitTypeDef;
void USART_StructInit(USART_InitTypeDef *uart);
void USART_Init(USART_TypeDef *port, USART_InitTypeDef *uart);
void USART_DeInit(USART_TypeDef *port);
void USART_Cmd(USART_TypeDef *port, FunctionalState enabled);
void USART_HalfDuplexCmd(USART_TypeDef *port, FunctionalState enabled);
void USART_ITConfig(USART_TypeDef *port, uint16_t interrupt, FunctionalState enabled);
FlagStatus USART_GetFlagStatus(USART_TypeDef *port, uint16_t flag);
void USART_SendData(USART_TypeDef *port, uint16_t data);
