#ifndef SERVO_TEST_MCU_H
#define SERVO_TEST_MCU_H
#include <stdint.h>
typedef enum {RESET = 0, SET = 1} FlagStatus;
typedef enum {DISABLE = 0, ENABLE = 1} FunctionalState;
typedef struct {volatile uint32_t SR, DR, CR1, CR3;} USART_TypeDef;
typedef struct {volatile uint32_t CR, NDTR, PAR, M0AR, FCR;} DMA_Stream_TypeDef;
typedef struct {volatile uint32_t HISR, HIFCR;} DMA_TypeDef;
extern DMA_TypeDef mock_dma;
DMA_Stream_TypeDef *mock_dma_stream(void);
#define DMA1 (&mock_dma)
#define DMA1_Stream4 mock_dma_stream()
#define DMA_SxCR_EN 1U
#define DMA_SxCR_CHSEL_2 0x08000000U
#define DMA_SxCR_MINC 0x400U
#define DMA_SxCR_DIR_0 0x40U
#define DMA_SxCR_PL_1 0x20000U
#define DMA_HISR_TCIF4 32U
#define DMA_HISR_TEIF4 8U
#define DMA_HISR_DMEIF4 4U
#define DMA_HISR_FEIF4 1U
#define DMA_HIFCR_CTCIF4 32U
#define DMA_HIFCR_CHTIF4 16U
#define DMA_HIFCR_CTEIF4 8U
#define DMA_HIFCR_CDMEIF4 4U
#define DMA_HIFCR_CFEIF4 1U
#define USART_CR3_DMAT 128U
#define __DMB() ((void)0)
extern USART_TypeDef mock_uart;
#define UART4 (&mock_uart)
#define UART4_IRQn 52
#define USART_CR1_RE 4U
#define USART_CR1_TE 8U
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_RXNE 32U
uint32_t mock_primask(void);
void mock_disable_irq(void);
void mock_set_primask(uint32_t value);
uint32_t mock_ipsr(void);
#define __get_PRIMASK mock_primask
#define __disable_irq mock_disable_irq
#define __set_PRIMASK mock_set_primask
#define __get_IPSR mock_ipsr
#endif
