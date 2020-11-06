#ifndef UART1_DMA_H
#define UART1_DMA_H
#include "hc32_ddl.h"

#define USART1_UNIT   M4_USART1

#define USART1_BAUDRATE      (115200)

#define USART1_RX_PORT       PortA
#define USART1_RX_PIN        Pin02
#define USART1_RX_FUNC       Func_Usart1_Rx

#define USART1_TX_PORT       PortA
#define USART1_TX_PIN        Pin03
#define USART1_TX_FUNC       Func_Usart1_Tx

#define USART1_RI_NUM        INT_USART1_RI
#define USART1_EI_NUM        INT_USART1_EI
#define USART1_TI_NUM        INT_USART1_TI

#define USART1_CLK           PWC_FCG1_PERIPH_USART1

#define USART1_RX_IRQn       Int022_IRQn
#define USART1_ER_IRQn       Int023_IRQn
#define USART1_RTO_IRQn     Int024_IRQn

#define DMA1_CH0_IRQn       Int025_IRQn
void Hw_Uart1_Init(void);
void Test_UART1_TX(void);
void UART1_TX_DMA_Init(void);
void UART1_DMA_TX_Write_Buffer(uint8_t *data, uint8_t len);
#endif

