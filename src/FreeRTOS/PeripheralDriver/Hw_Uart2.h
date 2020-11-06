#ifndef UART2_DMA_H
#define UART2_DMA_H


#define USART2_UNIT   M4_USART2

#define USART2_BAUDRATE      (115200)

#define USART2_RX_PORT       PortA
#define USART2_RX_PIN        Pin11
#define USART2_RX_FUNC       Func_Usart2_Rx

#define USART2_TX_PORT       PortA
#define USART2_TX_PIN        Pin12
#define USART2_TX_FUNC       Func_Usart2_Tx

#define USART2_RI_NUM        INT_USART2_RI
#define USART2_EI_NUM        INT_USART2_EI
#define USART2_TI_NUM        INT_USART2_TI

#define USART2_CLK           PWC_FCG1_PERIPH_USART2

#define USART2_RX_IRQn       Int025_IRQn
#define USART2_ER_IRQn       Int026_IRQn
#define USART2_RTO_IRQn      Int027_IRQn

void Hw_Uart2_Init(void);
void Test_UART2_TX(void);
#endif

