#ifndef UART3_DMA_H
#define UART3_DMA_H


#define USART3_UNIT   M4_USART3

#define USART3_BAUDRATE      (115200)

#define USART3_RX_PORT       PortB
#define USART3_RX_PIN        Pin12
#define USART3_RX_FUNC       Func_Usart3_Rx

#define USART3_TX_PORT       PortB
#define USART3_TX_PIN        Pin13
#define USART3_TX_FUNC       Func_Usart3_Tx

#define USART3_RI_NUM        INT_USART3_RI
#define USART3_EI_NUM        INT_USART3_EI
#define USART3_TI_NUM        INT_USART3_TI

#define USART3_CLK           PWC_FCG1_PERIPH_USART3

#define USART3_RX_IRQn       Int022_IRQn
#define USART3_ER_IRQn       Int023_IRQn
#define USART3_RTO_IRQn     Int024_IRQn

void Hw_Uart3_Init(void);
void Test_UART3_TX(void);
#endif

