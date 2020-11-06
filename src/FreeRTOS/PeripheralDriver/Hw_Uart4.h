#ifndef UART4_DMA_H
#define UART4_DMA_H


#define USART4_UNIT   M4_USART4

#define USART4_BAUDRATE      (115200)

#define USART4_RX_PORT       PortC//PortB
#define USART4_RX_PIN        Pin13//Pin14
#define USART4_RX_FUNC       Func_Usart4_Rx

#define USART4_TX_PORT       PortH//PortB
#define USART4_TX_PIN        Pin02//Pin15
#define USART4_TX_FUNC       Func_Usart4_Tx

#define USART4_RI_NUM        INT_USART4_RI
#define USART4_EI_NUM        INT_USART4_EI
#define USART4_TI_NUM        INT_USART4_TI

#define USART4_CLK           PWC_FCG1_PERIPH_USART4

#define USART4_RX_IRQn       Int025_IRQn
#define USART4_ER_IRQn       Int026_IRQn
#define USART4_RTO_IRQn      Int027_IRQn

void Hw_Uart4_Init(void);
void Test_UART4_TX(void);
#endif

