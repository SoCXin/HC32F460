#ifndef HW_DMA_TX_UART3_H
#define HW_DMA_TX_UART3_H
#include "hc32_ddl.h"

#define USART3_UNIT   M4_USART3

#define USART3_BAUDRATE      (115200)

#define USART3_RX_PORT       PortE
#define USART3_RX_PIN        Pin04
#define USART3_RX_FUNC       Func_Usart3_Rx

#define USART3_TX_PORT       PortE
#define USART3_TX_PIN        Pin05
#define USART3_TX_FUNC       Func_Usart3_Tx

#define USART3_RI_NUM        INT_USART3_RI
#define USART3_EI_NUM        INT_USART3_EI
#define USART3_TI_NUM        INT_USART3_TI
#define USART3_TCI_NUM		 INT_USART3_TCI

#define USART3_CLK           PWC_FCG1_PERIPH_USART3

#define USART3_RX_IRQn       Int022_IRQn
#define USART3_ER_IRQn       Int023_IRQn
#define USART3_RTO_IRQn     Int024_IRQn

#define DMA1_CH0_IRQn       Int025_IRQn
#define DMA1_CH0_TRGSRC		EVT_USART3_TI


void Hw_Uart3_Init(void);
void Test_UART3_TX(void);
void UART3_DMA_TX_Write_Buffer(uint8_t *data, uint8_t len);
void UART_INT_Write_Buffer(uint8_t *data, uint8_t len);
#endif

