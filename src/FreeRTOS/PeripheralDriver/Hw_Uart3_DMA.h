#ifndef UART_DMA_H
#define UART_DMA_H


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

#define USART3_CLK           PWC_FCG1_PERIPH_USART3

//#define USART3_RX_IRQn       Int022_IRQn
#define USART3_ER_IRQn       Int023_IRQn
#define USART3_RTO_IRQn     Int024_IRQn
//#define TIMER02_CHA_IRQn    Int025_IRQn

#define UART3_DMA2_UNIT               (M4_DMA2)
#define UART3_DMA_RXCH                  (DmaCh0)
#define UART3_DMA_CLK                 PWC_FCG0_PERIPH_DMA2
#define UART3_DMA_TRNCNT              (40u)//传输次数，最大包长
#define UART3_DMA_BLKSIZE                   (1u)
#define UART3_DMA_RPT_SIZE            UART3_DMA_TRNCNT
#define UART3_DMA_INT_SRC                   INT_DMA2_BTC0
#define UART3_DMA_Trg_Src                   EVT_USART3_RI
//#define DMA2_CH0_IRQn           Int009_IRQn



void hw_rxdma_init(void);
void hw_uart3Init(void);
void UART_RTO_Timer_Init(void);
void Test_UART_TX(void);

#endif

