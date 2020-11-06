#ifndef USER_UART_H
#define USER_UART_H
#include "hc32_ddl.h"
#define USART_CH   M4_USART4

#define USART_BAUDRATE      (115200)

#define USART_RX_PORT       PortC
#define USART_RX_PIN        Pin13
#define USART_RX_FUNC       Func_Usart4_Rx

#define USART_TX_PORT       PortH
#define USART_TX_PIN        Pin02
#define USART_TX_FUNC       Func_Usart4_Tx

#define USART_RI_NUM        INT_USART4_RI
#define USART_EI_NUM        INT_USART4_EI
#define USART_TI_NUM        INT_USART4_TI

#define USART_CLK           PWC_FCG1_PERIPH_USART4

#define USART_RX_IRQn       Int022_IRQn
#define USART_ER_IRQn       Int023_IRQn


extern volatile uint8_t Uartdata[4000];
extern volatile bool flag_RX2_receviced;

void User_USART_Init(void);
void Test_UART_TX(void);
void User_USART2_Init(void);
en_result_t Get_UART2_data(uint8_t *RX2_data, uint32_t *length);
void test_Usart2(void);

#endif
