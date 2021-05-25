/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief This sample demonstrates UART hardware flow control CTS.
 **
 **   - 2018-11-27 CDT First version for Device Driver Library of USART
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* USART channel definition */
#define USART_CH                        (M4_USART2)

/* USART baudrate definition */
#define USART_BAUDRATE                  (3000000ul)

/* USART TX Port/Pin definition */
#define USART_TX_PORT                   (PortA)
#define USART_TX_PIN                    (Pin02)
#define USART_TX_FUNC                   (Func_Usart2_Tx)

/* USART CTS Port/Pin definition */
#define USART_CTS_PORT                  (PortA)
#define USART_CTS_PIN                   (Pin00)
#define USART_CTS_FUNC                  (Func_Usart2_Cts)

/* USART send times */
#define USART_TX_TIMES                  (50u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function of project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint8_t i = 0u;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | \
                             PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput,
        UsartClkDiv_1,
        UsartDataBits8,
        UsartDataLsbFirst,
        UsartOneStopBit,
        UsartParityNone,
        UsartSampleBit8,
        UsartStartBitFallEdge,
        UsartCtsEnable,
    };

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    /* Initialize Key */
    BSP_KEY_Init();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_FUNC, Disable);
    PORT_SetFunc(USART_CTS_PORT, USART_CTS_PIN, USART_CTS_FUNC, Disable);

    /* Initialize USART */
    USART_UART_Init(USART_CH, &stcInitCfg);

    /* Set baudrate */
    USART_SetBaudrate(USART_CH, USART_BAUDRATE);

    /*Enable TX function*/
    USART_FuncCmd(USART_CH, UsartTx, Enable);

    /* User key : SW2 */
    while (Reset == BSP_KEY_GetStatus(BSP_KEY_2))
    {
    }

    for (i = 0u; i < USART_TX_TIMES ; i++)
    {
        while(Reset == USART_GetStatus(USART_CH, UsartTxEmpty))
        {
        }

        USART_SendData(USART_CH, (uint16_t)i);
    }

    BSP_LED_On(LED_GREEN);  /* Send completely */

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
