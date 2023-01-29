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
 ** \brief This sample demonstrates clock sync hardware flow control CTS.
 **
 **   - 2021-04-16 CDT First version for Device Driver Library of USART
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief buffer handle
 **
 ******************************************************************************/
typedef struct stc_buf_handle
{
    uint8_t u8Size;
    uint8_t au8Buf[200];
} stc_buf_handle_t;

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

/* USART CK Port/Pin definition */
#define USART_CK_PORT                   (PortD)
#define USART_CK_PIN                    (Pin07)
#define USART_CK_FUNC                   (Func_Usart_Ck)

/* USART CTS Port/Pin definition */
#define USART_CTS_PORT                  (PortA)
#define USART_CTS_PIN                   (Pin00)
#define USART_CTS_FUNC                  (Func_Usart2_Cts)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_buf_handle_t m_stcTxBufHanlde;

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
    const stc_usart_clksync_init_t stcInitCfg = {
        UsartIntClkCkOutput,
        UsartClkDiv_1,
        UsartDataLsbFirst,
        UsartCtsEnable,
    };

    /* Initialize buffer */
    m_stcTxBufHanlde.u8Size = (uint8_t)sizeof(m_stcTxBufHanlde.au8Buf);

    for (i = 0u; i < m_stcTxBufHanlde.u8Size ; i++)
    {
        m_stcTxBufHanlde.au8Buf[i] = i;
    }

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    /* Initialize KEY */
    BSP_KEY_Init();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_CK_PORT, USART_CK_PIN, USART_CK_FUNC, Disable);
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_FUNC, Disable);
    PORT_SetFunc(USART_CTS_PORT, USART_CTS_PIN, USART_CTS_FUNC, Disable);

    /* Initialize USART */
    if (Ok != USART_CLKSYNC_Init(USART_CH, &stcInitCfg))
    {
        while (1)
        {
        }
    }
    else
    {
    }

    /* Set baudrate */
    if (Ok != USART_SetBaudrate(USART_CH, USART_BAUDRATE))
    {
        while (1)
        {
        }
    }
    else
    {
    }

    BSP_LED_On(LED_GREEN);  /* Configure completely */

    /* User key : K1 */
    while (Reset == BSP_KEY_GetStatus(BSP_KEY_1))
    {
    }

    BSP_LED_Off(LED_GREEN);

    /*Enable TX function*/
    USART_FuncCmd(USART_CH, UsartTx, Enable);

    for (i = 0u; i < m_stcTxBufHanlde.u8Size ; i++)
    {
        while (Reset == USART_GetStatus(USART_CH, UsartTxEmpty))
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
