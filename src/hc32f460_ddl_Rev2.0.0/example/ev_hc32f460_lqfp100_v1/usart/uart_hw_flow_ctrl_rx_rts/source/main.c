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
 ** \brief This sample demonstrates UART hardware flow control RTS.
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

/* USART RX Port/Pin definition */
#define USART_RX_PORT                   (PortA)
#define USART_RX_PIN                    (Pin03)
#define USART_RX_FUNC                   (Func_Usart2_Rx)

/* USART RTS Port/Pin definition */
#define USART_RTS_PORT                  (PortA)
#define USART_RTS_PIN                   (Pin01)
#define USART_RTS_FUNC                  (Func_Usart2_Rts)

/* USART interrupt  */
#define USART_EI_NUM                    (INT_USART2_EI)
#define USART_EI_IRQn                   (Int001_IRQn)

/* USART delay time for reading */
#define USART_DELAY                     (10u)

/* USART send times */
#define USART_RX_TIMES                  (50u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void UsartErrIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief USART RX error irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartErrIrqCallback(void)
{
    if (Set == USART_GetStatus(USART_CH, UsartFrameErr))
    {
        USART_ClearStatus(USART_CH, UsartFrameErr);
    }

    if (Set == USART_GetStatus(USART_CH, UsartParityErr))
    {
        USART_ClearStatus(USART_CH, UsartParityErr);
    }

    if (Set == USART_GetStatus(USART_CH, UsartOverrunErr))
    {
        USART_ClearStatus(USART_CH, UsartOverrunErr);
    }
}

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
    uint8_t u8RecData = 0u;
    en_result_t enTestResult = Ok;
    stc_irq_regi_conf_t stcIrqRegiCfg;
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
        UsartRtsEnable,
    };

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);
    PORT_SetFunc(USART_RTS_PORT, USART_RTS_PIN, USART_RTS_FUNC, Disable);

    /* Initialize USART */
    USART_UART_Init(USART_CH, &stcInitCfg);

    /* Set baudrate */
    USART_SetBaudrate(USART_CH, USART_BAUDRATE);

    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = USART_EI_IRQn;
    stcIrqRegiCfg.pfnCallback = &UsartErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /*Enable RX && RX interupt function*/
    USART_FuncCmd(USART_CH, UsartRx, Enable);

    while (i < USART_RX_TIMES)
    {
        if (Set == USART_GetStatus(USART_CH, UsartRxNoEmpty))
        {
            Ddl_Delay1ms(USART_DELAY);
            u8RecData = (uint8_t)USART_RecData(USART_CH);

            if (u8RecData != i)
            {
                enTestResult = Error;
                break;
            }

            ++i;
        }
    }

    if (Ok == enTestResult)
    {
        BSP_LED_On(LED_GREEN);  /* Test pass && meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);    /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
