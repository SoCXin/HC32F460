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
 ** \brief This sample demonstrates clock sync data receive by interrupt.
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
    uint8_t u8Cnt;
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

/* USART RX Port/Pin definition */
#define USART_RX_PORT                   (PortA)
#define USART_RX_PIN                    (Pin03)
#define USART_RX_FUNC                   (Func_Usart2_Rx)

/* USART CK Port/Pin definition */
#define USART_CK_PORT                   (PortD)
#define USART_CK_PIN                    (Pin07)
#define USART_CK_FUNC                   (Func_Usart_Ck)

/* USART interrupt number  */
#define USART_RI_NUM                    (INT_USART2_RI)
#define USART_RI_IRQn                   (Int000_IRQn)
#define USART_EI_NUM                    (INT_USART2_EI)
#define USART_EI_IRQn                   (Int001_IRQn)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void UsartRxIrqCallback(void);
static void UsartErrIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static en_result_t m_enTestResult = Ok;

static stc_buf_handle_t m_stcRxBufHanlde;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief USART RX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartRxIrqCallback(void)
{
    if (m_stcRxBufHanlde.u8Cnt < m_stcRxBufHanlde.u8Size)
    {
        m_stcRxBufHanlde.au8Buf[m_stcRxBufHanlde.u8Cnt] = (uint8_t)USART_RecData(USART_CH);

        if (m_stcRxBufHanlde.u8Cnt != m_stcRxBufHanlde.au8Buf[m_stcRxBufHanlde.u8Cnt])
        {
            m_enTestResult = Error;
        }
        else
        {
        }

        m_stcRxBufHanlde.u8Cnt++;
    }
    else
    {
        m_enTestResult = Error;
    }
}

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
    m_enTestResult = Error;

    if (Set == USART_GetStatus(USART_CH, UsartFrameErr))
    {
        USART_ClearStatus(USART_CH, UsartFrameErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(USART_CH, UsartParityErr))
    {
        USART_ClearStatus(USART_CH, UsartParityErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(USART_CH, UsartOverrunErr))
    {
        USART_ClearStatus(USART_CH, UsartOverrunErr);
    }
    else
    {
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
    stc_irq_regi_conf_t stcIrqRegiCfg;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | \
                             PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;
    const stc_usart_clksync_init_t stcInitCfg = {
        UsartExtClk,
        UsartClkDiv_1,
        UsartDataLsbFirst,
        UsartRtsEnable,
    };

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_CK_PORT, USART_CK_PIN, USART_CK_FUNC, Disable);
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);

    /* Initialize buffer */
    m_stcRxBufHanlde.u8Size = (uint8_t)sizeof(m_stcRxBufHanlde.au8Buf);

    /* Initialize USART */
    if (Ok != USART_CLKSYNC_Init(USART_CH, &stcInitCfg))
    {
        while (1)
        {
        }
    }

    /* Set baudrate */
    if (Ok != USART_SetBaudrate(USART_CH, USART_BAUDRATE))
    {
        while (1)
        {
        }
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = USART_RI_IRQn;
    stcIrqRegiCfg.pfnCallback = &UsartRxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

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
    USART_FuncCmd(USART_CH, UsartRxInt, Enable);

    while (m_stcRxBufHanlde.u8Size != m_stcRxBufHanlde.u8Cnt)
    {
    }

    if (Ok == m_enTestResult)
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
