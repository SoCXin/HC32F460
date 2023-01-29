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
 ** \brief This sample demonstrates clock sync data send by interrupt.
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

/* USART TX Port/Pin definition */
#define USART_TX_PORT                   (PortA)
#define USART_TX_PIN                    (Pin02)
#define USART_TX_FUNC                   (Func_Usart2_Tx)

/* USART CK Port/Pin definition */
#define USART_CK_PORT                   (PortD)
#define USART_CK_PIN                    (Pin07)
#define USART_CK_FUNC                   (Func_Usart_Ck)

/* USART interrupt number  */
#define USART_TI_NUM                    (INT_USART2_TI)
#define USART_TI_IRQn                   (Int000_IRQn)
#define USART_TCI_NUM                   (INT_USART2_TCI)
#define USART_TCI_IRQn                  (Int001_IRQn)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void UsartTxIrqCallback(void);
static void UsartTxCmpltIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_buf_handle_t m_stcTxBufHanlde;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief USART TX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTxIrqCallback(void)
{
    if (m_stcTxBufHanlde.u8Cnt < (m_stcTxBufHanlde.u8Size - 1u))
    {
        USART_SendData(USART_CH, (uint16_t)(m_stcTxBufHanlde.au8Buf[m_stcTxBufHanlde.u8Cnt++]));
    }
    else
    {
        USART_FuncCmd(USART_CH, UsartTxEmptyInt, Disable);
        USART_SendData(USART_CH, (uint16_t)(m_stcTxBufHanlde.au8Buf[m_stcTxBufHanlde.u8Cnt++]));
        USART_FuncCmd(USART_CH, UsartTxCmpltInt, Enable);
    }
}

/**
 *******************************************************************************
 ** \brief USART TX complete irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTxCmpltIrqCallback(void)
{
    USART_FuncCmd(USART_CH, UsartTxCmpltInt, Disable);
    USART_FuncCmd(USART_CH, UsartTx, Disable);
    BSP_LED_On(LED_GREEN);  /* Send completely */
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
    uint8_t i;
    en_result_t enRet = Ok;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | \
                             PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;
    const stc_usart_clksync_init_t stcInitCfg = {
        UsartIntClkCkOutput,
        UsartClkDiv_1,
        UsartDataLsbFirst,
        UsartRtsEnable,
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

    /* Initialize Clock sync */
    enRet = USART_CLKSYNC_Init(USART_CH, &stcInitCfg);
    if (enRet != Ok)
    {
        while (1)
        {
        }
    }

    /* Set baudrate */
    enRet = USART_SetBaudrate(USART_CH, USART_BAUDRATE);
    if(enRet != Ok)
    {
        while (1)
        {
        }
    }

    /* Set USART TX IRQ */
    stcIrqRegiCfg.enIRQn = USART_TI_IRQn;
    stcIrqRegiCfg.pfnCallback = &UsartTxIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_TI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART TX complete IRQ */
    stcIrqRegiCfg.enIRQn = USART_TCI_IRQn;
    stcIrqRegiCfg.pfnCallback = &UsartTxCmpltIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_TCI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* User key : K1 */
    while (Reset == BSP_KEY_GetStatus(BSP_KEY_1))
    {
    }

    /*Enable TX && TX interupt function*/
    USART_FuncCmd(USART_CH, UsartTxAndTxEmptyInt, Enable);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
