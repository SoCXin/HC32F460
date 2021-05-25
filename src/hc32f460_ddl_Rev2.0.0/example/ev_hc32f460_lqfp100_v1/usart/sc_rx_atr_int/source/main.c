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
 ** \brief This sample demonstrates UART receive Smart-card ATR by interrupt.
 **
 **   - 2018-12-06 CDT First version for Device Driver Library of USART
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

/* Smart-card USART channel definition */
#define SC_USART_CH                     (M4_USART2)

/* Smart-card USART baudrate definition */
#define SC_USART_BAUDRATE               (9600ul)

/* Smart-card USART RX Port/Pin definition */
#define SC_RX_PORT                      (PortA)
#define SC_RX_PIN                       (Pin03)
#define SC_RX_FUNC                      (Func_Usart2_Rx)

/* Smart-card USART TX Port/Pin definition */
#define SC_TX_PORT                      (PortA)
#define SC_TX_PIN                       (Pin02)
#define SC_TX_FUNC                      (Func_Usart2_Tx)

/* Smart-card USART CK Port/Pin definition */
#define SC_CK_PORT                      (PortD)
#define SC_CK_PIN                       (Pin07)
#define SC_CK_FUNC                      (Func_Usart_Ck)

/* Smart-card CD Port/Pin definition */
#define SC_CD_PORT                      (PortB)
#define SC_CD_PIN                       (Pin01)

/* Smart-card RST Port/Pin definition */
#define SC_RST_PORT                     (PortA)
#define SC_RST_PIN                      (Pin00)

/* Smart-card power on Port/Pin definition */
#define SC_PWREN_PORT                   (PortA)
#define SC_PWREN_PIN                    (Pin01)

/* Smart-card CD pin operation */
#define IS_CARD_INSERTED()              (Reset == PORT_GetBit(SC_CD_PORT, SC_CD_PIN))

/* Smart-card reset pin operation */
#define SC_RESET_LOW()                  (PORT_ResetBits(SC_RST_PORT, SC_RST_PIN))
#define SC_RESET_HIGH()                 (PORT_SetBits(SC_RST_PORT, SC_RST_PIN))

/* Smart-card power pin operation */
#define SC_POWER_ON()                   (PORT_ResetBits(SC_PWREN_PORT, SC_PWREN_PIN))
#define SC_POWER_OFF()                  (PORT_SetBits(SC_PWREN_PORT, SC_PWREN_PIN))

/* Smart-card USART interrupt number  */
#define SC_USART_RI_NUM                 (INT_USART2_RI)
#define SC_USART_RI_IRQn                (Int001_IRQn)
#define SC_USART_EI_NUM                 (INT_USART2_EI)
#define SC_USART_EI_IRQn                (Int002_IRQn)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ScPinInit(void);
static void ScUsartRxIrqCallback(void);
static void ScUsartErrIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_buf_handle_t m_stcRxBufHanlde;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Initialize card-detect pin.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScCdPinInit(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    /* CD Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(SC_CD_PORT, SC_CD_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Initialize smart card interface pin.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScPinInit(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    /* RX/TX/CK Pin initialization */
    PORT_SetFunc(SC_RX_PORT, SC_RX_PIN, SC_RX_FUNC, Disable);
    PORT_SetFunc(SC_TX_PORT, SC_TX_PIN, SC_TX_FUNC, Disable);
    PORT_SetFunc(SC_CK_PORT, SC_CK_PIN, SC_CK_FUNC, Disable);

    /* Set RST pin output High-level */
    PORT_SetBits(SC_RST_PORT, SC_RST_PIN);
    /* RST Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(SC_RST_PORT, SC_RST_PIN, &stcPortInit);

    /* Set Power on pin output High-level: Power off */
    PORT_SetBits(SC_PWREN_PORT, SC_PWREN_PIN);
    /* Power on Pin initialization */
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(SC_PWREN_PORT, SC_PWREN_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Smart-card USART RX irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScUsartRxIrqCallback(void)
{
    if (m_stcRxBufHanlde.u8Cnt < m_stcRxBufHanlde.u8Size)
    {
        m_stcRxBufHanlde.au8Buf[m_stcRxBufHanlde.u8Cnt++] = (uint8_t)USART_RecData(SC_USART_CH);;
    }
    else
    {
    }
}

/**
 *******************************************************************************
 ** \brief Smart-card USART RX error irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ScUsartErrIrqCallback(void)
{
    if (Set == USART_GetStatus(SC_USART_CH, UsartFrameErr))
    {
        USART_ClearStatus(SC_USART_CH, UsartFrameErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(SC_USART_CH, UsartParityErr))
    {
        USART_ClearStatus(SC_USART_CH, UsartParityErr);
    }
    else
    {
    }

    if (Set == USART_GetStatus(SC_USART_CH, UsartOverrunErr))
    {
        USART_ClearStatus(SC_USART_CH, UsartOverrunErr);
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
    en_result_t enRet = Ok;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | \
                             PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;
    const stc_usart_sc_init_t stcScInitCfg = {
        UsartIntClkCkOutput,
        UsartClkDiv_1,
        UsartDataLsbFirst,
    };

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize buffer */
    m_stcRxBufHanlde.u8Size = (uint8_t)sizeof(m_stcRxBufHanlde.au8Buf);

    /* Initialize card-detect IO */
    ScCdPinInit();

    while (!IS_CARD_INSERTED())
    {
    }

    /* Initialize smart card IO */
    ScPinInit();
    SC_POWER_OFF();
    SC_RESET_LOW();
    USART_DeInit(SC_USART_CH);

    /* Initialize UART */
    enRet = USART_SC_Init(SC_USART_CH, &stcScInitCfg);
    if(enRet != Ok)
    {
        while (1)
        {
        }
    }

    /* Set baudrate */
    enRet = USART_SetBaudrate(SC_USART_CH, SC_USART_BAUDRATE);
    if(enRet != Ok)
    {
        while (1)
        {
        }
    }

    /* Set USART RX IRQ */
    stcIrqRegiCfg.enIRQn = SC_USART_RI_IRQn;
    stcIrqRegiCfg.pfnCallback = &ScUsartRxIrqCallback;
    stcIrqRegiCfg.enIntSrc = SC_USART_RI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Set USART RX error IRQ */
    stcIrqRegiCfg.enIRQn = SC_USART_EI_IRQn;
    stcIrqRegiCfg.pfnCallback = &ScUsartErrIrqCallback;
    stcIrqRegiCfg.enIntSrc = SC_USART_EI_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /*Enable RX && RX interupt function*/
    USART_FuncCmd(SC_USART_CH, UsartRxInt, Enable);
    USART_FuncCmd(SC_USART_CH, UsartRx, Enable);

    /* Cold reset :smart card */
    SC_POWER_ON();
    Ddl_Delay1ms(1ul);
    SC_RESET_HIGH();
    Ddl_Delay1ms(150ul);  /* Delay for receving Smart-card ATR */

    /* Printf ATR */
    for (uint8_t i = 0u; i < m_stcRxBufHanlde.u8Cnt; i++)
    {
        DDL_Printf("%x ", m_stcRxBufHanlde.au8Buf[i]);
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
