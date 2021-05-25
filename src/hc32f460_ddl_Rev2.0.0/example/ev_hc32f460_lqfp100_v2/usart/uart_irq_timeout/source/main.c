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
 ** \brief This sample demonstrates how to check UART data receive by timeout.
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
typedef struct
{
    uint16_t u16Capacity;
    __IO uint16_t u16UsedSize;
    uint16_t u16InIdx;
    uint16_t u16OutIdx;
    uint8_t  au8Buf[500];
} stc_ring_buffer_t;
/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* USART channel definition */
#define USART_CH                        (M4_USART4)

/* USART baudrate definition */
#define USART_BAUDRATE                  (115200ul)

/* USART RX Port/Pin definition */
#define USART_RX_PORT                   (PortB)
#define USART_RX_PIN                    (Pin09)
#define USART_RX_FUNC                   (Func_Usart4_Rx)

/* USART TX Port/Pin definition */
#define USART_TX_PORT                   (PortE)
#define USART_TX_PIN                    (Pin06)
#define USART_TX_FUNC                   (Func_Usart4_Tx)

/* USART interrupt number  */
#define USART_RI_NUM                    (INT_USART4_RI)
#define USART_RI_IRQn                   (Int000_IRQn)
#define USART_EI_NUM                    (INT_USART4_EI)
#define USART_EI_IRQn                   (Int001_IRQn)
#define USART_RTO_NUM                   (INT_USART4_RTO)
#define USART_RTO_IRQn                  (Int002_IRQn)
#define USART_TI_NUM                    (INT_USART4_TI)
#define USART_TI_IRQn                   (Int003_IRQn)
#define USART_TCI_NUM                   (INT_USART4_TCI)
#define USART_TCI_IRQn                  (Int004_IRQn)

/* Timer0 unit definition */
#define TMR_UNIT                        (M4_TMR02)
#define TMR_FCG_PERIPH                  (PWC_FCG2_PERIPH_TIM02)

/* Ring buffer size */
#define IS_RING_BUFFER_EMPTY(x)         (0U == ((x)->u16UsedSize))

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void Timer0Init(void);
static void UsartRxIrqCallback(void);
static void UsartTimeoutIrqCallback(void);
static void UsartTxIrqCallback(void);
static void UsartTxCmpltIrqCallback(void);
static void UsartErrIrqCallback(void);
static en_result_t RingBufWrite(stc_ring_buffer_t *pstcBuffer, uint8_t u8Data);
static en_result_t RingBufRead(stc_ring_buffer_t *pstcBuffer, uint8_t *pu8Data);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_ring_buffer_t m_stcRingBuf = {
    .u16InIdx = 0,
    .u16OutIdx = 0,
    .u16UsedSize = 0,
    .u16Capacity = sizeof (m_stcRingBuf.au8Buf),
};
static uint8_t m_u8Status = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Initliaze Timer0.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Timer0Init(void)
{
    stc_clk_freq_t stcClkTmp;
    stc_tim0_base_init_t stcTimerCfg;
    stc_tim0_trigger_init_t StcTimer0TrigInit;

    MEM_ZERO_STRUCT(stcClkTmp);
    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(StcTimer0TrigInit);

    /* Timer0 peripheral enable */
    PWC_Fcg2PeriphClockCmd(TMR_FCG_PERIPH, Enable);

    /* Clear CNTAR register for channel B */
    TIMER0_WriteCntReg(TMR_UNIT, Tim0_ChannelA, 0u);
    TIMER0_WriteCntReg(TMR_UNIT, Tim0_ChannelB, 0u);

    /* Config register for channel B */
    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv8;
    stcTimerCfg.Tim0_CmpValue = 32000u;
    TIMER0_BaseInit(TMR_UNIT, Tim0_ChannelB, &stcTimerCfg);

    /* Clear compare flag */
    TIMER0_ClearFlag(TMR_UNIT, Tim0_ChannelB);

    /* Config timer0 hardware trigger */
    StcTimer0TrigInit.Tim0_InTrigEnable = false;
    StcTimer0TrigInit.Tim0_InTrigClear = true;
    StcTimer0TrigInit.Tim0_InTrigStart = true;
    StcTimer0TrigInit.Tim0_InTrigStop = false;
    TIMER0_HardTriggerInit(TMR_UNIT, Tim0_ChannelB, &StcTimer0TrigInit);
}

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
    uint16_t u16Data = USART_RecData(USART_CH);

    (void)RingBufWrite(&m_stcRingBuf, (uint8_t)u16Data);
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
 ** \brief USART timeout irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void UsartTimeoutIrqCallback(void)
{
    BSP_LED_On(LED_RED);
    TIMER0_Cmd(TMR_UNIT, Tim0_ChannelB,Disable);
    USART_ClearStatus(USART_CH, UsartRxTimeOut);
}

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
    uint8_t u8Data = 0u;

    if (Ok == RingBufRead(&m_stcRingBuf, &u8Data))
    {
        USART_SendData(USART_CH, (uint16_t)u8Data);
    }
    if (IS_RING_BUFFER_EMPTY(&m_stcRingBuf))
    {
        USART_FuncCmd(USART_CH, UsartTxEmptyInt, Disable);
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
    BSP_LED_Off(LED_RED);
    USART_FuncCmd(USART_CH, UsartTx, Disable);
    USART_FuncCmd(USART_CH, UsartTxCmpltInt, Disable);
    m_u8Status = 0u;
}

/**
 *******************************************************************************
 ** \brief Write ring buffer.
 **
 ** \param [in] pstcBuffer              Pointer to a @ref stc_ring_buffer_t structure
 ** \param [in] u8Data                  Data to write
 **
 ** \retval An en_result_t enumeration value:
 **           - Ok: Write success.
 **           - ErrorBufferEmpty: Buffer is full.
 **
 ******************************************************************************/
static en_result_t RingBufWrite(stc_ring_buffer_t *pstcBuffer, uint8_t u8Data)
{
    en_result_t enRet = Ok;

    if (pstcBuffer->u16UsedSize >= pstcBuffer->u16Capacity)
    {
        enRet = ErrorBufferFull;
    }
    else
    {
        pstcBuffer->au8Buf[pstcBuffer->u16InIdx++] = u8Data;
        pstcBuffer->u16InIdx %= pstcBuffer->u16Capacity;
        pstcBuffer->u16UsedSize++;
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief Read ring buffer.
 **
 ** \param [in] pstcBuffer              Pointer to a @ref stc_ring_buffer_t structure
 ** \param [in] pu8Data                 Pointer to data buffer to read
 **
 ** \retval An en_result_t enumeration value:
 **           - Ok: Read success.
 **           - ErrorBufferEmpty: Buffer is empty.
 **
 ******************************************************************************/
static en_result_t RingBufRead(stc_ring_buffer_t *pstcBuffer, uint8_t *pu8Data)
{
    en_result_t enRet = Ok;

    if (pstcBuffer->u16UsedSize == 0u)
    {
        enRet = Error;
    }
    else
    {
        *pu8Data = pstcBuffer->au8Buf[pstcBuffer->u16OutIdx++];
        pstcBuffer->u16OutIdx %= pstcBuffer->u16Capacity;
        pstcBuffer->u16UsedSize--;
    }

    return enRet;
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
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkOutput,
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

    /* Initialize Timer0 */
    Timer0Init();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);
    PORT_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_FUNC, Disable);

    /* Initialize UART */
    enRet = USART_UART_Init(USART_CH, &stcInitCfg);
    if (enRet != Ok)
    {
        while (1)
        {
        }
    }

    /* Set baudrate */
    enRet = USART_SetBaudrate(USART_CH, USART_BAUDRATE);
    if (enRet != Ok)
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

    /* Set USART RX timeout error IRQ */
    stcIrqRegiCfg.enIRQn = USART_RTO_IRQn;
    stcIrqRegiCfg.pfnCallback = &UsartTimeoutIrqCallback;
    stcIrqRegiCfg.enIntSrc = USART_RTO_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

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

    /*Enable RX && RX interupt && timeout interrupt function*/
    USART_FuncCmd(USART_CH, UsartRx, Enable);
    USART_FuncCmd(USART_CH, UsartRxInt, Enable);
    USART_FuncCmd(USART_CH, UsartTimeOut, Enable);
    USART_FuncCmd(USART_CH, UsartTimeOutInt, Enable);

    while (1)
    {
        if ((!IS_RING_BUFFER_EMPTY(&m_stcRingBuf)) && (0u == m_u8Status))
        {
            USART_FuncCmd(USART_CH, UsartTxAndTxEmptyInt, Enable);
            m_u8Status = 1u;
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
