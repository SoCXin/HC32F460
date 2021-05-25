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
 ** \brief This sample demonstrates clock sync data receive and transfer by DMA.
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
/* DMAC */
#define DMA_UNIT                        (M4_DMA1)
#define DMA_CH                          (DmaCh0)
#define DMA_TRG_SEL                     (EVT_USART2_RI)

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
#define USART_EI_NUM                    (INT_USART2_EI)
#define USART_EI_IRQn                   (Int000_IRQn)

/* DMA block transfer complete interrupt */
#define DMA_BTC_INT_NUM                 (INT_DMA1_BTC0)
#define DMA_BTC_INT_IRQn                (Int002_IRQn)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void DmaInit(void);
static void DmaBtcIrqCallback(void);
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
 ** \brief Initialize DMA.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void DmaInit(void)
{
    stc_dma_config_t stcDmaInit;
    stc_irq_regi_conf_t stcIrqRegiCfg;

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1 | PWC_FCG0_PERIPH_DMA2,Enable);

    /* Enable DMA. */
    DMA_Cmd(DMA_UNIT,Enable);

    /* Initialize DMA. */
    MEM_ZERO_STRUCT(stcDmaInit);
    stcDmaInit.u16BlockSize = 1u; /* 1 block */
    stcDmaInit.u16TransferCnt = (uint16_t)m_stcRxBufHanlde.u8Size;   /* Transfer count */
    stcDmaInit.u32SrcAddr = ((uint32_t)(&USART_CH->DR)+2ul);        /* Set source address. */
    stcDmaInit.u32DesAddr = (uint32_t)(m_stcRxBufHanlde.au8Buf);    /* Set destination address. */
    stcDmaInit.stcDmaChCfg.enSrcInc = AddressFix;           /* Set source address mode. */
    stcDmaInit.stcDmaChCfg.enDesInc = AddressIncrease;      /* Set destination address mode. */
    stcDmaInit.stcDmaChCfg.enIntEn = Enable;                /* Enable interrupt. */
    stcDmaInit.stcDmaChCfg.enTrnWidth = Dma8Bit;            /* Set data width 8bit. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaInit);

    /* Enable the specified DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH, Enable);

    /* Clear DMA flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);

    /* Enable peripheral circuit trigger function. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);

    /* Set DMA trigger source. */
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, DMA_TRG_SEL);

    /* Set DMA block transfer complete IRQ */
    stcIrqRegiCfg.enIRQn = DMA_BTC_INT_IRQn;
    stcIrqRegiCfg.pfnCallback = &DmaBtcIrqCallback;
    stcIrqRegiCfg.enIntSrc = DMA_BTC_INT_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
}

/**
 *******************************************************************************
 ** \brief DMA block transfer complete irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void DmaBtcIrqCallback(void)
{
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);
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
    uint16_t i;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    uint32_t u32Fcg1Periph = PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | \
                             PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;
    const stc_usart_clksync_init_t stcInitCfg = {
        UsartExtClk,
        UsartClkDiv_1,
        UsartDataLsbFirst,
        UsartRtsEnable,
    };

    /* Initialize buffer */
    m_stcRxBufHanlde.u8Size = (uint8_t)sizeof(m_stcRxBufHanlde.au8Buf);

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    /* Initialize DMA */
    DmaInit();

    /* Enable peripheral clock */
    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    /* Initialize USART IO */
    PORT_SetFunc(USART_CK_PORT, USART_CK_PIN, USART_CK_FUNC, Disable);
    PORT_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_FUNC, Disable);

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

    /* Wait transfer completely */
    while (Reset == DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq))
    {
    }

    for (i = 0u; i < m_stcRxBufHanlde.u8Size; i++)
    {
        if ( m_stcRxBufHanlde.au8Buf[i] != i)
        {
            m_enTestResult = Error;
        }
        else
        {
        }
    }

    if (Ok == m_enTestResult)
    {
        BSP_LED_On(LED_GREEN);  /* Test pass && meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);  /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
