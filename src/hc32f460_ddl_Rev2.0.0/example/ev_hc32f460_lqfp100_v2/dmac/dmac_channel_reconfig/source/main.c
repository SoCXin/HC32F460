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
 ** \brief dmac sample
 **
 **   - 2018-10-20  CDT  First version for Device Driver Library of
 **     dmac
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
/* DMAC */
/* DMA_UNIT_NUM = 1 means M4_DMA1, DMA_UNIT_NUM = 2 means  M4_DMA2. */
#define DMA_UNIT_INDEX          (2u)

#define DMA_UNIT                (M4_DMA2)
#define DMA_CH                  (DmaCh0)
#define DMA_TRNCNT              (20u)
#define DMA_BLKSIZE             (1u)
#define DMA_RPTB_SIZE           (5u)
#define DMA_TRIGGER_SRC         (EVT_AOS_STRG)
#define DMA_RECFG_TRIGGER_SRC   (EVT_PORT_EIRQ1)

#define KEY10_PORT              (PortB)
#define KEY10_PIN               (Pin01)
#define KEY10_EXINT_CH          (ExtiCh01)
#define KEY10_INT_SRC           (INT_PORT_EIRQ1)
#define KEY10_INT_IRQn          (Int007_IRQn)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
int32_t CmpRet = 1;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const uint32_t u32SrcBuf[22] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                       11, 12, 13, 14, 15, 16, 17, 18,
                                       19, 20, 21, 22};
static uint32_t u32DstBuf[22] = {0};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  KEY10 interrupt callback function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt01_Callback(void)
{
    static uint32_t u32ExpectDstBufData[22] = {1, 2, 3, 4, 5, 1, 2, 3, 4, 5};
    EXINT_IrqFlgClr(KEY10_EXINT_CH);
    AOS_SW_Trigger();

    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
    {
        AOS_SW_Trigger();
    }
    CmpRet = memcmp(u32DstBuf, u32ExpectDstBufData, sizeof(u32DstBuf));
}
/**
 *******************************************************************************
 ** \brief KEY10(SW10) init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Key10_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /**************************************************************************/
    /* External Int Ch.3                                                      */
    /**************************************************************************/
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY10_PORT, KEY10_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = KEY10_EXINT_CH;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    /* Falling edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.1 */
    stcIrqRegiConf.enIntSrc = KEY10_INT_SRC;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = KEY10_INT_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &ExtInt01_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief DMA init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Dma_Init(void)
{
    stc_dma_config_t    stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Set data block size. */
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = DMA_TRNCNT;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u32SrcBuf[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u32DstBuf[0]);

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Enable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Disable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Disable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma32Bit;

    /* Initialize DMA. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);
}

/**
 *******************************************************************************
 ** \brief DMA Re_Config init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Dma_ReCfgInit(void)
{
    stc_dma_recfg_ctl_t stcDmaReCfg;

    MEM_ZERO_STRUCT(stcDmaReCfg);

    /* set remain transfer count after re_config :
    same with transfer count whitch DMA_SNSEQCTLBx.SNSCNTB / DMA_RPTBx.SRPTB set.*/
    stcDmaReCfg.enCntMd = CntSrcAddr;
    /* destination address update DMA_DARx after re_config */
    stcDmaReCfg.enDaddrMd = DaddrRep;
    /* source address update DMA_SARx after re_config */
    stcDmaReCfg.enSaddrMd = SaddrRep;
    /* re_config destination repeat size */
    stcDmaReCfg.u16DesRptBSize = DMA_RPTB_SIZE + 5u;
    /* re_config source repeat size */
    stcDmaReCfg.u16SrcRptBSize = DMA_RPTB_SIZE;

    /* Initialize DMA re_config */
    DMA_InitReConfig(DMA_UNIT, DMA_CH, &stcDmaReCfg);
    /* Enable DMA re_config */
    DMA_ReCfgCmd(DMA_UNIT, Enable);
}
/**
 *******************************************************************************
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /* Clk initialization */
    BSP_CLK_Init();
    /* Led initialization */
    BSP_LED_Init();
    /* Key initialization */
    Key10_Init();

    /* Enable DMA clock. */
#if (DMA_UNIT_INDEX == 1u)
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1,Enable);
#endif
#if (DMA_UNIT_INDEX == 2u)
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2,Enable);
#endif

    /* Enable DMA. */
    DMA_Cmd(DMA_UNIT,Enable);

    /* Init Dma channel */
    Dma_Init();
    /* Init Dma re_config */
    Dma_ReCfgInit();

    /* Enable DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);

    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH,TrnCpltIrq);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);

    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, DMA_TRIGGER_SRC);
    DMA_SetReConfigTriggerSrc(DMA_RECFG_TRIGGER_SRC);

    /* KEY10 */
    while(Reset != PORT_GetBit(KEY10_PORT, KEY10_PIN))
    {
        ;
    }

    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
    {
        AOS_SW_Trigger();
    }


    if(0 == CmpRet)
    {
        BSP_LED_On(LED_BLUE);    /* Meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);    /* Don't meet the expected */
    }

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
