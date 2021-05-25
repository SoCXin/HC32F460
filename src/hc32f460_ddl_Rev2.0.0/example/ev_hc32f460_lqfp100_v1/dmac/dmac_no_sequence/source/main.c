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
#define DMA_TRG_SEL             (INT_PORT_EIRQ4)  /* External Int Ch.4 trigger request number */
#define DMA_TRNCNT              (4u)
#define DMA_BLKSIZE             (5u)
#define DMA_SNSEQ_CNT           (4u)
#define DMA_SNSEQ_OFFSET        (6u)
#define DMA_DNSEQ_CNT           (5u)
#define DMA_DNSEQ_OFFSET        (6u)
#define DMA_TRIGGER_SRC         (EVT_AOS_STRG)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const uint32_t u32SrcBuf[40] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                   11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                   21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
                                   31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
static uint32_t u32DstBuf[40] = {0};

/* Note: if change DMAC configure, please modify the buffer data */
static uint32_t u32ExpectDstBufData[40] = { 1,  2,  3,  4, 10, 0, 0, 0, 0, 0,
                                           11, 12, 13, 19, 20, 0, 0, 0, 0, 0,
                                           21, 22, 28, 29, 30, 0, 0, 0, 0, 0,
                                           31, 37, 38, 39, 40, 0, 0, 0, 0, 0};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

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
    int32_t CmpRet = 0;
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Clk initialization */
    BSP_CLK_Init();
    /* Led initialization */
    BSP_LED_Init();

    /* Set data block size. */
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = DMA_TRNCNT;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u32SrcBuf[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u32DstBuf[0]);

    /* Enable non_sequence transfer. */
    stcDmaCfg.stcDmaChCfg.enSrcNseqEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesNseqEn = Enable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma32Bit;

    /* Enable DMA clock. */
#if (DMA_UNIT_INDEX == 1u)
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1,Enable);
#endif
#if (DMA_UNIT_INDEX == 2u)
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2,Enable);
#endif

    /* Init Dma channel. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);

    stc_dma_nseq_cfg_t stcSrcNseqCfg;
    stc_dma_nseq_cfg_t stcDesNseqCfg;

    stcSrcNseqCfg.u16Cnt = DMA_SNSEQ_CNT;
    stcSrcNseqCfg.u32Offset = DMA_SNSEQ_OFFSET;
    DMA_SetSrcNseqCfg(DMA_UNIT, DMA_CH, &stcSrcNseqCfg);

    stcDesNseqCfg.u16Cnt = DMA_DNSEQ_CNT;
    stcDesNseqCfg.u32Offset = DMA_DNSEQ_OFFSET;
    DMA_SetDesNseqCfg(DMA_UNIT, DMA_CH, &stcDesNseqCfg);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    /* Set dma trigger source. */
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, DMA_TRIGGER_SRC);

    /*  Clear interrupt flag  */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);

    /* Enable DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);

    /* 1st trigger for DMA */
    AOS_SW_Trigger();

    /*  Wait transfter completion  */
    while(Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq))
    {
        DMA_ClearIrqFlag(DMA_UNIT, DMA_CH,TrnCpltIrq);
        AOS_SW_Trigger();
        while(Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq))
        {
            ;
        }
    }

    /* Verify destination buffer data && expeted data */
    CmpRet = memcmp(u32DstBuf, u32ExpectDstBufData, sizeof(u32DstBuf));
    if(0 == CmpRet)
    {
        BSP_LED_On(LED_BLUE);     /* Meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);     /* Don't meet the expected */
    }

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
