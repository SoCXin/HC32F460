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
#define DMA_UNIT_INDEX          (1u)

#define DMA_UNIT                (M4_DMA1)
#define DMA_CH                  (DmaCh3)
#define DMA_TRG_SEL             (INT_PORT_EIRQ4)  /* External Int Ch.4 trigger request number */
#define DMA_TRNCNT              (1u)
#define DMA_BLKSIZE             (ARRAY_SZ(u16SrcBuf))
#define DMA_LLP_MODE            (LlpRunNow)
#define DMA_INC_MODE            (AddressIncrease)
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
static const uint8_t u8SrcBuf[10] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
static uint8_t u8DstBuf[10] = {0};

static const uint16_t u16SrcBuf[10] = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
static uint16_t u16DstBuf[10] = {0};

static const uint32_t u32SrcBuf[10] = {31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
static uint32_t u32DstBuf[10] = {0};

static stc_dma_llp_descriptor_t stcLlpDesc[2] = {0};
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
    uint32_t u32CmpRet = 0ul;
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Clk initialization */
    BSP_CLK_Init();
    /* Led initialization */
    BSP_LED_Init();

    /* descriptor 0 */
    stcLlpDesc[0].SARx = (uint32_t)(&u16SrcBuf[0]);
    stcLlpDesc[0].DARx = (uint32_t)(&u16DstBuf[0]);
    stcLlpDesc[0].DTCTLx_f.CNT = DMA_TRNCNT;
    stcLlpDesc[0].DTCTLx_f.BLKSIZE = DMA_BLKSIZE;
    stcLlpDesc[0].LLPx = (uint32_t)(&stcLlpDesc[1]);
    stcLlpDesc[0].CHxCTL_f.SINC = DMA_INC_MODE;
    stcLlpDesc[0].CHxCTL_f.DINC = DMA_INC_MODE;
    stcLlpDesc[0].CHxCTL_f.HSIZE = Dma16Bit;
    stcLlpDesc[0].CHxCTL_f.LLPEN = Enable;
    stcLlpDesc[0].CHxCTL_f.LLPRUN = DMA_LLP_MODE;

    /* descriptor 1 */
    stcLlpDesc[1].SARx = (uint32_t)(&u32SrcBuf[0]);
    stcLlpDesc[1].DARx = (uint32_t)(&u32DstBuf[0]);
    stcLlpDesc[1].DTCTLx_f.CNT = DMA_TRNCNT;
    stcLlpDesc[1].DTCTLx_f.BLKSIZE = DMA_BLKSIZE;
    stcLlpDesc[1].CHxCTL_f.SINC = DMA_INC_MODE;
    stcLlpDesc[1].CHxCTL_f.DINC = DMA_INC_MODE;
    stcLlpDesc[1].CHxCTL_f.HSIZE = Dma32Bit;
    stcLlpDesc[1].CHxCTL_f.LLPEN = Disable;

    /* Set data block size. */
    stcDmaCfg.u16BlockSize = (uint16_t)DMA_BLKSIZE;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = DMA_TRNCNT;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u8SrcBuf[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u8DstBuf[0]);

    /* Config dma channel. */
    /* Enable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Enable;
    stcDmaCfg.stcDmaChCfg.enLlpMd = DMA_LLP_MODE;
    stcDmaCfg.u32DmaLlp = (uint32_t)(&stcLlpDesc[0]);
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    /* Enable DMA clock. */
#if (DMA_UNIT_INDEX == 1u)
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1,Enable);
#endif
#if (DMA_UNIT_INDEX == 2u)
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2,Enable);
#endif

    /* Init Dma channel. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);
    /* Set dma trigger source. */
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, DMA_TRIGGER_SRC);

    /*  Clear interrupt flag  */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);

    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);

    AOS_SW_Trigger();

    /*  Wait transfter completion  */
    while(Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq))
    {
        ;
    }

    /* Verify destination buffer data && expeted data */
    if(0 != memcmp(u8DstBuf, u8SrcBuf, sizeof(u8SrcBuf)))
    {
        u32CmpRet += 1ul;
    }

    if(0 != memcmp(u16DstBuf, u16SrcBuf, sizeof(u16SrcBuf)))
    {
        u32CmpRet += 1ul;
    }

    if(0 != memcmp(u32DstBuf, u32SrcBuf, sizeof(u32SrcBuf)))
    {
        u32CmpRet += 1ul;
    }

    if(u32CmpRet == 0ul)
    {
        BSP_LED_On(LED_BLUE);     /* Meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);      /* Don't meet the expected */
    }

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
