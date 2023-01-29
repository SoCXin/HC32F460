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
 ** \brief This example demonstrates how to use read protection of SMPU2
 **        region5 4KByte range.
 **
 **   - 2018-10-20 CDT First version for Device Driver Library of MPU
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
/* MPU */
#define MPU_TYPE                        (SMPU2Region)
#define MPU_RIGION_NUM                  (MpuRegionNum5)
#define MPU_RIGION_SIZE                 (MpuRegionSize4KByte)

/* DMAC */
#define DMA_UNIT                        (M4_DMA2)
#define DMA_CH                          (DmaCh0)
#define DMA_TRG_SEL                     (EVT_AOS_STRG)
#define DMA_TRNCNT                      (2u)
#define DMA_BLKSIZE                     (1024u)
#define DMA_RPTSIZE                     (DMA_BLKSIZE)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void DmaInit(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
__attribute__ ((aligned (DMA_BLKSIZE)))
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

__attribute__ ((aligned (DMA_BLKSIZE)))
static uint8_t m_au8DstBuf[DMA_BLKSIZE];

#elif defined (__ICCARM__)                /* IAR Compiler */
#pragma data_alignment=4096
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

#pragma data_alignment=4096
static uint8_t m_au8DstBuf[DMA_BLKSIZE] = {0};

#elif defined (__CC_ARM)                /* ARM Compiler */
__align(4096)
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

__align(4096)
static uint8_t m_au8DstBuf[DMA_BLKSIZE] = {0};
#endif

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

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1 | PWC_FCG0_PERIPH_DMA2,Enable);

    /* Enable DMA. */
    DMA_Cmd(DMA_UNIT,Enable);

    /* Initialize DMA. */
    MEM_ZERO_STRUCT(stcDmaInit);
#if (DMA_BLKSIZE == 1024u)
    stcDmaInit.u16BlockSize = 0u; /* 0 == 1024Byte */
#else
    stcDmaInit.u16BlockSize = DMA_BLKSIZE;
#endif
    stcDmaInit.u16TransferCnt = DMA_TRNCNT;   /* Set transfer count. */
    stcDmaInit.u32SrcAddr = (uint32_t)(&m_au8SrcBuf[0]);  /* Set source address. */
    stcDmaInit.u32DesAddr = (uint32_t)(&m_au8DstBuf[0]);  /* Set destination address. */
#if (DMA_RPTSIZE == 1024u)
    stcDmaInit.u16SrcRptSize = 0u; /* 0 == 1024Byte */
    stcDmaInit.u16DesRptSize = 0u; /* 0 == 1024Byte */
#else
    stcDmaInit.u16SrcRptSize = DMA_BLKSIZE;
    stcDmaInit.u16DesRptSize = DMA_BLKSIZE;
#endif
    stcDmaInit.stcDmaChCfg.enSrcRptEn = Enable;  /* Enable repeat. */
    stcDmaInit.stcDmaChCfg.enDesRptEn = Enable;  /* Enable repeat. */
    stcDmaInit.stcDmaChCfg.enSrcInc = AddressIncrease;  /* Set source address mode. */
    stcDmaInit.stcDmaChCfg.enDesInc = AddressIncrease;  /* Set destination address mode. */
    stcDmaInit.stcDmaChCfg.enIntEn = Enable;      /* Enable interrupt. */
    stcDmaInit.stcDmaChCfg.enTrnWidth = Dma8Bit;  /* Set data width 8bit. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaInit);

    /* Enable the specified DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH, Enable);

    /* Clear DMA flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);

    /* Enable peripheral circuit trigger function. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS,Enable);

    /* Set DMA trigger source. */
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, DMA_TRG_SEL);
}

/**
 *******************************************************************************
 ** \brief Initialize DMA buffer.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void DmaBufferInit(void)
{
    /* Initialize source buffer */
    for (uint16_t i = 0u; i < DMA_BLKSIZE; i++)
    {
        m_au8SrcBuf[i] = (uint8_t)(i % 256u);
        m_au8DstBuf[i] = 0u;
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
    en_result_t enTestResult = Ok;
    stc_mpu_prot_region_init_t stcProtRegionInit;
    en_flag_status_t enTrnCpltFlag = Reset;
    en_flag_status_t enBlkTrnCpltFlag = Reset;

    /* Disable SMPU region */
    MPU_WriteProtCmd(Disable);
    MPU_RegionTypeCmd(MPU_TYPE, Disable);

    /* Initialize buffer && LED && DMA */
    DmaBufferInit();
    DmaInit();

    /* Initialize LED port */
    BSP_LED_Init();

    /* Trigger DMA */
    AOS_SW_Trigger();

    while (Reset == DMA_GetIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq)) /* Wait DMA block transfer complete */
    {
    }

    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);

    if (0 != memcmp(m_au8SrcBuf, m_au8DstBuf, sizeof(m_au8DstBuf))) /* Verify DMA function */
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Initialize buffer */
    DmaBufferInit();

    /* Initialize SMPU */
    MEM_ZERO_STRUCT(stcProtRegionInit);
    stcProtRegionInit.u32RegionBaseAddress = (uint32_t)m_au8SrcBuf;
    stcProtRegionInit.enRegionSize = MPU_RIGION_SIZE;
    stcProtRegionInit.stcSMPU2Permission.enRegionEnable = Enable;
    stcProtRegionInit.stcSMPU2Permission.enWriteEnable = Enable;
    stcProtRegionInit.stcSMPU2Permission.enReadEnable = Disable;
    if (Ok != MPU_ProtRegionInit(MPU_RIGION_NUM, &stcProtRegionInit))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Enable SMPU region */
    MPU_RegionTypeCmd(MPU_TYPE, Enable);

    /* Trigger DMA */
    AOS_SW_Trigger();

    /* Wait DMA transfer complete */
    while ((Set != enTrnCpltFlag) || (Set != enBlkTrnCpltFlag))
    {
        enTrnCpltFlag = DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);
        enBlkTrnCpltFlag = DMA_GetIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);
    }

    /* DMA source buffer is protected(forbid read) by MPU, so DMA can't read source buffer */
    if (0 == memcmp(m_au8SrcBuf, m_au8DstBuf, sizeof(m_au8DstBuf)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Clear DMA flag */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);
    MPU_ClearStatus(MPU_TYPE);

    if (Ok == enTestResult)
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
