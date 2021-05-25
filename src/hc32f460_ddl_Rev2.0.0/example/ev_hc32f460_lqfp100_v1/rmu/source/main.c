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
 ** \brief RMU sample
 **
 **   - 2018-11-18  CDT First version for Device Driver Library of
 **     RMU
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
/* Reset mode definition. */
#define POWER_ON_RESET              (0u)
#define NRST_PIN_RESET              (1u)
#define UNDERVOLTAGE_RESET          (2u)
#define PVD1_RESET                  (3u)
#define PVD2_RESET                  (4u)
#define WDT_RESET                   (5U)
#define SWDT_RESET                  (6u)
#define POWER_DOWN_RESET            (7u)
#define SOFTWARE_RESET              (8u)
#define MPU_ERR_RESET               (9u)
#define RAM_PARITY_ERR_RESET        (10u)
#define RAM_ECC_RESET               (11u)
#define CLK_FREQ_ERR_RESET          (12u)
#define XTAL_ERR_RESET              (13u)
#define NORMAL_RESET                (0xFFu)

#define RESET_MODE                  (NORMAL_RESET)

#if (RESET_MODE == MPU_ERR_RESET)
#define MPU_TYPE                    (SMPU1Region)
#define MPU_RIGION_NUM              (MpuRegionNum0)
#define MPU_RIGION_SIZE             (MpuRegionSize32Byte)

#define DMA_UNIT                    (M4_DMA1)
#define DMA_CH                      (DmaCh0)
#define DMA_TRG_SEL                 (EVT_AOS_STRG)
#define DMA_TRNCNT                  (2u)
#define DMA_BLKSIZE                 (64u)
#define DMA_RPTSIZE                 (DMA_BLKSIZE)
#endif

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedIndicate(void);
static void SetResetMode(void);
static void MakeReset(void);
static void PrintResetMode(stc_rmu_rstcause_t stcRst);

#if (RESET_MODE == MPU_ERR_RESET)
static void DmaInit(void);
static void DmaBufferInit(void);
#endif

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
#if (RESET_MODE == MPU_ERR_RESET)

#if defined (__ICCARM__)                /* IAR Compiler */
#pragma data_alignment=DMA_BLKSIZE
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

#pragma data_alignment=DMA_BLKSIZE
static uint8_t m_au8DstBuf[DMA_BLKSIZE] = {0};

#elif defined (__CC_ARM)                /* ARM Compiler */
__align(DMA_BLKSIZE)
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

__align(DMA_BLKSIZE)
static uint8_t m_au8DstBuf[DMA_BLKSIZE] = {0};
#endif

#endif // #if (RESET_MODE == MPU_ERR_RESET)

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None.
 **
 ** \retval int32_t return value, if needed.
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_rmu_rstcause_t stcResetFlag;

    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    BSP_LED_Init();
    Ddl_Delay1ms(10u);

    RMU_GetResetCause(&stcResetFlag);
    RMU_ClrResetFlag();
    PrintResetMode(stcResetFlag);

    SetResetMode();

    Ddl_Delay1ms(100u);
    MakeReset();
    while (1u)
    {
        //: YOUR CODE
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Led indicate.
 **
 ******************************************************************************/
static void LedIndicate(void)
{
    BSP_LED_On(LED_RED);
    Ddl_Delay1ms(500u);
    BSP_LED_Off(LED_RED);
}

/**
 *******************************************************************************
 ** \brief Set reset mode.
 **
 ******************************************************************************/
static void SetResetMode(void)
{
#if (RESET_MODE == UNDERVOLTAGE_RESET)

#elif (RESET_MODE == PVD1_RESET)
    stc_pwc_pvd_cfg_t stcPvdCfg;
    MEM_ZERO_STRUCT(stcPvdCfg);

    stcPvdCfg.stcPvd1Ctl.enPvdIREn = Enable;
    stcPvdCfg.stcPvd1Ctl.enPvdMode = PvdReset;
    stcPvdCfg.stcPvd1Ctl.enPvdCmpOutEn = Enable;
    stcPvdCfg.enPvd1Level = Pvd1Level21;
    PWC_PvdCfg(&stcPvdCfg);
    PWC_Pvd1Cmd(Enable);

#elif (RESET_MODE == PVD2_RESET)
    stc_pwc_pvd_cfg_t stcPvdCfg;
    MEM_ZERO_STRUCT(stcPvdCfg);

    stcPvdCfg.stcPvd2Ctl.enPvdIREn = Enable;
    stcPvdCfg.stcPvd2Ctl.enPvdMode = PvdReset;
    stcPvdCfg.stcPvd2Ctl.enPvdCmpOutEn = Enable;
    stcPvdCfg.enPvd2Level = Pvd2Level3;
    PWC_PvdCfg(&stcPvdCfg);
    PWC_Pvd2Cmd(Enable);

#elif (RESET_MODE == WDT_RESET)
    stc_wdt_init_t stcWdtInit;
    MEM_ZERO_STRUCT(stcWdtInit);

    stcWdtInit.enClkDiv = WdtPclk3Div512;
    stcWdtInit.enCountCycle = WdtCountCycle16384;
    stcWdtInit.enRefreshRange = WdtRefresh0To25Pct;
    stcWdtInit.enSleepModeCountEn = Disable;
    stcWdtInit.enRequestType = WdtTriggerResetRequest;
    WDT_Init(&stcWdtInit);
    WDT_RefreshCounter();

#elif (RESET_MODE == SWDT_RESET)

#elif (RESET_MODE == POWER_DOWN_RESET)

#elif (RESET_MODE == SOFTWARE_RESET)

#elif (RESET_MODE == MPU_ERR_RESET)
    stc_mpu_prot_region_init_t stcProtRegionInit;
    /* Disable SMPU region */
    MPU_WriteProtCmd(Disable);
    MPU_RegionTypeCmd(MPU_TYPE, Disable);
    /* Initialize SMPU */
    MEM_ZERO_STRUCT(stcProtRegionInit);
    stcProtRegionInit.u32RegionBaseAddress = (uint32_t)(&m_au8DstBuf[0]);
    stcProtRegionInit.enRegionSize = MPU_RIGION_SIZE;
    stcProtRegionInit.stcSMPU1Permission.enRegionEnable = Enable;
    stcProtRegionInit.stcSMPU1Permission.enWriteEnable = Disable;
    stcProtRegionInit.stcSMPU1Permission.enReadEnable = Enable;
    MPU_ProtRegionInit(MPU_RIGION_NUM, &stcProtRegionInit);
    MPU_SetNoPermissionAcessAction(MPU_TYPE, MpuTrigReset);
    MPU_RegionTypeCmd(MPU_TYPE, Enable);

    DmaBufferInit();
    DmaInit();

#elif (RESET_MODE == RAM_PARITY_ERR_RESET)
    stc_sram_config_t stcSramConfig;
    MEM_ZERO_STRUCT(stcSramConfig);

    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramPyOp    = SramReset;
    SRAM_Init(&stcSramConfig);
    SRAM_WT_Enable();
    SRAM_CK_Enable();

#elif (RESET_MODE == RAM_ECC_RESET)
    stc_sram_config_t stcSramConfig;
    MEM_ZERO_STRUCT(stcSramConfig);

    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramEccOp   = SramReset;
    SRAM_Init(&stcSramConfig);
    SRAM_WT_Enable();
    SRAM_CK_Enable();

#elif (RESET_MODE == CLK_FREQ_ERR_RESET)

#elif (RESET_MODE == XTAL_ERR_RESET)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_xtal_stp_cfg_t stcXtalStpCfg;
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcXtalStpCfg);

    stcXtalStpCfg.enDetect = Disable;
    stcXtalStpCfg.enMode = ClkXtalStpModeReset;
    stcXtalStpCfg.enModeReset = Enable;
    stcXtalStpCfg.enModeInt = Disable;
    CLK_XtalStpConfig(&stcXtalStpCfg);

    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);

    CLK_XtalCmd(Enable);
    Ddl_Delay1ms(100u);

    stcXtalStpCfg.enDetect = Enable;
    CLK_XtalStpConfig(&stcXtalStpCfg);

#else
#endif
}

/**
 *******************************************************************************
 ** \brief  Make an reset.
 **
 ******************************************************************************/
static void MakeReset(void)
{
#if (RESET_MODE == UNDERVOLTAGE_RESET)

#elif (RESET_MODE == PVD1_RESET)

#elif (RESET_MODE == PVD2_RESET)

#elif (RESET_MODE == WDT_RESET)

#elif (RESET_MODE == SWDT_RESET)

#elif (RESET_MODE == POWER_DOWN_RESET)

#elif (RESET_MODE == SOFTWARE_RESET)
    __NVIC_SystemReset();

#elif (RESET_MODE == MPU_ERR_RESET)
    /* Trigger DMA to write m_au8DstBuf */
    AOS_SW_Trigger();

#elif (RESET_MODE == RAM_PARITY_ERR_RESET)

#elif (RESET_MODE == RAM_ECC_RESET)

#elif (RESET_MODE == CLK_FREQ_ERR_RESET)

#elif (RESET_MODE == XTAL_ERR_RESET)

#else
#endif
}

/**
 *******************************************************************************
 ** \brief  Print reset information.
 **
 ******************************************************************************/
static void PrintResetMode(stc_rmu_rstcause_t stcRst)
{
    if (Set == stcRst.enMultiRst)
    {
        DDL_Printf("Multiple reset\n");
    }
    if (Set == stcRst.enPowerOn)
    {
        DDL_Printf("Power on reset.\n");
    }
    if (Set == stcRst.enRstPin)
    {
        LedIndicate();
        DDL_Printf("NRST pin reset.\n");
    }
    if (Set == stcRst.enBrownOut)
    {
        DDL_Printf("Undervoltage reset.\n");
    }
    if (Set == stcRst.enPvd1)
    {
        DDL_Printf("Pvd1 reset.\n");
    }
    if (Set == stcRst.enPvd2)
    {
        DDL_Printf("Pvd2 reset.\n");
    }
    if (Set == stcRst.enWdt)
    {
#if (RESET_MODE == WDT_RESET)
        BSP_LED_On(LED_RED);
        Ddl_Delay1ms(500u);
        BSP_LED_Off(LED_RED);
#endif
        DDL_Printf("WDT reset.\n");
    }
    if (Set == stcRst.enSwdt)
    {
        DDL_Printf("Special WDT reset.\n");
    }
    if (Set == stcRst.enPowerDown)
    {
        DDL_Printf("Power down reset.\n");
    }
    if (Set == stcRst.enSoftware)
    {
        DDL_Printf("Software reset.\n");
    }
    if (Set == stcRst.enMpuErr)
    {
        DDL_Printf("MPU error reset.\n");
    }
    if (Set == stcRst.enRamParityErr)
    {
        DDL_Printf("RAM parity error reset.\n");
    }
    if (Set == stcRst.enRamEcc)
    {
        DDL_Printf("RAM ECC reset.\n");
    }
    if (Set == stcRst.enClkFreqErr)
    {
        DDL_Printf("Clock frequence error reset.\n");
    }
    if (Set == stcRst.enXtalErr)
    {
        DDL_Printf("XTAL error reset.\n");
    }
}

#if (RESET_MODE == MPU_ERR_RESET)
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
    stcDmaInit.u16BlockSize = DMA_BLKSIZE;    /* Set data block size. */
    stcDmaInit.u16TransferCnt = DMA_TRNCNT;   /* Set transfer count. */
    stcDmaInit.u32SrcAddr = (uint32_t)(&m_au8SrcBuf[0]);  /* Set source address. */
    stcDmaInit.u32DesAddr = (uint32_t)(&m_au8DstBuf[0]);  /* Set destination address. */
    stcDmaInit.u16SrcRptSize = DMA_RPTSIZE;      /* Set repeat size. */
    stcDmaInit.u16DesRptSize = DMA_RPTSIZE;      /* Set repeat size. */
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
    for(uint16_t i = 0; i < DMA_BLKSIZE; i++)
    {
        m_au8SrcBuf[i] = i;
        m_au8DstBuf[i] = 0;
    }
}
#endif // #if (RESET_MODE == MPU_ERR_RESET)

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
