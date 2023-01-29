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
 ** \brief clock switch system clock sample
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of
 **     clock
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
#define CLK_HP_FREQ         (168 * 1000 * 1000u)
#define CLK_HS_FREQ         (8 * 1000 * 1000u)
/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Xtal initialize
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void XtalInit(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;

    MEM_ZERO_STRUCT(stcXtalCfg);

    /* Config Xtal and Enable Xtal */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  Xtal initialize
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Xtal32Init(void)
{
    stc_clk_xtal32_cfg_t stcXtal32Cfg;

    MEM_ZERO_STRUCT(stcXtal32Cfg);

    stcXtal32Cfg.enDrv        = ClkXtal32MidDrv;
    stcXtal32Cfg.enFilterMode = ClkXtal32FilterModeFull;
    CLK_Xtal32Config(&stcXtal32Cfg);
    CLK_Xtal32Cmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  MPLL initialize
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void MpllInit(void)
{
    stc_clk_mpll_cfg_t      stcMpllCfg;
    stc_sram_config_t       stcSramConfig;

    MEM_ZERO_STRUCT(stcMpllCfg);
    MEM_ZERO_STRUCT(stcSramConfig);

    /* sram init include read/write wait cycle setting */
    stcSramConfig.u8SramIdx = Sram12Idx | Sram3Idx | SramHsIdx | SramRetIdx;
    stcSramConfig.enSramRC = SramCycle2;
    stcSramConfig.enSramWC = SramCycle2;
    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramEccOp = SramNmi;
    stcSramConfig.enSramPyOp = SramNmi;
    SRAM_Init(&stcSramConfig);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* MPLL config (XTAL / pllmDiv * plln / PllpDiv = 200M). */
    stcMpllCfg.pllmDiv = 1ul;
    stcMpllCfg.plln =50ul;
    stcMpllCfg.PllpDiv = 2ul;
    stcMpllCfg.PllqDiv = 2ul;
    stcMpllCfg.PllrDiv = 2ul;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);
    /* Enable MPLL. */
    CLK_MpllCmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  Main function of switch system clock source to MPLL project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_output_cfg_t    stcOutputClkCfg;

    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcOutputClkCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Configure clock output system clock */
    stcOutputClkCfg.enOutputSrc = ClkOutputSrcSysclk;
    stcOutputClkCfg.enOutputDiv = ClkOutputDiv8;
    CLK_OutputClkConfig(ClkOutputCh1,&stcOutputClkCfg);
    /* Configure clock output pin */
    PORT_SetFunc(PortA, Pin08, Func_Mclkout, Disable);
    /* MCO1 output enable */
    CLK_OutputClkCmd(ClkOutputCh1,Enable);

    /* Key initialization */
    BSP_KEY_Init();

    XtalInit();
    Xtal32Init();
    MpllInit();
    CLK_LrcCmd(Enable);
    CLK_HrcCmd(Enable);

    /* Switch driver ability */
    PWC_HS2HP();
    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);

    while(1)
    {
        /* MRC output */
        if (Set == BSP_KEY_GetStatus(BSP_KEY_1))
        {
            if (SystemCoreClock > CLK_HP_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcMRC);
                PWC_HP2HS();
            }
            else if (SystemCoreClock < CLK_HS_FREQ)
            {
                PWC_LS2HS();
                CLK_SetSysClkSource(ClkSysSrcMRC);
            }
            else
            {
                CLK_SetSysClkSource(ClkSysSrcMRC);
            }
        }
        /* Xtal output */
        if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
        {
            if (SystemCoreClock > CLK_HP_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcXTAL);
                PWC_HP2HS();
            }
            else if (SystemCoreClock < CLK_HS_FREQ)
            {
                PWC_LS2HS();
                CLK_SetSysClkSource(ClkSysSrcXTAL);
            }
            else
            {
                CLK_SetSysClkSource(ClkSysSrcXTAL);
            }
        }
        /* HRC output */
        if (Set == BSP_KEY_GetStatus(BSP_KEY_3))
        {
            if (SystemCoreClock > CLK_HP_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcHRC);
                PWC_HP2HS();
            }
            else if (SystemCoreClock < CLK_HS_FREQ)
            {
                PWC_LS2HS();
                CLK_SetSysClkSource(ClkSysSrcHRC);
            }
            else
            {
                CLK_SetSysClkSource(ClkSysSrcHRC);
            }
        }
        /* LRC output */
        if (Set == BSP_KEY_GetStatus(BSP_KEY_4))
        {
            if (SystemCoreClock > CLK_HP_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcLRC);
                PWC_HP2LS();
            }
            else if (SystemCoreClock < CLK_HS_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcLRC);
            }
            else
            {
                CLK_SetSysClkSource(ClkSysSrcLRC);
                PWC_HS2LS();
            }
        }
        /* XTAL32 output */
        if (Set == BSP_KEY_GetStatus(BSP_KEY_5))
        {
            if (SystemCoreClock > CLK_HP_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcXTAL32);
                PWC_HP2LS();
            }
            else if (SystemCoreClock < CLK_HS_FREQ)
            {
                CLK_SetSysClkSource(ClkSysSrcXTAL32);
            }
            else
            {
                CLK_SetSysClkSource(ClkSysSrcXTAL32);
                PWC_HS2LS();
            }
        }
        /* MPLL output */
        if (Set == BSP_KEY_GetStatus(BSP_KEY_6))
        {
            if (SystemCoreClock > CLK_HP_FREQ)
            {
                CLK_SetSysClkSource(CLKSysSrcMPLL);
            }
            else if (SystemCoreClock < CLK_HS_FREQ)
            {
                PWC_LS2HP();
                CLK_SetSysClkSource(CLKSysSrcMPLL);
            }
            else
            {
                PWC_HS2HP();
                CLK_SetSysClkSource(CLKSysSrcMPLL);
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
