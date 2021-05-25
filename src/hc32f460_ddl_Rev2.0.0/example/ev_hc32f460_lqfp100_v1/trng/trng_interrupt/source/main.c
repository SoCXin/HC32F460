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
 ** \brief TRNG sample
 **
 **   - 2018-10-20  CDT First version for Device Driver Library of
 **     Trng
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
/* TRNG clock selection definition. */
#define TRNG_CLK_PCLK4              (1u)
#define TRNG_CLK_MPLLQ              (2u)
#define TRNG_CLK_UPLLR              (3u)

/* Select UPLLR as TRNG clock. */
#define TRNG_CLK                    (TRNG_CLK_UPLLR)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void SystemClockConfig(void);

static void TrngConfig(void);
static void TrngClockConfig(void);
static void TrngInitConfig(void);
static void TrngIrqConfig(void);

static void TrngIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint32_t m_au32Random[2u];
static bool m_bTrngIrqFlag = false;

static stc_clk_sysclk_cfg_t m_stcSysclkCfg =
{
    /* Default system clock division. */
    .enHclkDiv  = ClkSysclkDiv1,  // 200MHz
    .enExclkDiv = ClkSysclkDiv2,  // 100Hz
    .enPclk0Div = ClkSysclkDiv1,  // 200MHz
    .enPclk1Div = ClkSysclkDiv2,  // 100MHz
    .enPclk2Div = ClkSysclkDiv4,  // 50MHz
    .enPclk3Div = ClkSysclkDiv4,  // 50MHz
    .enPclk4Div = ClkSysclkDiv2,  // 100MHz
};

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
    /* Configuring a new system clock if you need. */
    SystemClockConfig();

    /* Config TRNG. */
    TrngConfig();

    /* Start TRNG. */
    TRNG_StartIT();

    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    /***************** Configuration end, application start **************/

    while (1u)
    {
        /* Start TRNG. */
        TRNG_StartIT();

        /* Check TRNG. */
        while (false == m_bTrngIrqFlag)
        {
            ;
        }
        m_bTrngIrqFlag = false;

        DDL_Printf("Rand number: 0x%.8lx 0x%.8lx.\n", m_au32Random[0u], m_au32Random[1u]);
        Ddl_Delay1ms(100u);
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Configuring a new system clock.
 **         System clock frequency: 168MHz.
 **         System clock source:    MPLL.
 **         MPLL clock source:      XTAL(8MHz).
 **
 ******************************************************************************/
static void SystemClockConfig(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;
    stc_sram_config_t  stcSramConfig;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clock division first. */
    CLK_SysClkConfig(&m_stcSysclkCfg);

    /* Switch system clock source to MPLL. */
    /* Use XTAL as MPLL source. */
    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Set MPLL out 200MHz. */
    stcMpllCfg.pllmDiv = 1u;
    /* sysclk = 8M / pllmDiv * plln / PllpDiv */
    stcMpllCfg.plln    = 50u;
    stcMpllCfg.PllpDiv = 2u;
    stcMpllCfg.PllqDiv = 16u;
    stcMpllCfg.PllrDiv = 16u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* Flash read wait cycle setting. */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* If the system clock frequency is higher than 100MHz and SRAM1, SRAM2, SRAM3 or Ret_SRAM is used,
       the wait cycle must be set. */
    stcSramConfig.u8SramIdx     = Sram12Idx | Sram3Idx | SramRetIdx;
    stcSramConfig.enSramRC      = SramCycle2;
    stcSramConfig.enSramWC      = SramCycle2;
    stcSramConfig.enSramEccMode = EccMode0;
    stcSramConfig.enSramEccOp   = SramNmi;
    stcSramConfig.enSramPyOp    = SramNmi;
    SRAM_Init(&stcSramConfig);

    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
        ;
    }

    /* Set system clock source. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 *******************************************************************************
 ** \brief  TRNG configuration, including clock configuration,
 **         initial configuration and interrupt configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngConfig(void)
{
    TrngClockConfig();
    TrngInitConfig();
    TrngIrqConfig();
}

/**
 *******************************************************************************
 ** \brief  TRNG clock configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngClockConfig(void)
{
#if (TRNG_CLK == TRNG_CLK_PCLK4)
    /* PCLK4 is TRNG's clock. */
    /* TRNG's clock frequency below 1MHz(inclusive) if possible will be better. */
    m_stcSysclkCfg.enPclk4Div = ClkSysclkDiv64;  // 84MHz.
    CLK_SysClkConfig(&m_stcSysclkCfg);
    CLK_SetPeriClkSource(ClkPeriSrcPclk);

#elif (TRNG_CLK == TRNG_CLK_MPLLQ)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;
    en_clk_sys_source_t enSysClkSrc;

    enSysClkSrc = CLK_GetSysClkSource();
    if (enSysClkSrc == CLKSysSrcMPLL)
    {
        /*
         * Configure MPLLQ(same as MPLLP and MPLLR) when you
         * configure MPLL as the system clock.
         */
    }
    else
    {
        /* Use XTAL as MPLL source. */
        stcXtalCfg.enFastStartup = Enable;
        stcXtalCfg.enMode = ClkXtalModeOsc;
        stcXtalCfg.enDrv  = ClkXtalLowDrv;
        CLK_XtalConfig(&stcXtalCfg);
        CLK_XtalCmd(Enable);

        /* Set MPLL out 240MHz. */
        stcMpllCfg.pllmDiv = 1u;
        /* mpll = 8M / pllmDiv * plln */
        stcMpllCfg.plln    = 30u;
        stcMpllCfg.PllpDiv = 2u;
        stcMpllCfg.PllqDiv = 16u;
        stcMpllCfg.PllrDiv = 4u;
        CLK_SetPllSource(ClkPllSrcXTAL);
        CLK_MpllConfig(&stcMpllCfg);
        CLK_MpllCmd(Enable);
    }

    CLK_SetPeriClkSource(ClkPeriSrcMpllp);

#elif (TRNG_CLK == TRNG_CLK_UPLLR)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_upll_cfg_t stcUpllCfg;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcUpllCfg);

    /* Use XTAL as UPLL source. */
    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv  = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Set UPLL out 240MHz. */
    stcUpllCfg.pllmDiv = 2u;
    /* upll = 8M(XTAL) / pllmDiv * plln */
    stcUpllCfg.plln    = 60u;
    stcUpllCfg.PllpDiv = 2u;
    stcUpllCfg.PllqDiv = 16u;
    stcUpllCfg.PllrDiv = 16u;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_UpllConfig(&stcUpllCfg);
    CLK_UpllCmd(Enable);
    CLK_SetPeriClkSource(ClkPeriSrcUpllr);
#endif
}

/**
 *******************************************************************************
 ** \brief  TRNG initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngInitConfig(void)
{
    stc_trng_init_t stcTrngInit;

    stcTrngInit.enLoadCtrl   = TrngLoadNewInitValue_Enable;
    stcTrngInit.enShiftCount = TrngShiftCount_64;

    /* 1. Enable TRNG. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_TRNG, Enable);
    /* 2. Initialize TRNG. */
    TRNG_Init(&stcTrngInit);
}

/**
 *******************************************************************************
 ** \brief  TRNG interrupt configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void TrngIrqConfig(void)
{
    stc_irq_regi_conf_t stcTrngIrqCfg;
    en_result_t         enIrqRegResult;

    /* Config TRNG interrupt. */
    stcTrngIrqCfg.enIntSrc    = INT_TRNG_END;
    /* stcTrngIrqCfg.enIRQn: [Int000_IRQn, Int031_IRQn] [Int116_IRQn, Int121_IRQn] */
    stcTrngIrqCfg.enIRQn      = Int121_IRQn;
    stcTrngIrqCfg.pfnCallback = &TrngIrqCallback;
    enIrqRegResult = enIrqRegistration(&stcTrngIrqCfg);

    if (Ok == enIrqRegResult)
    {
        NVIC_ClearPendingIRQ(stcTrngIrqCfg.enIRQn);
        NVIC_SetPriority(stcTrngIrqCfg.enIRQn, DDL_IRQ_PRIORITY_03);
        NVIC_EnableIRQ(stcTrngIrqCfg.enIRQn);
    }
}

/**
 *******************************************************************************
 ** \brief  TRNG interrupt callback function.
 **         Its main function is to read random numbers.
 **
 ******************************************************************************/
static void TrngIrqCallback(void)
{
    TRNG_GetRandomNum(m_au32Random, 2u);
    m_bTrngIrqFlag = true;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
