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
 ** \brief ADC sample
 **
 **   - 2018-11-30  CDT First version for Device Driver Library of
 **     ADC
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
/* ADC clock selection definition. */
#define ADC_CLK_PCLK                (1u)
#define ADC_CLK_MPLLQ               (2u)
#define ADC_CLK_UPLLR               (3u)

/* Select MPLLQ as ADC clock. */
#define ADC_CLK                     (ADC_CLK_PCLK)

/* ADC1 channel definition for this example. */
#define ADC1_SA_NORMAL_CHANNEL      (ADC1_CH_INTERNAL)
#define ADC1_SA_CHANNEL             (ADC1_SA_NORMAL_CHANNEL)
#define ADC1_SA_CHANNEL_COUNT       (1u)

#define ADC1_CHANNEL                (ADC1_SA_CHANNEL)

#define ADC1_INTERNAL_INDEX         (ADC_CH_IDX16)

/* ADC1 channel sampling time.      ADC1_CH_INTERNAL */
#define ADC1_SA_CHANNEL_SAMPLE_TIME { 0x30 }

/* ADC2 channel definition for this example. */
#define ADC2_SA_NORMAL_CHANNEL      (ADC2_CH_INTERNAL)
#define ADC2_SA_CHANNEL             (ADC2_SA_NORMAL_CHANNEL)
#define ADC2_SA_CHANNEL_COUNT       (1u)

#define ADC2_CHANNEL                (ADC2_SA_CHANNEL)

#define ADC2_INTERNAL_INDEX         (ADC_CH_IDX8)

/* ADC2 channel sampling time.     ADC2_CH_INTERNAL */
#define ADC2_SA_CHANNEL_SAMPLE_TIME { 0x30 }

/* ADC internal channel selection definition. */
#define USE_ADC1_INTERNAL           (1u)
#define USE_ADC2_INTERNAL           (2u)

/* Select ADC1 internal channel. */
#define USE_INTERNAL_CH             (USE_ADC1_INTERNAL)

#if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
#define DAC1_TO_ADC1                (1u)
#define DAC2_TO_ADC1                (2u)
#define VREF_TO_ADC1                (3u)
#define ADC1_INTERNAL_SRC           (VREF_TO_ADC1)
#else
#define DAC1_TO_ADC2                (1u)
#define DAC2_TO_ADC2                (2u)
#define VREF_TO_ADC2                (3u)
#define ADC2_INTERNAL_SRC           (DAC2_TO_ADC2)
#endif

#define TIMEOUT_VAL                 (10u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void AdcConfig(void);
static void AdcClockConfig(void);
static void AdcInitConfig(void);
static void AdcChannelConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t m_u16AdcValue;

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
    /* Default clock is MRC(8MHz). */

    /* Config ADC. */
    AdcConfig();

    /***************** Configuration end, application start **************/

    while (1u)
    {
#if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
        ADC_StartConvert(M4_ADC1);
        while (Reset == ADC_GetEocFlag(M4_ADC1, ADC_SEQ_A))
        {
            ;
        }
        m_u16AdcValue = ADC_GetValue(M4_ADC1, ADC1_INTERNAL_INDEX);
        ADC_ClrEocFlag(M4_ADC1, ADC_SEQ_A);
#else
        ADC_StartConvert(M4_ADC2);
        while (Reset == ADC_GetEocFlag(M4_ADC2, ADC_SEQ_A))
        {
            ;
        }
        m_u16AdcValue = ADC_GetValue(M4_ADC2, ADC2_INTERNAL_INDEX);
        ADC_ClrEocFlag(M4_ADC2, ADC_SEQ_A);
#endif
        (void)m_u16AdcValue;
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  ADC configuration, including clock configuration, initial configuration,
 **         channel configuration and trigger source configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void AdcConfig(void)
{
    AdcClockConfig();
    AdcInitConfig();
    AdcChannelConfig();
}

/**
 *******************************************************************************
 ** \brief  ADC clock configuration.
 **
 ** \note   1) ADCLK max frequency is 60MHz.
 **         2) If PCLK2 and PCLK4 are selected as the ADC clock,
 **            the following conditions must be met:
 **            a. ADCLK(PCLK2) max 60MHz;
 **            b. PCLK4 : ADCLK = 1:1, 2:1, 4:1, 8:1, 1:2, 1:4
 **
 ******************************************************************************/
static void AdcClockConfig(void)
{
#if (ADC_CLK == ADC_CLK_PCLK)
    stc_clk_sysclk_cfg_t stcSysclkCfg;

    /* Set bus clock division, depends on the system clock frequency. */
    stcSysclkCfg.enHclkDiv  = ClkSysclkDiv1;  // 200MHz
    stcSysclkCfg.enExclkDiv = ClkSysclkDiv2;  // 100MHz
    stcSysclkCfg.enPclk0Div = ClkSysclkDiv1;  // 200MHz
    stcSysclkCfg.enPclk1Div = ClkSysclkDiv2;  // 100MHz
    stcSysclkCfg.enPclk2Div = ClkSysclkDiv4;  // 50MHz
    stcSysclkCfg.enPclk3Div = ClkSysclkDiv4;  // 50MHz
    stcSysclkCfg.enPclk4Div = ClkSysclkDiv2;  // 100MHz.
    CLK_SysClkConfig(&stcSysclkCfg);
    CLK_SetPeriClkSource(ClkPeriSrcPclk);

#elif (ADC_CLK == ADC_CLK_MPLLQ)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_mpll_cfg_t stcMpllCfg;

    if (CLKSysSrcMPLL == CLK_GetSysClkSource())
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
        stcMpllCfg.PllpDiv = 16u;
        stcMpllCfg.PllqDiv = 16u;
        stcMpllCfg.PllrDiv = 16u;
        CLK_SetPllSource(ClkPllSrcXTAL);
        CLK_MpllConfig(&stcMpllCfg);
        CLK_MpllCmd(Enable);
    }
    CLK_SetPeriClkSource(ClkPeriSrcMpllp);

#elif (ADC_CLK == ADC_CLK_UPLLR)
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
    stcUpllCfg.PllpDiv = 16u;
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
 ** \brief  ADC initial configuration.
 **
 ******************************************************************************/
static void AdcInitConfig(void)
{
    stc_adc_init_t stcAdcInit;

    MEM_ZERO_STRUCT(stcAdcInit);

    stcAdcInit.enResolution = AdcResolution_8Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnce;

#if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
    /* 1. Enable ADC1. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);
    /* 2. Initialize ADC1. */
    ADC_Init(M4_ADC1, &stcAdcInit);
#else
    /* 1. Enable ADC2. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC2, Enable);
    /* 2. Initialize ADC2. */
    ADC_Init(M4_ADC2, &stcAdcInit);
#endif
}

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ******************************************************************************/
static void AdcChannelConfig(void)
{
    stc_adc_ch_cfg_t stcChCfg;
#if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
    uint8_t au8AdcSampTime[ADC1_SA_CHANNEL_COUNT] = ADC1_SA_CHANNEL_SAMPLE_TIME;
#else
    uint8_t au8AdcSampTime[ADC2_SA_CHANNEL_COUNT] = ADC2_SA_CHANNEL_SAMPLE_TIME;
#endif

    MEM_ZERO_STRUCT(stcChCfg);

#if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
    stcChCfg.u32Channel  = ADC1_SA_CHANNEL;
    stcChCfg.u8Sequence  = ADC_SEQ_A;
    stcChCfg.pu8SampTime = au8AdcSampTime;
    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

    #if (ADC1_INTERNAL_SRC == DAC1_TO_ADC1)
        /* DAC1 to ADC1 */
        PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
        CMP_ADC_SetRefVoltPath(CmpAdcRefVoltPathDac1);
        CMP_DAC_SetData(CmpDac1, 0x9Fu);
        CMP_DAC_Cmd(CmpDac1, Enable);
    #endif

    #if (ADC1_INTERNAL_SRC == DAC2_TO_ADC1)
        /* DAC2 to ADC1 */
        PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
        CMP_ADC_SetRefVoltPath(CmpAdcRefVoltPathDac2);
        CMP_DAC_SetData(CmpDac2, 0x88u);
        CMP_DAC_Cmd(CmpDac2, Enable);
    #endif

    #if (ADC1_INTERNAL_SRC == VREF_TO_ADC1)
        /* VREF to ADC1 */
        PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
        CMP_ADC_SetRefVoltPath(CmpAdcRefVoltPathVref);
        PWC_PwrMonitorCmd(Enable);
    #endif

#else // #if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
    stcChCfg.u32Channel  = ADC2_SA_CHANNEL;
    stcChCfg.u8Sequence  = ADC_SEQ_A;
    stcChCfg.pu8SampTime = au8AdcSampTime;
    ADC_AddAdcChannel(M4_ADC2, &stcChCfg);

    #if (ADC2_INTERNAL_SRC == DAC1_TO_ADC2)
        /* DAC1 to ADC2 */
        PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
        CMP_ADC_SetRefVoltPath(CmpAdcRefVoltPathDac1);
        CMP_DAC_SetData(CmpDac1, 0x9Fu);
        CMP_DAC_Cmd(CmpDac1, Enable);
    #endif

    #if (ADC2_INTERNAL_SRC == DAC2_TO_ADC2)
        /* DAC2 to ADC2 */
        PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
        CMP_ADC_SetRefVoltPath(CmpAdcRefVoltPathDac2);
        CMP_DAC_SetData(CmpDac2, 0x88u);
        CMP_DAC_Cmd(CmpDac2, Enable);
    #endif

    #if (ADC2_INTERNAL_SRC == VREF_TO_ADC2)
        /* VREF to ADC2 */
        PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
        CMP_ADC_SetRefVoltPath(CmpAdcRefVoltPathVref);
        PWC_PwrMonitorCmd(Enable);
    #endif
#endif  // #if (USE_INTERNAL_CH == USE_ADC1_INTERNAL)
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
