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
/*
 * If you remap the mapping between the channel and the pin with the function
 * ADC_ChannelRemap, define ADC_CH_REMAP as non-zero, otherwise define as 0.
 */
#define ADC_CH_REMAP                (0u)

/* ADC clock selection definition. */
#define ADC_CLK_PCLK                (1u)
#define ADC_CLK_MPLLQ               (2u)
#define ADC_CLK_UPLLR               (3u)

/* Select MPLLQ as ADC clock. */
#define ADC_CLK                     (ADC_CLK_MPLLQ)

/* ADC1 channel definition for this example. */
#define ADC1_SA_NORMAL_CHANNEL      (ADC1_CH0 | ADC1_CH1)
#define ADC1_SA_AWD_CHANNEL         (ADC1_CH4 | ADC1_CH5)
#define ADC1_SA_CHANNEL             (ADC1_SA_NORMAL_CHANNEL | ADC1_SA_AWD_CHANNEL)
#define ADC1_SA_CHANNEL_COUNT       (4u)

#define ADC1_SB_NORMAL_CHANNEL      (ADC1_CH2 | ADC1_CH3)
#define ADC1_SB_AWD_CHANNEL         (ADC1_CH6)
#define ADC1_SB_CHANNEL             (ADC1_SB_NORMAL_CHANNEL | ADC1_SB_AWD_CHANNEL)
#define ADC1_SB_CHANNEL_COUNT       (3u)

#define ADC1_AWD_CHANNEL            (ADC1_SA_AWD_CHANNEL | ADC1_SB_AWD_CHANNEL)
#define ADC1_CHANNEL                (ADC1_SA_CHANNEL | ADC1_SB_CHANNEL)

/* ADC1 channel sampling time.      ADC1_CH0  ADC1_CH1  ADC1_CH4  ADC1_CH5 */
#define ADC1_SA_CHANNEL_SAMPLE_TIME { 0x30,    0x40,     0x50,     0x60 }

/* ADC1 channel sampling time.      ADC1_CH2  ADC1_CH3  ADC1_CH6 */
#define ADC1_SB_CHANNEL_SAMPLE_TIME { 0x50,    0x60,     0x45 }

/* ADC2 channel definition for this example. */
#define ADC2_SA_NORMAL_CHANNEL      (ADC2_CH0)
#define ADC2_SA_AWD_CHANNEL         (ADC2_CH5)
#define ADC2_SA_CHANNEL             (ADC2_SA_NORMAL_CHANNEL | ADC2_SA_AWD_CHANNEL)
#define ADC2_SA_CHANNEL_COUNT       (2u)

#define ADC2_SB_NORMAL_CHANNEL      (ADC2_CH2 | ADC2_CH4)
#define ADC2_SB_AWD_CHANNEL         (ADC2_CH3 | ADC2_CH7)
#define ADC2_SB_CHANNEL             (ADC2_SB_NORMAL_CHANNEL | ADC2_SB_AWD_CHANNEL)
#define ADC2_SB_CHANNEL_COUNT       (4u)

#define ADC2_AWD_CHANNEL            (ADC2_SA_AWD_CHANNEL | ADC2_SB_AWD_CHANNEL)
#define ADC2_CHANNEL                (ADC2_SA_CHANNEL | ADC2_SB_CHANNEL)

/* ADC2 channel sampling time.      ADC2_CH0  ADC2_CH5 */
#define ADC2_SA_CHANNEL_SAMPLE_TIME { 0x60,    0x50 }

/* ADC2 channel sampling time.      ADC2_CH2  ADC2_CH3  ADC2_CH4  ADC2_CH7 */
#define ADC2_SB_CHANNEL_SAMPLE_TIME { 0x60,    0x50,     0x60,     0x50 }

/* AWD range definition. */
#define AWD_LOWER_1                 (33u)
#define AWD_UPPER_1                 (666u)

#define AWD_LOWER_2                 (666u)
#define AWD_UPPER_2                 (777u)

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
static void AdcTriggerConfig(void);

static void AdcSetChannelPinMode(const M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode);
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

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
    uint32_t u32AwdChSet = 0u;

    /* Default clock is MRC(8MHz) */

    /* Config ADCs. */
    AdcConfig();

    /***************** Configuration end, application start **************/

    /* Start ADC1. */
    ADC_StartConvert(M4_ADC1);

    /* Check AWD of ADC1. */
    /* ADC1 sequence A. Waiting for sequence A conversion end. */
    while (Reset == ADC_GetEocFlag(M4_ADC1, ADC_SEQ_A))
    {
        ;
    }
    /* Clear EOC(End of Conversion) flag. */
    ADC_ClrEocFlag(M4_ADC1, ADC_SEQ_A);
    /* Get the data if needed. */
    /* Get the AWD channel flag. */
    u32AwdChSet = ADC_GetAwdFlag(M4_ADC1);
    /* Check the AWD channel if meet the comparison conditions. */
    if (u32AwdChSet & ADC1_SA_AWD_CHANNEL)
    {
        /* The AWD channles in sequence A that meet the comparison conditions. */
        ADC_ClrAwdChFlag(M4_ADC1, (u32AwdChSet & ADC1_SA_AWD_CHANNEL));
    }

    /* ADC1 sequence B. Waiting for sequence B conversion end. */
    while (Reset == ADC_GetEocFlag(M4_ADC1, ADC_SEQ_B))
    {
        ;
    }
    /* Clear EOC(End of Conversion) flag. */
    ADC_ClrEocFlag(M4_ADC1, ADC_SEQ_B);
    /* Get the AWD channel flag. */
    u32AwdChSet = ADC_GetAwdFlag(M4_ADC1);
    /* Check the AWD channel if meet the comparison conditions. */
    if (u32AwdChSet & ADC1_SB_AWD_CHANNEL)
    {
        /* The AWD channles in sequence B that meet the comparison conditions. */
        ADC_ClrAwdChFlag(M4_ADC1, (u32AwdChSet & ADC1_SB_AWD_CHANNEL));
    }

    /* Start ADC2. */
    ADC_StartConvert(M4_ADC2);
    /* Check AWD of ADC2. */
    /* ADC2 sequence A. Waiting for sequence A conversion end. */
    while (Reset == ADC_GetEocFlag(M4_ADC2, ADC_SEQ_A))
    {
        ;
    }
    ADC_ClrEocFlag(M4_ADC2, ADC_SEQ_A);
    u32AwdChSet = ADC_GetAwdFlag(M4_ADC2);
    if (u32AwdChSet & ADC2_SA_AWD_CHANNEL)
    {
        /* The AWD channles in sequence A that meet the comparison conditions. */
        ADC_ClrAwdChFlag(M4_ADC2, (u32AwdChSet & ADC2_SA_AWD_CHANNEL));
    }

    /* ADC2 sequence B. Waiting for sequence B conversion end. */
    while (Reset == ADC_GetEocFlag(M4_ADC2, ADC_SEQ_B))
    {
        ;
    }
    ADC_ClrEocFlag(M4_ADC2, ADC_SEQ_B);
    u32AwdChSet = ADC_GetAwdFlag(M4_ADC2);
    if (u32AwdChSet & ADC2_SB_AWD_CHANNEL)
    {
        /* The AWD channles in sequence B that meet the comparison conditions. */
        ADC_ClrAwdChFlag(M4_ADC2, (u32AwdChSet & ADC2_SB_AWD_CHANNEL));
    }

    while (1u)
    {
        // TODO: YOUR CODE
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
    AdcTriggerConfig();
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
    stcSysclkCfg.enPclk4Div = ClkSysclkDiv1;  // 100MHz.
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

    stcAdcInit.enResolution = AdcResolution_10Bit;
    stcAdcInit.enDataAlign  = AdcDataAlign_Right;
    stcAdcInit.enAutoClear  = AdcClren_Disable;
    stcAdcInit.enScanMode   = AdcMode_SAOnceSBOnce;
    stcAdcInit.enRschsel    = AdcRschsel_Continue;
    /* 1. Enable ADC1. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);
    /* 2. Initialize ADC1. */
    ADC_Init(M4_ADC1, &stcAdcInit);

    /* 1. Enable ADC2. */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC2, Enable);
    /* 2. Initialize ADC2. */
    ADC_Init(M4_ADC2, &stcAdcInit);
}

/**
 *******************************************************************************
 ** \brief  ADC channel configuration.
 **
 ******************************************************************************/
static void AdcChannelConfig(void)
{
    stc_adc_ch_cfg_t  stcChCfg;
    stc_adc_awd_cfg_t stcAwdCfg;
    uint8_t au8Adc1SaSampTime[ADC1_SA_CHANNEL_COUNT] = ADC1_SA_CHANNEL_SAMPLE_TIME;
    uint8_t au8Adc1SbSampTime[ADC1_SB_CHANNEL_COUNT] = ADC1_SB_CHANNEL_SAMPLE_TIME;
    uint8_t au8Adc2SaSampTime[ADC2_SA_CHANNEL_COUNT] = ADC2_SA_CHANNEL_SAMPLE_TIME;
    uint8_t au8Adc2SbSampTime[ADC2_SB_CHANNEL_COUNT] = ADC2_SB_CHANNEL_SAMPLE_TIME;

    MEM_ZERO_STRUCT(stcChCfg);
    MEM_ZERO_STRUCT(stcAwdCfg);

    /**************************** Add ADC1 channels ****************************/
    /* 1. Set the ADC pin to analog mode. */
    AdcSetChannelPinMode(M4_ADC1, ADC1_CHANNEL, Pin_Mode_Ana);

    stcChCfg.u32Channel  = ADC1_SA_CHANNEL;
    stcChCfg.u8Sequence  = ADC_SEQ_A;
    stcChCfg.pu8SampTime = au8Adc1SaSampTime;
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

    stcChCfg.u32Channel  = ADC1_SB_CHANNEL;
    stcChCfg.u8Sequence  = ADC_SEQ_B;
    stcChCfg.pu8SampTime = au8Adc1SbSampTime;
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC1, &stcChCfg);

    /* 3. Config AWD and add AWD channels. */
    stcAwdCfg.enAwdmd   = AdcAwdCmpMode_1;
    stcAwdCfg.enAwdss   = AdcAwdSel_SA;
    stcAwdCfg.u16AwdDr0 = AWD_LOWER_1;
    stcAwdCfg.u16AwdDr1 = AWD_UPPER_1;
    ADC_ConfigAwd(M4_ADC1, &stcAwdCfg);
    ADC_AddAwdChannel(M4_ADC1, ADC1_AWD_CHANNEL);
    ADC_AwdCmd(M4_ADC1, Enable);

    /**************************** Add ADC2 channels ****************************/
    /* 1. Set the ADC pin to analog mode. */
    AdcSetChannelPinMode(M4_ADC2, ADC2_CHANNEL, Pin_Mode_Ana);

    stcChCfg.u32Channel  = ADC2_SA_CHANNEL;
    stcChCfg.u8Sequence  = ADC_SEQ_A;
    stcChCfg.pu8SampTime = au8Adc2SaSampTime;
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC2, &stcChCfg);

    stcChCfg.u32Channel  = ADC2_SB_CHANNEL;
    stcChCfg.u8Sequence  = ADC_SEQ_B;
    stcChCfg.pu8SampTime = au8Adc2SbSampTime;
    /* 2. Add ADC channel. */
    ADC_AddAdcChannel(M4_ADC2, &stcChCfg);

    /* 3. Config AWD and add AWD channels. */
    ADC_ConfigAwd(M4_ADC2, &stcAwdCfg);
    ADC_AddAwdChannel(M4_ADC2, ADC2_AWD_CHANNEL);
    ADC_AwdCmd(M4_ADC2, Enable);
}

/**
 *******************************************************************************
 ** \brief  ADC trigger source configuration.
 **
 ******************************************************************************/
static void AdcTriggerConfig(void)
{
    stc_adc_trg_cfg_t stcTrgCfg;

    MEM_ZERO_STRUCT(stcTrgCfg);

    /*
     * If select an event(@ref en_event_src_t) to trigger ADC,
     * AOS must be enabled first.
     */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* ADC1 sequence A will be started by software. */
    ADC_TriggerSrcCmd(M4_ADC1, ADC_SEQ_A, Disable);

    /* ADC1 sequence A scan ends to trigger ADC1 sequence B. */
    stcTrgCfg.u8Sequence = ADC_SEQ_B;
    stcTrgCfg.enTrgSel   = AdcTrgsel_TRGX0;
    stcTrgCfg.enInTrg0   = EVT_ADC1_EOCA;
    ADC_ConfigTriggerSrc(M4_ADC1, &stcTrgCfg);
    ADC_TriggerSrcCmd(M4_ADC1, ADC_SEQ_B, Enable);

    /* ADC2 sequence A will be started by software. */
    ADC_TriggerSrcCmd(M4_ADC2, ADC_SEQ_A, Disable);

    /* ADC2 sequence A scan ends to trigger ADC2 sequence B. */
    stcTrgCfg.u8Sequence = ADC_SEQ_B;
    stcTrgCfg.enTrgSel   = AdcTrgsel_TRGX1;
    stcTrgCfg.enInTrg1   = EVT_ADC2_EOCA;
    ADC_ConfigTriggerSrc(M4_ADC2, &stcTrgCfg);
    ADC_TriggerSrcCmd(M4_ADC2, ADC_SEQ_B, Enable);
}

/**
 *******************************************************************************
 ** \brief  Config the pin which is mapping the channel to analog or digit mode.
 **
 ******************************************************************************/
static void AdcSetChannelPinMode(const M4_ADC_TypeDef *ADCx,
                                 uint32_t u32Channel,
                                 en_pin_mode_t enMode)
{
    uint8_t u8ChIndex;
#if (ADC_CH_REMAP)
    uint8_t u8AdcPin;
#else
    uint8_t u8ChOffset = 0u;
#endif

    if (M4_ADC1 == ADCx)
    {
        u32Channel &= ADC1_PIN_MASK_ALL;
    }
    else
    {
        u32Channel &= ADC2_PIN_MASK_ALL;
#if (!ADC_CH_REMAP)
        u8ChOffset = 4u;
#endif
    }

    u8ChIndex = 0u;
    while (0u != u32Channel)
    {
        if (u32Channel & 0x1ul)
        {
#if (ADC_CH_REMAP)
            u8AdcPin = ADC_GetChannelPinNum(ADCx, u8ChIndex);
            AdcSetPinMode(u8AdcPin, enMode);
#else
            AdcSetPinMode((u8ChIndex + u8ChOffset), enMode);
#endif
        }

        u32Channel >>= 1u;
        u8ChIndex++;
    }
}

/**
 *******************************************************************************
 ** \brief  Set an ADC pin as analog input mode or digit mode.
 **
 ******************************************************************************/
static void AdcSetPinMode(uint8_t u8AdcPin, en_pin_mode_t enMode)
{
    en_port_t enPort = PortA;
    en_pin_t enPin   = Pin00;
    bool bFlag       = true;
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = enMode;
    stcPortInit.enPullUp  = Disable;

    switch (u8AdcPin)
    {
    case ADC1_IN0:
        enPort = PortA;
        enPin  = Pin00;
        break;

    case ADC1_IN1:
        enPort = PortA;
        enPin  = Pin01;
        break;

    case ADC1_IN2:
        enPort = PortA;
        enPin  = Pin02;
        break;

    case ADC1_IN3:
        enPort = PortA;
        enPin  = Pin03;
        break;

    case ADC12_IN4:
        enPort = PortA;
        enPin  = Pin04;
        break;

    case ADC12_IN5:
        enPort = PortA;
        enPin  = Pin05;
        break;

    case ADC12_IN6:
        enPort = PortA;
        enPin  = Pin06;
        break;

    case ADC12_IN7:
        enPort = PortA;
        enPin  = Pin07;
        break;

    case ADC12_IN8:
        enPort = PortB;
        enPin  = Pin00;
        break;

    case ADC12_IN9:
        enPort = PortB;
        enPin  = Pin01;
        break;

    case ADC12_IN10:
        enPort = PortC;
        enPin  = Pin00;
        break;

    case ADC12_IN11:
        enPort = PortC;
        enPin  = Pin01;
        break;

    case ADC1_IN12:
        enPort = PortC;
        enPin  = Pin02;
        break;

    case ADC1_IN13:
        enPort = PortC;
        enPin  = Pin03;
        break;

    case ADC1_IN14:
        enPort = PortC;
        enPin  = Pin04;
        break;

    case ADC1_IN15:
        enPort = PortC;
        enPin  = Pin05;
        break;

    default:
        bFlag = false;
        break;
    }

    if (true == bFlag)
    {
        PORT_Init(enPort, enPin, &stcPortInit);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
