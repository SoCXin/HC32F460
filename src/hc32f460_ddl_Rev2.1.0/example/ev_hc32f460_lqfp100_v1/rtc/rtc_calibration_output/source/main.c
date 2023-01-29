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
 ** \brief The example of Rtc calibration output function
 **
 **   - 2018-11-28  CDT  First version for Device Driver Library of Rtc.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include <math.h>

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* RTC 1Hz output Port/Pin definition */
#define RTC_ONEHZ_OUTPUT_PORT           (PortC)
#define RTC_ONEHZ_OUTPUT_PIN            (Pin13)

/* XTAL32 measure window lower and upper definition */
#define XTAL32_MEASURE_LOWER            20000u
#define XTAL32_MEASURE_UPPER            40000u

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const float EPSINON = 0.000001f;
static uint8_t u8SecIntFlag = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Rtc period callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void RtcPeriod_IrqCallback(void)
{
    u8SecIntFlag = 1u;
}

/**
 *******************************************************************************
 ** \brief Xtal32 clock config
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Xtal32_ClockConfig(void)
{
    stc_clk_xtal32_cfg_t stcXtal32Cfg;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcXtal32Cfg);

    /* Stop xtal32 */
    CLK_Xtal32Cmd(Disable);
    Ddl_Delay1ms(100u);
    /* Configuration xtal32 structure */
    stcXtal32Cfg.enDrv = ClkXtal32HighDrv;
    stcXtal32Cfg.enFilterMode = ClkXtal32FilterModeFull;
    CLK_Xtal32Config(&stcXtal32Cfg);
    /* Startup xtal32 */
    CLK_Xtal32Cmd(Enable);
    /* wait for xtal32 running */
    Ddl_Delay1ms(3000u);
}

/**
 *******************************************************************************
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void SystemClk_Init(void)
{
    stc_clk_xtal_cfg_t stcXtalCfg;

    MEM_ZERO_STRUCT(stcXtalCfg);

    /* Switch system clock source to XTAL. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Wait XTAL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagXTALRdy))
    {
    }
    /* Switch system clock source to XTAL. */
    CLK_SetSysClkSource(ClkSysSrcXTAL);
}

/**
 *******************************************************************************
 ** \brief clock measure configuration
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Clock_MeasureConfig(void)
{
    stc_clk_fcm_cfg_t stcClkFcmCfg;
    stc_clk_fcm_window_cfg_t stcClkFcmWinCfg;
    stc_clk_fcm_measure_cfg_t stcClkFcmMeasureCfg;
    stc_clk_fcm_reference_cfg_t stcClkFcmReferCfg;
    stc_clk_fcm_interrupt_cfg_t stcClkFcmIntCfg;

    /* Configure structure initialization */
    MEM_ZERO_STRUCT(stcClkFcmCfg);
    MEM_ZERO_STRUCT(stcClkFcmWinCfg);
    MEM_ZERO_STRUCT(stcClkFcmMeasureCfg);
    MEM_ZERO_STRUCT(stcClkFcmReferCfg);
    MEM_ZERO_STRUCT(stcClkFcmIntCfg);

    /* Enable FCM clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_FCM, Enable);

    /* use xtal measure xtal32 */
    stcClkFcmWinCfg.windowLower = XTAL32_MEASURE_LOWER; /* zero error = 48828 */
    stcClkFcmWinCfg.windowUpper = XTAL32_MEASURE_UPPER;

    stcClkFcmMeasureCfg.enSrc = ClkFcmSrcXtal;
    stcClkFcmMeasureCfg.enSrcDiv = ClkFcmMeaDiv1;

    stcClkFcmReferCfg.enRefSel = ClkFcmInterRef;
    stcClkFcmReferCfg.enExtRef = Disable;
    stcClkFcmReferCfg.enIntRefSrc = ClkFcmSrcXtal32;
    stcClkFcmReferCfg.enIntRefDiv = ClkFcmIntrefDiv128;
    stcClkFcmReferCfg.enEdge = ClkFcmEdgeRising;
    stcClkFcmReferCfg.enFilterClk = ClkFcmFilterClkNone;

    stcClkFcmCfg.pstcFcmIntCfg = &stcClkFcmIntCfg;
    stcClkFcmCfg.pstcFcmMeaCfg = &stcClkFcmMeasureCfg;
    stcClkFcmCfg.pstcFcmRefCfg = &stcClkFcmReferCfg;
    stcClkFcmCfg.pstcFcmWindowCfg = &stcClkFcmWinCfg;
    /* enable clock measure */
    CLK_FcmConfig(&stcClkFcmCfg);
}

/**
 *******************************************************************************
 ** \brief Get rtc compensation value
 **
 ** \param [in]  None
 **
 ** \retval uint16_t                            Rtc compensation value
 **
 ******************************************************************************/
static uint16_t Rtc_GetCompenValue(void)
{
    uint32_t u32Tmp = 0ul;
    en_flag_status_t enStaTmp;
    float clkMeasureVal;
    uint16_t integerVal = 0u, decimalsVal = 0u;
    uint16_t clkCompenVal = 0u;
    stc_clk_freq_t stcClkFreq;

    MEM_ZERO_STRUCT(stcClkFreq);
    /* start measure */
    CLK_FcmCmd(Enable);
    do
    {
        enStaTmp = CLK_GetFcmFlag(ClkFcmFlagErrf);
        /* counter overflow or trigger frequency abnormal */
        if ((Set == CLK_GetFcmFlag(ClkFcmFlagOvf)) || (Set == enStaTmp))
        {
            CLK_FcmCmd(Disable);
            CLK_ClearFcmFlag(ClkFcmFlagOvf);
            CLK_ClearFcmFlag(ClkFcmFlagErrf);
            u32Tmp = 0xffu;
        }
    } while (Reset == CLK_GetFcmFlag(ClkFcmFlagMendf));

    if (0xffu != u32Tmp)
    {
        /* Get measure result */
        CLK_GetClockFreq(&stcClkFreq);
        u32Tmp = CLK_GetFcmCounter();
        clkMeasureVal = ((float)stcClkFreq.sysclkFreq * 128.0f) / (float)u32Tmp;
        /* stop measure */
        CLK_FcmCmd(Disable);
        CLK_ClearFcmFlag(ClkFcmFlagMendf);

        /* calculate clock compensation value */
        if (!((clkMeasureVal >= -EPSINON) && (clkMeasureVal <= EPSINON)))
        {
            clkMeasureVal = (clkMeasureVal - (float)XTAL32_VALUE) / (float)XTAL32_VALUE * (float)1000000.0f;
            clkMeasureVal = clkMeasureVal * (float)XTAL32_VALUE / 1000000.0f;

            if (clkMeasureVal < -EPSINON)    /* negative */
            {
                clkMeasureVal = (float)fabs((double)clkMeasureVal);
                integerVal = (uint16_t)(((~((uint32_t)clkMeasureVal)) + 1u) & 0x0Fu);
                /* Magnify one thousand times */
                u32Tmp = (uint32_t)clkMeasureVal;
                clkMeasureVal = (clkMeasureVal - (float)u32Tmp) * 1000.0f;
                decimalsVal = (uint16_t)((((~((uint32_t)clkMeasureVal)) & 0x3E0u) >> 5u) + 1u);
            }
            else                            /* positive */
            {
                clkMeasureVal += 1.0f;
                integerVal = (uint16_t)(((uint32_t)clkMeasureVal) & 0x0Fu);
                /* Magnify one thousand times */
                u32Tmp = (uint32_t)clkMeasureVal;
                clkMeasureVal = (float)((clkMeasureVal - (float)u32Tmp) * 1000.0f);
                decimalsVal = (uint16_t)(((uint32_t)clkMeasureVal & 0x3E0u) >> 5u);
            }
        }
        clkCompenVal = ((uint16_t)(integerVal << 5u) | decimalsVal) & 0x1FFu;
    }

    return clkCompenVal;
}

/**
 *******************************************************************************
 ** \brief Configure Rtc peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Rtc_Config(void)
{
    stc_rtc_init_t stcRtcInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    uint16_t clkCompenVal = 0u;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcRtcInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configuration RTC output pin */
    PORT_SetFunc(RTC_ONEHZ_OUTPUT_PORT, RTC_ONEHZ_OUTPUT_PIN, Func_Rtcout, Disable);

    /* Reset rtc counter */
    if (RTC_DeInit() == ErrorTimeout)
    {
        DDL_Printf("reset rtc failed!\r\n");
    }
    else
    {
        clkCompenVal = Rtc_GetCompenValue();
        /* Configuration rtc structure */
        stcRtcInit.enClkSource = RtcClkXtal32;
        stcRtcInit.enPeriodInt = RtcPeriodIntOneSec;
        stcRtcInit.enTimeFormat = RtcTimeFormat24Hour;
        stcRtcInit.enCompenWay = RtcOutputCompenUniform;
        stcRtcInit.enCompenEn = Enable;
        stcRtcInit.u16CompenVal = clkCompenVal;
        RTC_Init(&stcRtcInit);
        RTC_OneHzOutputCmd(Enable);

        /* Configure interrupt of rtc period */
        stcIrqRegiConf.enIntSrc = INT_RTC_PRD;
        stcIrqRegiConf.enIRQn = Int006_IRQn;
        stcIrqRegiConf.pfnCallback = &RtcPeriod_IrqCallback;
        enIrqRegistration(&stcIrqRegiConf);
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        /* Enable period interrupt */
        RTC_IrqCmd(RtcIrqPeriod, Enable);
        /* Startup rtc count */
        RTC_Cmd(Enable);
    }
}

/**
 *******************************************************************************
 ** \brief  main function for Rtc calibration output function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configure XTAL32 clock */
    Xtal32_ClockConfig();
    /* Configure system clock frequency */
    SystemClk_Init();
    /* LED initialization */
    BSP_LED_Init();
    /* Debug uart init */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);
    /* Configure clock measure */
    Clock_MeasureConfig();
    /* Configure Rtc */
    Rtc_Config();

    while (1)
    {
        if (1u == u8SecIntFlag)
        {
            u8SecIntFlag = 0u;
            BSP_LED_Toggle(LED_RED);
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
