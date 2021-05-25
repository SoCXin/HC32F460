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
 ** \brief The example of Rtc calendar function
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of Rtc.
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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
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
 ** \brief Configure Rtc calendar function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Rtc_CalendarConfig(void)
{
    stc_rtc_date_time_t stcRtcDateTimeCfg;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcRtcDateTimeCfg);

    /* calendar configuration */
    stcRtcDateTimeCfg.u8Year = 18u;
    stcRtcDateTimeCfg.u8Month = 10u;
    stcRtcDateTimeCfg.u8Day = 10u;
    stcRtcDateTimeCfg.u8Weekday = RtcWeekdayWednesday;
    stcRtcDateTimeCfg.u8Hour = 23u;
    stcRtcDateTimeCfg.u8Minute = 59u;
    stcRtcDateTimeCfg.u8Second = 55u;
    if (RTC_SetDateTime(RtcDataFormatDec, &stcRtcDateTimeCfg, Enable, Enable) != Ok)
    {
        DDL_Printf("write calendar failed!\r\n");
    }
}

/**
 *******************************************************************************
 ** \brief Rtc display weekday
 **
 ** \param [in] u8Weekday               week day
 **
 ** \retval None
 **
 ******************************************************************************/
static void Rtc_DisplayWeekday(uint8_t u8Weekday)
{
    switch (u8Weekday)
    {
        case RtcWeekdaySunday:
            DDL_Printf("Sunday\r\n");
            break;
        case RtcWeekdayMonday:
            DDL_Printf("Monday\r\n");
            break;
        case RtcWeekdayTuesday:
            DDL_Printf("Tuesday\r\n");
            break;
        case RtcWeekdayWednesday:
            DDL_Printf("Wednesday\r\n");
            break;
        case RtcWeekdayThursday:
            DDL_Printf("Thursday\r\n");
            break;
        case RtcWeekdayFriday:
            DDL_Printf("Friday\r\n");
            break;
        case RtcWeekdaySaturday:
            DDL_Printf("Saturday\r\n");
            break;
        default:
            break;
    }
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

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcRtcInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configure interrupt of rtc period */
    stcIrqRegiConf.enIntSrc = INT_RTC_PRD;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &RtcPeriod_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Rtc stopped */
    if (0u == M4_RTC->CR1_f.START)
    {
        /* Reset rtc counter */
        if (RTC_DeInit() == ErrorTimeout)
        {
            DDL_Printf("reset rtc failed!\r\n");
        }
        else
        {
            /* Configuration rtc structure */
            stcRtcInit.enClkSource = RtcClkXtal32;
            stcRtcInit.enPeriodInt = RtcPeriodIntOneSec;
            stcRtcInit.enTimeFormat = RtcTimeFormat24Hour;
            stcRtcInit.enCompenWay = RtcOutputCompenDistributed;
            stcRtcInit.enCompenEn = Disable;
            stcRtcInit.u16CompenVal = 0u;
            RTC_Init(&stcRtcInit);

            /* Enable period interrupt */
            RTC_IrqCmd(RtcIrqPeriod, Enable);
            /* Startup rtc count */
            RTC_Cmd(Enable);

            /* wait for rtc running */
            Ddl_Delay1ms(1u);
            /* Update time after RTC startup */
            Rtc_CalendarConfig();
        }
    }
}

/**
 *******************************************************************************
 ** \brief  main function for Rtc calendar function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_rtc_date_time_t stcCurrDateTime;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcCurrDateTime);

    /* LED initialization */
    BSP_LED_Init();
    /* Configure XTAL32 clock */
    Xtal32_ClockConfig();
    /* Debug uart init */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);
    /* Configure Rtc */
    Rtc_Config();

    while (1)
    {
        if (1u == u8SecIntFlag)
        {
            u8SecIntFlag = 0u;
            BSP_LED_Toggle(LED_RED);
            /* Get current time */
            if (RTC_GetDateTime(RtcDataFormatDec, &stcCurrDateTime) != Ok)
            {
                DDL_Printf("get calendar failed!\r\n");
            }
            else
            {
                DDL_Printf("20%02d/%02d/%02d %02d:%02d:%02d ", stcCurrDateTime.u8Year,
                       stcCurrDateTime.u8Month, stcCurrDateTime.u8Day,
                       stcCurrDateTime.u8Hour, stcCurrDateTime.u8Minute,
                       stcCurrDateTime.u8Second);
                Rtc_DisplayWeekday(stcCurrDateTime.u8Weekday);
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
