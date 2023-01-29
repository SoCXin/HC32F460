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
 ** \brief The example of Rtc alarm clock function
 **
 **   - 2018-11-26  CDT  First version for Device Driver Library of Rtc.
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
static uint8_t u8AlarmIntFlag = 0u, u8AlarmCnt = 0u;

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
 ** \brief Rtc alarm clock callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void RtcAlarm_IrqCallback(void)
{
    u8AlarmCnt = 10u;
    u8AlarmIntFlag = 1u;
    RTC_ClearAlarmFlag();
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
    stcRtcDateTimeCfg.u8Hour = 11u;
    stcRtcDateTimeCfg.u8Minute = 59u;
    stcRtcDateTimeCfg.u8Second = 55u;
    stcRtcDateTimeCfg.enAmPm = RtcHour12Pm;
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
 ** \brief Rtc display date and time information
 **
 ** \param [in] enFormat                Date and time data format
 ** \arg RtcDataFormatDec               Decimal format
 ** \arg RtcDataFormatBcd               BCD format
 **
 ** \param [in] pTitle                  Display title of information
 **
 ** \param [in] pRtcDateTime            Pointer to RTC current date time struct
 ** \arg See the struct #stc_rtc_date_time_t
 **
 ** \retval None
 **
 ******************************************************************************/
static void Rtc_DisplayDataTimeInfo(en_rtc_data_format_t enFormat, char *pTitle,
                             const stc_rtc_date_time_t *pRtcDateTime)
{
    if (RtcDataFormatBcd == enFormat)
    {
        DDL_Printf("%s 20%02x/%02x/%02x %02x:%02x:%02x ", pTitle, pRtcDateTime->u8Year,
               pRtcDateTime->u8Month, pRtcDateTime->u8Day, pRtcDateTime->u8Hour,
               pRtcDateTime->u8Minute, pRtcDateTime->u8Second);
    }
    else
    {
        DDL_Printf("%s 20%02d/%02d/%02d %02d:%02d:%02d ", pTitle, pRtcDateTime->u8Year,
               pRtcDateTime->u8Month, pRtcDateTime->u8Day, pRtcDateTime->u8Hour,
               pRtcDateTime->u8Minute, pRtcDateTime->u8Second);
    }

    if (pRtcDateTime->enAmPm == RtcHour12Am)
    {
        DDL_Printf("Am ");
    }
    else
    {
        DDL_Printf("Pm ");
    }
    Rtc_DisplayWeekday(pRtcDateTime->u8Weekday);
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
    stc_rtc_alarm_time_t stcRtcAlarmCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcRtcInit);
    MEM_ZERO_STRUCT(stcRtcAlarmCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

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
        stcRtcInit.enTimeFormat = RtcTimeFormat12Hour;
        stcRtcInit.enCompenWay = RtcOutputCompenDistributed;
        stcRtcInit.enCompenEn = Disable;
        stcRtcInit.u16CompenVal = 0u;
        RTC_Init(&stcRtcInit);

        /* Configuration alarm clock time: Monday to friday，PM 12:00 */
        stcRtcAlarmCfg.u8Hour = 0x12u;
        stcRtcAlarmCfg.u8Minute = 0x00u;
        stcRtcAlarmCfg.u8Weekday = RtcAlarmWeekdayMonday    | RtcAlarmWeekdayTuesday  |
                                   RtcAlarmWeekdayWednesday | RtcAlarmWeekdayThursday |
                                   RtcAlarmWeekdayFriday;
        stcRtcAlarmCfg.enAmPm = RtcHour12Am;
        RTC_SetAlarmTime(RtcDataFormatBcd, &stcRtcAlarmCfg);
        RTC_AlarmCmd(Enable);

        /* Configure interrupt of rtc period */
        stcIrqRegiConf.enIntSrc = INT_RTC_PRD;
        stcIrqRegiConf.enIRQn = Int006_IRQn;
        stcIrqRegiConf.pfnCallback = &RtcPeriod_IrqCallback;
        enIrqRegistration(&stcIrqRegiConf);
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        /* Configuration interrupt of rtc alarm clock */
        stcIrqRegiConf.enIntSrc = INT_RTC_ALM;
        stcIrqRegiConf.enIRQn = Int007_IRQn;
        stcIrqRegiConf.pfnCallback = &RtcAlarm_IrqCallback;
        enIrqRegistration(&stcIrqRegiConf);
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        /* Enable period and alarm interrupt */
        RTC_IrqCmd(RtcIrqPeriod, Enable);
        RTC_IrqCmd(RtcIrqAlarm, Enable);

        /* Startup rtc count */
        RTC_Cmd(Enable);
    }
}

/**
 *******************************************************************************
 ** \brief  main function for Rtc alarm clock function
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
    /* wait for rtc running */
    Ddl_Delay1ms(1u);
    /* Update time after RTC startup */
    Rtc_CalendarConfig();

    while (1)
    {
        if (1u == u8SecIntFlag)
        {
            u8SecIntFlag = 0u;
            /* Print alarm information */
            if ((1u == u8AlarmIntFlag) && (u8AlarmCnt > 0u))
            {
                /* Alarm LED flicker */
                BSP_LED_Toggle(LED_RED);
                u8AlarmCnt--;
                if (0u == u8AlarmCnt)
                {
                    u8AlarmIntFlag = 0u;
                    BSP_LED_Off(LED_RED);
                }
                /* Get and print alarm time */
                if (RTC_GetDateTime(RtcDataFormatBcd, &stcCurrDateTime) != Ok)
                {
                    DDL_Printf("get alarm clock time failed!\r\n");
                }
                else
                {
                    Rtc_DisplayDataTimeInfo(RtcDataFormatBcd, "alarm", &stcCurrDateTime);
                }
            }
            /* Print current date and time information */
            else
            {
                /* Get current time */
                if (RTC_GetDateTime(RtcDataFormatDec, &stcCurrDateTime) != Ok)
                {
                    DDL_Printf("get calendar failed!\r\n");
                }
                else
                {
                    Rtc_DisplayDataTimeInfo(RtcDataFormatDec, "normal", &stcCurrDateTime);
                }
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
