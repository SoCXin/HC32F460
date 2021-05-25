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
 ** \brief The example of Timera compare function
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of
 **                      Timera.
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
/* KEY10 Port/Pin definition */
#define KEY10_PORT                      (PortB)
#define KEY10_PIN                       (Pin01)
#define KEY10_TRIGGER_EVENT             (EVT_PORT_EIRQ1)

/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    (M4_TMRA1)
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TIMERA_UNIT1_OVERFLOW_INT       (INT_TMRA1_OVF)

/* TIMERA channel 1 Port/Pin definition */
#define TIMERA_UNIT1_CH1                (TimeraCh1)
#define TIMERA_UNIT1_CH1_PORT           (PortA)
#define TIMERA_UNIT1_CH1_PIN            (Pin08)
#define TIMERA_UNIT1_CH1_FUNC           (Func_Tima0)

/* TIMERA channel 3 Port/Pin definition */
#define TIMERA_UNIT1_CH3                (TimeraCh3)
#define TIMERA_UNIT1_CH3_PORT           (PortE)
#define TIMERA_UNIT1_CH3_PIN            (Pin13)
#define TIMERA_UNIT1_CH3_FUNC           (Func_Tima0)

#define TIMERA_COUNT_OVERFLOW           (SystemCoreClock/2U/128U/100U/2)  // 100Hz

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8TmraUnit1Cnt = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Timera unit 1 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void TimeraUnit1_IrqCallback(void)
{
    u8TmraUnit1Cnt++;
    if (u8TmraUnit1Cnt >= 100u)      //1s
    {
        u8TmraUnit1Cnt = 0u;
        BSP_LED_Toggle(LED_RED);
    }
    TIMERA_ClearFlag(TIMERA_UNIT1, TimeraFlagOverflow);
}

/**
 *******************************************************************************
 ** \brief Configure Timera peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Timera_Config(void)
{
    stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_hw_startup_config_t stcTimeraHwConfig;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    MEM_ZERO_STRUCT(stcTimeraHwConfig);
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Configuration TIMERA compare pin */
    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH3_PORT, TIMERA_UNIT1_CH3_PIN, TIMERA_UNIT1_CH3_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv128;
    stcTimeraInit.enCntMode = TimeraCountModeTriangularWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW;
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);

    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = stcTimeraInit.u16PeriodVal * 4u / 5u;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputReverse;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputKeep;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
    stcTimerCompareInit.enCacheEn = Enable;
    stcTimerCompareInit.enTriangularTroughTransEn = Enable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
    TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH1, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH1, Enable);

    /* Configure channel 3 */
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputHigh;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputHigh;
    TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH3, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH3, Enable);

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure timera unit 1 startup */
    stcTimeraHwConfig.enSpecifyEventStartupEn = Enable;
    stcTimeraHwConfig.enTrigFallingStartupEn = Disable;
    stcTimeraHwConfig.enTrigRisingStartupEn = Disable;
    TIMERA_HwStartupConfig(TIMERA_UNIT1, &stcTimeraHwConfig);

    /* Set external Int trigger timera startup */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY10_PORT, KEY10_PIN, &stcPortInit);
    TIMERA_SetCountTriggerSrc(KEY10_TRIGGER_EVENT);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera compare function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint16_t u16TimerPeriod = 0u, u16DutyCycle = 0u;

    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Configure Timera */
    Timera_Config();
    u16DutyCycle = TIMERA_GetCompareValue(TIMERA_UNIT1, TIMERA_UNIT1_CH1);
    u16TimerPeriod = TIMERA_GetPeriodValue(TIMERA_UNIT1);

    while (1)
    {
        if (Set == BSP_KEY_GetStatus(BSP_KEY_1))
        {
            u16DutyCycle += u16TimerPeriod / 20u;
            if (u16DutyCycle > u16TimerPeriod)
            {
                u16DutyCycle = 0u;
            }
            TIMERA_SetCacheValue(TIMERA_UNIT1, TIMERA_UNIT1_CH1, u16DutyCycle);
            TIMERA_SetCacheValue(TIMERA_UNIT1, TIMERA_UNIT1_CH3, u16DutyCycle);
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
