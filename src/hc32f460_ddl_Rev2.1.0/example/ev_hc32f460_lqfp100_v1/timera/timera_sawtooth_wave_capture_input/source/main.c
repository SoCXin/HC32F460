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
 ** \brief The example of Timera capture function
 **
 **   - 2018-11-13  CDT  First version for Device Driver Library of
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
/* KEY2 Port/Pin definition */
#define KEY2_TRIGGER_EVENT              (EVT_PORT_EIRQ3)

/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    (M4_TMRA1)
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TIMERA_UNIT1_COMPARE_INT        (INT_TMRA1_CMP)

/* TIMERA channel 1 Port/Pin definition */
#define TIMERA_UNIT1_CH1                (TimeraCh1)
#define TIMERA_UNIT1_CH1_PORT           (PortE)
#define TIMERA_UNIT1_CH1_PIN            (Pin09)
#define TIMERA_UNIT1_CH1_FUNC           (Func_Tima0)
#define TIMERA_UNIT1_CH1_INT_FLAG       (TimeraFlagCaptureOrCompareCh1)
#define TIMERA_UNIT1_CH1_INT            (TimeraIrqCaptureOrCompareCh1)

/* TIMERA channel 2 Port/Pin definition */
#define TIMERA_UNIT1_CH2                (TimeraCh2)
#define TIMERA_UNIT1_CH2_PORT           (PortE)
#define TIMERA_UNIT1_CH2_PIN            (Pin11)
#define TIMERA_UNIT1_CH2_FUNC           (Func_Tima0)
#define TIMERA_UNIT1_CH2_INT_FLAG       (TimeraFlagCaptureOrCompareCh2)
#define TIMERA_UNIT1_CH2_INT            (TimeraIrqCaptureOrCompareCh2)

#define TIMERA_COUNT_OVERFLOW           (SystemCoreClock/2U/256U/10U)  // 100ms

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
 ** \brief Timera unit 1 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void TimeraUnit1_IrqCallback(void)
{
    /* Capture channel 0 */
    if (Set == TIMERA_GetFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH1_INT_FLAG))
    {
        BSP_LED_Toggle(LED_RED);
        TIMERA_ClearFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH1_INT_FLAG);
    }
    /* Capture channel 1 */
    if (Set == TIMERA_GetFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH2_INT_FLAG))
    {
        BSP_LED_Toggle(LED_GREEN);
        TIMERA_ClearFlag(TIMERA_UNIT1, TIMERA_UNIT1_CH2_INT_FLAG);
    }
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
    stc_timera_capture_init_t stcTimeraCaptureInit;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimeraCaptureInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Configuration TIMERA capture pin */
    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH2_PORT, TIMERA_UNIT1_CH2_PIN, TIMERA_UNIT1_CH2_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv256;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW;
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);

    /* Configuration timera unit 1 capture structure */
    stcTimeraCaptureInit.enCapturePwmRisingEn = Enable;
    stcTimeraCaptureInit.enCapturePwmFallingEn = Disable;
    stcTimeraCaptureInit.enCaptureSpecifyEventEn = Enable;
    stcTimeraCaptureInit.enPwmClkDiv = TimeraFilterPclkDiv4;
    stcTimeraCaptureInit.enPwmFilterEn = Enable;
    stcTimeraCaptureInit.enCaptureTrigRisingEn = Disable;
    stcTimeraCaptureInit.enCaptureTrigFallingEn = Disable;
    stcTimeraCaptureInit.enTrigClkDiv = TimeraFilterPclkDiv1;
    stcTimeraCaptureInit.enTrigFilterEn = Disable;
    /* Enable channel 1 capture and interrupt */
    TIMERA_CaptureInit(TIMERA_UNIT1, TIMERA_UNIT1_CH1, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH1_INT, Enable);

    /* Enable channel 2 capture and interrupt */
    TIMERA_CaptureInit(TIMERA_UNIT1, TIMERA_UNIT1_CH2, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH2_INT, Enable);

    /* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_COMPARE_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Set external Int trigger timera compare */
    TIMERA_SetCaptureTriggerSrc(KEY2_TRIGGER_EVENT);

    /* Timera unit 1 startup */
    TIMERA_Cmd(TIMERA_UNIT1, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera capture function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Configure Timera */
    Timera_Config();

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
