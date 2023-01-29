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
 ** \brief The example of Timera base count function
 **
 **   - 2018-11-12  CDT  First version for Device Driver Library of
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
/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    (M4_TMRA1)
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TIMERA_UNIT1_OVERFLOW_INT       (INT_TMRA1_OVF)

#define TIMERA_UNIT2                    (M4_TMRA2)
#define TIMERA_UNIT2_CLOCK              (PWC_FCG2_PERIPH_TIMA2)
#define TIMERA_UNIT2_OVERFLOW_INT       (INT_TMRA2_OVF)

#define TIMERA_COUNT_OVERFLOW           (SystemCoreClock/2U/128U/100U - 1U)  // 10ms

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8TmraUnit1Cnt = 0u, u8TmraUnit2Cnt = 0u;

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
 ** \brief Timera unit 2 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void TimeraUnit2_IrqCallback(void)
{
    u8TmraUnit2Cnt++;
    if (u8TmraUnit2Cnt >= 100u)      //1s
    {
        u8TmraUnit2Cnt = 0u;
        BSP_LED_Toggle(LED_GREEN);
    }
    TIMERA_ClearFlag(TIMERA_UNIT2, TimeraFlagOverflow);
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
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK | TIMERA_UNIT2_CLOCK, Enable);

    /* Configuration timera unit 1 structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv128;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = TIMERA_COUNT_OVERFLOW;
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);

    /* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configuration timera unit 2 structure */
    stcTimeraInit.enSyncStartupEn = Enable;
    TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);

    /* Configure interrupt of timera unit 2 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit2_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Sync startup timera unit 2 when timera unit 1 startup */
    TIMERA_Cmd(TIMERA_UNIT1, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera base count function
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
    /* Configure Timera */
    Timera_Config();

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
