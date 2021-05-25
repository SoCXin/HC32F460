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
 ** \brief The example of Timera position overflow count function
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
/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    (M4_TMRA1)
#define TIMERA_UNIT1_CLOCK              (PWC_FCG2_PERIPH_TIMA1)
#define TIMERA_UNIT1_OVERFLOW_INT       (INT_TMRA1_OVF)

#define TIMERA_UNIT2                    (M4_TMRA2)
#define TIMERA_UNIT2_CLOCK              (PWC_FCG2_PERIPH_TIMA2)
#define TIMERA_UNIT2_OVERFLOW_INT       (INT_TMRA2_OVF)

/* TIMERA CLKA Port/Pin definition */
#define TIMERA_UNIT1_CLKA_PORT          (PortA)
#define TIMERA_UNIT1_CLKA_PIN           (Pin08)
#define TIMERA_UNIT1_CLKA_FUNC          (Func_Tima0)

/* TIMERA CLKB Port/Pin definition */
#define TIMERA_UNIT1_CLKB_PORT          (PortE)
#define TIMERA_UNIT1_CLKB_PIN           (Pin11)
#define TIMERA_UNIT1_CLKB_FUNC          (Func_Tima0)

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
 ** \brief Timera unit 1 count overflow callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void TimeraUnit1Over_IrqCallback(void)
{
    BSP_LED_Toggle(LED_RED);
    TIMERA_ClearFlag(TIMERA_UNIT1, TimeraFlagOverflow);
}

/**
 *******************************************************************************
 ** \brief Timera unit 2 count overflow callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void TimeraUnit2Over_IrqCallback(void)
{
    BSP_LED_Toggle(LED_GREEN);
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
    stc_timera_orthogonal_coding_init_t stcTimeraCondingInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimeraCondingInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK | TIMERA_UNIT2_CLOCK, Enable);

    /* Configuration TIMERA coding pin */
    PORT_SetFunc(TIMERA_UNIT1_CLKA_PORT, TIMERA_UNIT1_CLKA_PIN, TIMERA_UNIT1_CLKA_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CLKB_PORT, TIMERA_UNIT1_CLKB_PIN, TIMERA_UNIT1_CLKB_FUNC, Disable);

    /* Configuration timera unit 1 structure */
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 1000u;
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);

    /* Configure timera uint 2 structure */
    stcTimeraInit.u16PeriodVal = 6u;
    TIMERA_BaseInit(TIMERA_UNIT2, &stcTimeraInit);
    TIMERA_IrqCmd(TIMERA_UNIT2, TimeraIrqOverflow, Enable);

    /* Configure coding count structure */
    stcTimeraCondingInit.enIncClkBHighAndClkARisingEn = Enable;
    stcTimeraCondingInit.enClkAFilterEn = Enable;
    stcTimeraCondingInit.enClkAClkDiv = TimeraFilterPclkDiv4;
    stcTimeraCondingInit.enClkBFilterEn = Enable;
    stcTimeraCondingInit.enClkBClkDiv = TimeraFilterPclkDiv4;
    TIMERA_OrthogonalCodingInit(TIMERA_UNIT1, &stcTimeraCondingInit);

    /* Configure position overflow count structure */
    MEM_ZERO_STRUCT(stcTimeraCondingInit);
    stcTimeraCondingInit.enIncAnotherUnitOverflowEn = Enable;
    TIMERA_OrthogonalCodingInit(TIMERA_UNIT2, &stcTimeraCondingInit);

    /* Configure count overflow interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit1Over_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure count overflow interrupt of timera unit 2 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT2_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    stcIrqRegiConf.pfnCallback = &TimeraUnit2Over_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Timera unit 1 and unit 2 startup */
    TIMERA_Cmd(TIMERA_UNIT1, Enable);
    TIMERA_Cmd(TIMERA_UNIT2, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for Timera position overflow count function
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
