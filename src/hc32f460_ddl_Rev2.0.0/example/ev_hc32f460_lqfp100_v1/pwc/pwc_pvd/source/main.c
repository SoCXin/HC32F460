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
 ** \brief power voltage detected interrupt sample
 **
 **   - 2018-11-06  CDT  First version for Device Driver Library of PWC.
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
#define PVD_IRQn                (Int141_IRQn)

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
 ** \brief  PVD1 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Pvd1_IrqHandler(void)
{
    PWC_ClearPvdFlag(PvdU1);
    BSP_LED_On(LED_RED);
}
/**
 *******************************************************************************
 ** \brief  PVD2 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Pvd2_IrqCallBack(void)
{
    PWC_ClearPvdFlag(PvdU2);
    BSP_LED_Off(LED_RED);
    BSP_LED_On(LED_YELLOW);
}
/**
 *******************************************************************************
 ** \brief  PVD1 & PVD2 interrupt.
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_pwc_pvd_cfg_t   stcPwcPvdCfg;
    stc_nmi_config_t    stcNmiCfd;

    MEM_ZERO_STRUCT(stcPwcPvdCfg);

    /* Initialize LED port */
    BSP_LED_Init();

    /* Config PVD1. */
    /* Disable filter. */
    stcPwcPvdCfg.enPvd1FilterEn = Disable;
    /* Msk interrupt. */
    stcPwcPvdCfg.enPvd1Int = MskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdMode = PvdInt;
    /* Enable Pvd1 interrupt. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdCmpOutEn = Enable;
    /* PVD1 Threshold Voltage 2.8V. */
    stcPwcPvdCfg.enPvd1Level = Pvd1Level6;

    /* Config PVD2.*/
    /* Disable filter. */
    stcPwcPvdCfg.enPvd2FilterEn = Disable;
    /* Non-Msk interrupt. */
    stcPwcPvdCfg.enPvd2Int = NonMskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdMode = PvdInt;
    /* Enable Pvd2 interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdCmpOutEn = Enable;
    /* PVD2 Threshold Voltage 2.3V. */
    stcPwcPvdCfg.enPvd2Level = Pvd2Level1;

    PWC_PvdCfg(&stcPwcPvdCfg);

    /* Config NMI.*/
    /* Set PVD2 as NMI source. */
    stcNmiCfd.u16NmiSrc = NmiSrcVdu2;
    /* Disbale filter. */
    stcNmiCfd.enFilterEn = Disable;
    /* Set Pvd2 interrupt callback. */
    stcNmiCfd.pfnNmiCallback = &Pvd2_IrqCallBack;

    NMI_Init(&stcNmiCfd);

    /* Set PVD1 interrupt. */
    enShareIrqEnable(INT_PVD_PVD1);
    /* Enable NVIC */
    NVIC_ClearPendingIRQ(PVD_IRQn);
    NVIC_SetPriority(PVD_IRQn, DDL_IRQ_PRIORITY_01);
    NVIC_EnableIRQ(PVD_IRQn);

    /* Enable PVD1. */
    PWC_Pvd1Cmd(Enable);
    /* Enable PVD2. */
    PWC_Pvd2Cmd(Enable);

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
