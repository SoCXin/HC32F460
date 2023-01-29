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
 **   - 2021-04-16  CDT  First version for Device Driver Library of PWC.
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

/* PVD Port/Pin definition */
#define PVD_PORT                (PortB)
#define PVD_PIN                 (Pin02)

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
 ** \brief  PVD2 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Pvd2_IrqHandler(void)
{
    PWC_ClearPvdFlag(PvdU2);
    BSP_LED_On(LED_RED);
}

/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_pwc_pvd_cfg_t   stcPwcPvdCfg;
    stc_port_init_t     stcPvdPortInit;
    stc_nmi_config_t    stcNmiCfd;

    MEM_ZERO_STRUCT(stcPwcPvdCfg);
    MEM_ZERO_STRUCT(stcPvdPortInit);
    MEM_ZERO_STRUCT(stcNmiCfd);

    /* Initialize LED port */
    BSP_LED_Init();

    /* Initialize PVD2 external port */
    stcPvdPortInit.enPinMode =  Pin_Mode_Ana;
    PORT_Init(PVD_PORT, PVD_PIN, &stcPvdPortInit);
    //PORT_SetFunc(PVD_PORT, PVD_PIN, Func_Vcout, Disable);

    /* Config PVD2.*/
    /* Disable filter. */
    stcPwcPvdCfg.enPvd2FilterEn = Disable;
    /* Non-Msk interrupt. */
    stcPwcPvdCfg.enPvd2Int = MskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdMode = PvdInt;
    /* Enable Pvd2 interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdCmpOutEn = Enable;
    /* PVD2 Threshold Voltage 1.1V. */
    stcPwcPvdCfg.enPvd2Level = Pvd2Level7;

    PWC_PvdCfg(&stcPwcPvdCfg);

    /* Set PVD2 interrupt. */
    enShareIrqEnable(INT_PVD_PVD2);
    /* Enable NVIC */
    NVIC_ClearPendingIRQ(PVD_IRQn);
    NVIC_SetPriority(PVD_IRQn, DDL_IRQ_PRIORITY_01);
    NVIC_EnableIRQ(PVD_IRQn);

    /* Enable external input */
    PWC_ExVccCmd(Enable);
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
