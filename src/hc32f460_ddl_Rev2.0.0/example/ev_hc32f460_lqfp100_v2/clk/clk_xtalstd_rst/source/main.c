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
 ** \brief clock switch system clock sample
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of
 **     clock
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

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Main function of dectect xtal break down.
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_xtal_stp_cfg_t  stcXtalStpCfg;
    stc_clk_output_cfg_t    stcOutputClkCfg;
    stc_rmu_rstcause_t      stcRstCause;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcXtalStpCfg);
    MEM_ZERO_STRUCT(stcOutputClkCfg);

    BSP_LED_Init();

    RMU_GetResetCause(&stcRstCause);
    if(Set == stcRstCause.enXtalErr)
    {
        RMU_ClrResetFlag();
        PORT_Unlock();
        while(1)
        {
            BSP_LED_On(LED_RED);
            BSP_LED_Off(LED_GREEN);
        }
    }

    /* Xtal config. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Enable xtal fault dectect and occur reset. */
    stcXtalStpCfg.enDetect = Enable;
    stcXtalStpCfg.enMode = ClkXtalStpModeReset;
    stcXtalStpCfg.enModeInt = Disable;
    stcXtalStpCfg.enModeReset = Enable;
    CLK_XtalStpConfig(&stcXtalStpCfg);

    /* Clk output.*/
    stcOutputClkCfg.enOutputSrc = ClkOutputSrcXtal;
    stcOutputClkCfg.enOutputDiv = ClkOutputDiv8;
    CLK_OutputClkConfig(ClkOutputCh1,&stcOutputClkCfg);
    CLK_OutputClkCmd(ClkOutputCh1,Enable);

    PORT_SetFunc(PortA, Pin08, Func_Mclkout, Disable);

    PORT_Unlock();

    BSP_LED_On(LED_GREEN);

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
