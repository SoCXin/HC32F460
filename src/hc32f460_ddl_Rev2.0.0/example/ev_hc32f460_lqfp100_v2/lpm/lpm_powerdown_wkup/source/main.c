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
 **   - 2021-04-16  CDT  First version for Device Driver Library of LPM.
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
#define DLY_MS                          (1000u)

#define PWC_PD_WAKEUP_EVENT             (PWC_PDWKEN0_WKUP01)
//#define PWC_PD_WAKEUP_EVENT             (PWC_PDWKEN2_NMI)
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
 ** \brief  Main function of power down mode wake up
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_pwc_pwr_mode_cfg_t  stcPwcPwrMdCfg;

    MEM_ZERO_STRUCT(stcPwcPwrMdCfg);

    BSP_CLK_Init();
    BSP_LED_Init();

    /* Config power down mode. */
    stcPwcPwrMdCfg.enPwrDownMd = PowerDownMd1;
    stcPwcPwrMdCfg.enRLdo = Enable;
    stcPwcPwrMdCfg.enIoRetain = IoPwrDownRetain;
    stcPwcPwrMdCfg.enRetSram = Disable;
    stcPwcPwrMdCfg.enPwrDWkupTm = Vcap0047;

    PWC_PowerModeCfg(&stcPwcPwrMdCfg);

    if(PWC_PDWKEN0_WKUP01 == PWC_PD_WAKEUP_EVENT)
    {
        /* wake_up 0_1 event */
        PWC_ClearWakeup0Flag(PWC_PTWK0_WKUPFLAG);
        PWC_PdWakeupEvtEdgeCfg(PWC_PDWKUP_EDGE_WKP0, EdgeFalling);
        PWC_PdWakeup0Cmd(PWC_PDWKEN0_WKUP01,Enable);
    }
    else
    {
        /* NMI wake_up. */
        PWC_ClearWakeup0Flag(PWC_NMI_WKUPFLAG);
        PWC_PdWakeupEvtEdgeCfg(PWC_PDWKUP_EDGE_NMI, EdgeFalling);
        PWC_PdWakeup2Cmd(PWC_PDWKEN2_NMI,Enable);
    }

    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);

    PWC_EnterPowerDownMd();

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
