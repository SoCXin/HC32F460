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
 ** \brief The example of ICG VDU0 function
 **
 **   - 2018-10-22  CDT  First version for Device Driver Library of ICG.
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
/* Reset source definition */
#define RESET_VDU0_TRIGGER              (0u)
#define RESET_OTHER_TRIGGER             (1u)

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
 ** \brief  main function for ICG VDU0 function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /**
     ***************************************************************************
     @verbatim
     ** Modify hc32f460_icg.h file of defines
     ** #define ICG1_VDU0_HARDWARE_START    ICG_FUNCTION_ON
     **
     ** #define ICG1_VDU0_BOR_LEV           VDU0_VOLTAGE_THRESHOLD_2P3
     ** #define ICG1_VDU0_BORDIS            VDU0_START_AFTER_RESET
     @endverbatim
     **************************************************************************/
    uint8_t resetSta;
    stc_rmu_rstcause_t stcRmuRstCause;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcRmuRstCause);

    /* BSP initialization */
    BSP_LED_Init();
    /* Get RMU information */
    RMU_GetResetCause(&stcRmuRstCause);
    if (Set == stcRmuRstCause.enBrownOut)
    {
        resetSta = RESET_VDU0_TRIGGER;
        BSP_LED_On(LED_RED);
    }
    else
    {
        resetSta = RESET_OTHER_TRIGGER;
    }
    RMU_ClrResetFlag();

    while (1)
    {
        if (RESET_OTHER_TRIGGER == resetSta)
        {
            BSP_LED_Toggle(LED_RED);
            Ddl_Delay1ms(1000u);
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
