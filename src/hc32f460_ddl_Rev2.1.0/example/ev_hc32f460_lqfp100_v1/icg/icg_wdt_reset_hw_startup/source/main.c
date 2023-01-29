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
 ** \brief The example of ICG WDT Reset function
 **
 **   - 2018-10-24  CDT  First version for Device Driver Library of ICG.
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
/* WDT count cycle definition */
#define WDT_COUNT_CYCLE                 (16384u)

/* Reset source definition */
#define RESET_WDT_TRIGGER               (0u)
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
static uint8_t u8SysWorkSta;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  main function for ICG WDT Reset function
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
     ** #define ICG0_WDT_HARDWARE_START         ICG_FUNCTION_ON
     **
     ** #define ICG0_WDT_AUTS                   WDT_AUTO_START_AFTER_RESET
     ** #define ICG0_WDT_ITS                    WDT_RESET_REQUEST
     ** #define ICG0_WDT_PERI                   WDT_COUNT_UNDERFLOW_CYCLE_16384
     ** #define ICG0_WDT_CKS                    WDT_COUNT_PCLK3_DIV512
     ** #define ICG0_WDT_WDPT                   WDT_0To25PCT
     ** #define ICG0_WDT_SLPOFF                 WDT_SPECIAL_MODE_COUNT_STOP
     @endverbatim
     **************************************************************************/
    uint16_t u16CmpVal;
    stc_rmu_rstcause_t stcRmuRstCause;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcRmuRstCause);

    /* BSP initialization */
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Get RMU information */
    RMU_GetResetCause(&stcRmuRstCause);
    if (Set == stcRmuRstCause.enWdt)
    {
        u8SysWorkSta = RESET_WDT_TRIGGER;
        BSP_LED_On(LED_RED);
    }
    else
    {
        u8SysWorkSta = RESET_OTHER_TRIGGER;
    }
    RMU_ClrResetFlag();

    /* Wait for WDT module to complete initial load */
    Ddl_Delay1ms(200u);
    /* Count cycle=16384,range=0%-25% */
    u16CmpVal = WDT_COUNT_CYCLE / 4u;

    while (1)
    {
        if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
        {
            u16CmpVal = WDT_COUNT_CYCLE / 2u;
        }

        if (WDT_GetCountValue() < u16CmpVal)
        {
            WDT_RefreshCounter();
            /* wait for the count value to update */
            Ddl_Delay1ms(10u);
            if (RESET_OTHER_TRIGGER == u8SysWorkSta)
            {
                BSP_LED_Toggle(LED_RED);
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
