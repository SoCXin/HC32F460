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
 ** \brief The example of WDT Reset function
 **
 **   - 2018-10-24  CDT  First version for Device Driver Library of WDT.
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
 ** \brief WDT configure
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Wdt_Config(void)
{
    stc_wdt_init_t stcWdtInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcWdtInit);

    stcWdtInit.enClkDiv = WdtPclk3Div512;
    stcWdtInit.enCountCycle = WdtCountCycle16384;
    stcWdtInit.enRefreshRange = WdtRefresh0To25Pct;
    stcWdtInit.enSleepModeCountEn = Disable;
    stcWdtInit.enRequestType = WdtTriggerResetRequest;
    WDT_Init(&stcWdtInit);
}

/**
 *******************************************************************************
 ** \brief  main function for WDT Reset function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
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

    /* WDT configure */
    Wdt_Config();
    /* First refresh to start WDT */
    WDT_RefreshCounter();
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
