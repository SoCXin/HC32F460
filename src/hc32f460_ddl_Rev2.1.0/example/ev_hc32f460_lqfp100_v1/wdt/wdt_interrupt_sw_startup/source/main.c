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
 ** \brief The example of WDT Interrupt function
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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8ExIntCnt = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief WDT interrupt callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Wdt_IrqCallback(void)
{
    en_flag_status_t enFlagSta;

    Ddl_Delay1ms(2u);
    enFlagSta = WDT_GetFlag(WdtFlagCountUnderflow);
    /* WDT underflow interrupt */
    if (Set == enFlagSta)
    {
        WDT_ClearFlag(WdtFlagCountUnderflow);
        /* Normal mode */
        if (0u == u8ExIntCnt)
        {
            BSP_LED_Toggle(LED_RED);
        }
        /* Sleep mode */
        else
        {
            BSP_LED_Toggle(LED_GREEN);
        }
    }
    WDT_RefreshCounter();
}

/**
 *******************************************************************************
 ** \brief ExtInt callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void BSP_KEY_KEY2_IrqHandler(void)
{
    if (Set == EXINT_IrqFlgGet(BSP_KEY_KEY2_EXINT))
    {
        u8ExIntCnt++;
        if (u8ExIntCnt >= 2u)
        {
            u8ExIntCnt = 0u;
        }
        BSP_LED_Off(LED_RED);
        BSP_LED_Off(LED_GREEN);
        EXINT_IrqFlgClr(BSP_KEY_KEY2_EXINT);
    }
}

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
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcWdtInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* WDT structure parameters configure */
    stcWdtInit.enClkDiv = WdtPclk3Div512;
    stcWdtInit.enCountCycle = WdtCountCycle16384;
    stcWdtInit.enRefreshRange = WdtRefresh0To100Pct;
    stcWdtInit.enSleepModeCountEn = Enable;
    stcWdtInit.enRequestType = WdtTriggerInterruptRequest;
    WDT_Init(&stcWdtInit);

    /* Select Int source WDT */
    stcIrqRegiConf.enIntSrc = INT_WDT_REFUDF;
    /* Register WDT Int to Vect.No.006 */
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Wdt_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief  main function for WDT Interrupt function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /* BSP initialization */
    BSP_LED_Init();
    BSP_KEY_Init();
    /* WDT configure */
    Wdt_Config();
    /* First refresh to start WDT */
    WDT_RefreshCounter();

    while (1)
    {
        /* Sleep mode */
        if (1u == u8ExIntCnt)
        {
            PWC_EnterSleepMd();
            __WFI();
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
