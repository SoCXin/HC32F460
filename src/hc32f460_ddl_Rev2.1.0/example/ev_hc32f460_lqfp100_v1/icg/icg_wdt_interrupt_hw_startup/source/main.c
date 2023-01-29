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
 ** \brief The example of ICG WDT Interrupt function
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
 ** \brief WDT interrupt config
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Wdt_InterruptConfig(void)
{
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcIrqRegiConf);

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
 ** \brief  main function for ICG WDT Interrupt function
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
     ** #define ICG0_WDT_ITS                    WDT_INTERRUPT_REQUEST
     ** #define ICG0_WDT_PERI                   WDT_COUNT_UNDERFLOW_CYCLE_16384
     ** #define ICG0_WDT_CKS                    WDT_COUNT_PCLK3_DIV512
     ** #define ICG0_WDT_WDPT                   WDT_0To100PCT
     ** #define ICG0_WDT_SLPOFF                 WDT_SPECIAL_MODE_COUNT_CONTINUE
     @endverbatim
     **************************************************************************/
    /* BSP initialization */
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Configure WDT interrupt */
    Wdt_InterruptConfig();

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
