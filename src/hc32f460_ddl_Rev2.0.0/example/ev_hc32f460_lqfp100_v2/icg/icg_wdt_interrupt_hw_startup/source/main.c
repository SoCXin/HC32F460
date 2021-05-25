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
 **   - 2021-04-16  CDT  First version for Device Driver Library of ICG.
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
/* KEY10 Port/Pin definition */
#define KEY10_PORT                      (PortB)
#define KEY10_PIN                       (Pin01)
#define KEY10_EXINT_CH                  (ExtiCh01)
#define KEY10_INT_SRC                   (INT_PORT_EIRQ1)
#define KEY10_IRQn                      (Int007_IRQn)

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
static void ExtInt_IrqCallback(void)
{
    if (Set == EXINT_IrqFlgGet(KEY10_EXINT_CH))
    {
        u8ExIntCnt++;
        if (u8ExIntCnt >= 2u)
        {
            u8ExIntCnt = 0u;
        }
        BSP_LED_Off(LED_RED);
        BSP_LED_Off(LED_GREEN);
        EXINT_IrqFlgClr(KEY10_EXINT_CH);
    }
}

/**
 *******************************************************************************
 ** \brief KEY10 init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Key10_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set External Int */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY10_PORT, KEY10_PIN, &stcPortInit);
    stcExtiConfig.enExitCh = KEY10_EXINT_CH;
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Registration IRQ */
    stcIrqRegiConf.enIntSrc    = KEY10_INT_SRC;
    stcIrqRegiConf.enIRQn      = KEY10_IRQn;
    stcIrqRegiConf.pfnCallback = &ExtInt_IrqCallback;
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
    /* Configure WDT interrupt */
    Wdt_InterruptConfig();
    /* Key10 initialization */
    Key10_Init();

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
