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
#define KEY10_PORT              (PortB)
#define KEY10_PIN               (Pin01)
#define KEY10_EXINT_CH          (ExtiCh01)
#define KEY10_INT_SRC           (INT_PORT_EIRQ1)
#define KEY10_IRQn              (Int007_IRQn)

#define DLY_MS                  (1000u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t u8IntCnt = 0u;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  SW4 interrupt callback function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt_IrqCallback(void)
{
    if (Set == EXINT_IrqFlgGet(KEY10_EXINT_CH))
    {
        u8IntCnt++;
        EXINT_IrqFlgClr(KEY10_EXINT_CH);
    }
}

/**
 *******************************************************************************
 ** \brief KEY1(SW4) init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void key10_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set PB1 as External Int Ch.1 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY10_PORT, KEY10_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = KEY10_EXINT_CH;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    /* Rising edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.1 */
    stcIrqRegiConf.enIntSrc = KEY10_INT_SRC;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = KEY10_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &ExtInt_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}
/**
 *******************************************************************************
 ** \brief  Main function of sleep mode wake up
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /* LED initialization */
    BSP_LED_Init();
    /* key10 initialization */
    key10_Init();

    /* key10 */
    while(Reset != PORT_GetBit(KEY10_PORT, KEY10_PIN))
    {
        ;
    }

    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_RED);
    Ddl_Delay1ms(DLY_MS);

    PWC_EnterSleepMd();

    while(1)
    {
        if(u8IntCnt % 2u)
        {
            BSP_LED_Toggle(LED_RED);
            Ddl_Delay1ms(DLY_MS);
        }
        else
        {
            PWC_EnterSleepMd();
        }
    }

}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
