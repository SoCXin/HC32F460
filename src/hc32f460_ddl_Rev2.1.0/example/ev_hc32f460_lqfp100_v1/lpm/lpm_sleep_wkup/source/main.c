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
 **   - 2018-11-06  CDT  First version for Device Driver Library of LPM.
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
void BSP_KEY_KEY4_IrqHandler(void)
{
    if (Set == EXINT_IrqFlgGet(BSP_KEY_KEY4_EXINT))
    {
        u8IntCnt++;
        EXINT_IrqFlgClr(BSP_KEY_KEY4_EXINT);
    }
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
    /* KEY initialization */
    BSP_KEY_Init();

    /* SW2 */
    while(Reset != PORT_GetBit(PortD, Pin03))
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
