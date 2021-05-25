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
static uint32_t u32ExtInt04Count = 0ul;
static uint32_t u32ExtInt05Count = 0ul;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void BSP_KEY_KEY4_IrqHandler(void)
{
    /* Recover clock. */
    PWC_IrqClkRecover();

    /* To show the times the interrupt occured. */
    u32ExtInt04Count++;
    /* Clear the interrupt flag. */
    EXINT_IrqFlgClr(BSP_KEY_KEY4_EXINT);

    BSP_LED_Toggle(LED_BLUE);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_BLUE);
    Ddl_Delay1ms(DLY_MS);

    /* Switch system clock as MRC. */
    PWC_IrqClkBackup();
}

void BSP_KEY_KEY3_IrqHandler(void)
{
    /* Recover clock. */
    PWC_IrqClkRecover();

    /* To show the times the interrupt occured. */
    u32ExtInt05Count++;
    /* Clear the interrupt flag. */
    EXINT_IrqFlgClr(BSP_KEY_KEY3_EXINT);

    BSP_LED_Toggle(LED_YELLOW);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_YELLOW);
    Ddl_Delay1ms(DLY_MS);

    /* Switch system clock as MRC. */
    PWC_IrqClkBackup();
}
/**
 *******************************************************************************
 ** \brief  Main function of stop mode wake up
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_pwc_stop_mode_cfg_t stcPwcStopCfg;
    uint32_t u32tmp1, u32tmp2;

    MEM_ZERO_STRUCT(stcPwcStopCfg);

    /* Clk initialization */
    BSP_CLK_Init();
    /* Led initialization */
    BSP_LED_Init();
    /* Key initialization */
    BSP_KEY_Init();

    /* Config stop mode. */
    stcPwcStopCfg.enStpDrvAbi = StopHighspeed;
    stcPwcStopCfg.enStopClk = ClkFix;
    stcPwcStopCfg.enStopFlash = Wait;
    stcPwcStopCfg.enPll = Enable;

    while(Ok != PWC_StopModeCfg(&stcPwcStopCfg))
    {
        ;
    }

    /* Set wake up source EIRQ4, EIRQ5. */
    enIntWakeupEnable(Extint4WU);
    enIntWakeupEnable(Extint5WU);

    /* Ensure DMA disable */
    u32tmp1 =  M4_DMA1->EN_f.EN;
    u32tmp2 =  M4_DMA2->EN_f.EN;
    while((0ul != u32tmp1) && ((0ul != u32tmp2)))
    {
        ;
    }
    /* Ensure FLASH is ready */
    while(1ul != M4_EFM->FSR_f.RDY)
    {
        ;
    }

    /* SW2 */
    while(Reset != PORT_GetBit(PortD, Pin03))
    {
        ;
    }

    while(1)
    {
        BSP_LED_Toggle(LED_RED);
        Ddl_Delay1ms(DLY_MS);
        BSP_LED_Toggle(LED_RED);
        Ddl_Delay1ms(DLY_MS);
        BSP_LED_Toggle(LED_RED);
        Ddl_Delay1ms(DLY_MS);
        BSP_LED_Toggle(LED_RED);

        /* Enter stop mode. */
        PWC_EnterStopMd();
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
