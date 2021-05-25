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
 ** \brief clock switch system clock sample
 **
 **   - 2018-10-02  CDT  First version for Device Driver Library of
 **     clock
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
#define FCM_IRQn                (Int141_IRQn)

/* FCM windows lower/upper limitition */
#define FCM_WINDOWS_LOWER       (0x0u)
#define FCM_WINDOWS_UPPER       (0xFFFFu)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/* u16ExpectCnt = stcFcmMeasCfg.enSrc
                  / stcFcmMeasCfg.enSrcDiv
                  * stcFcmRefCfg.enIntRefDiv
                  / stcFcmRefCfg.enIntRefSrc   */

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 /**
 *******************************************************************************
 ** \brief  Fcm end interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void FcmEnd_IrqHandler(void)
{
    uint32_t u32ExpectCnt = 1953u;
    uint32_t u32Cnt = 0u;

    u32Cnt = CLK_GetFcmCounter();

    if((u32Cnt < (u32ExpectCnt+20u)) &&(u32Cnt > (u32ExpectCnt-20u)))
    {
        BSP_LED_On(LED_GREEN);
    }
    else
    {
        BSP_LED_On(LED_RED);
    }

    CLK_ClearFcmFlag(ClkFcmFlagMendf);
}

/**
 *******************************************************************************
 ** \brief  Main function
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_clk_sysclk_cfg_t        stcSysClkCfg;
    stc_clk_fcm_cfg_t           stcFcmCfg;
    stc_clk_fcm_window_cfg_t    stcFcmWinCfg;
    stc_clk_fcm_measure_cfg_t   stcFcmMeasCfg;
    stc_clk_fcm_reference_cfg_t stcFcmRefCfg;
    stc_clk_fcm_interrupt_cfg_t stcFcmIntCfg;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcFcmCfg);
    MEM_ZERO_STRUCT(stcFcmWinCfg);
    MEM_ZERO_STRUCT(stcFcmMeasCfg);
    MEM_ZERO_STRUCT(stcFcmRefCfg);
    MEM_ZERO_STRUCT(stcFcmIntCfg);

    BSP_LED_Init();

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // 16MMHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // 8 MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // 16MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // 8 MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // 4 MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // 4 MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // 8 MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Enable HRC. */
    CLK_HrcCmd(Enable);
    /* Enable XTAL32. */
    CLK_Xtal32Cmd(Enable);

    /* Switch system clock source to HRC. */
    CLK_SetSysClkSource(ClkSysSrcHRC);

    /* Enable Fcm Clk. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_FCM,Enable);

    /* Fcm measurement config. */
    stcFcmMeasCfg.enSrc = ClkFcmSrcPclk1;
    stcFcmMeasCfg.enSrcDiv = ClkFcmMeaDiv4;
    /* Fmc reference config. */
    stcFcmRefCfg.enEdge = ClkFcmEdgeRising;
    stcFcmRefCfg.enExtRef = Disable;
    stcFcmRefCfg.enFilterClk = ClkFcmFilterClkNone;
    stcFcmRefCfg.enIntRefSrc = ClkFcmSrcXtal32;
    stcFcmRefCfg.enIntRefDiv = ClkFcmIntrefDiv32;
    stcFcmRefCfg.enRefSel = ClkFcmInterRef;
    /* Fcm windows config. */
    stcFcmWinCfg.windowLower = FCM_WINDOWS_LOWER;
    stcFcmWinCfg.windowUpper = FCM_WINDOWS_UPPER;
    /* Fcm interrupt config. */
    stcFcmIntCfg.enHandleSel = ClkFcmHandleInterrupt;
    stcFcmIntCfg.enHandleInterrupt = Disable;
    stcFcmIntCfg.enEndInterrupt = Enable;
    stcFcmIntCfg.enHandleReset = Disable;
    stcFcmIntCfg.enOvfInterrupt = Disable;
    /* Fcm Config. */
    stcFcmCfg.pstcFcmWindowCfg = &stcFcmWinCfg;
    stcFcmCfg.pstcFcmMeaCfg = &stcFcmMeasCfg;
    stcFcmCfg.pstcFcmRefCfg = &stcFcmRefCfg;
    stcFcmCfg.pstcFcmIntCfg = &stcFcmIntCfg;
    CLK_FcmConfig(&stcFcmCfg);

    /* Set Fcm interrupt. */
    enShareIrqEnable(INT_FCMMENDI);

    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(FCM_IRQn);
    NVIC_SetPriority(FCM_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(FCM_IRQn);

    /* Enable Fcm. */
    CLK_FcmCmd(Enable);

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
