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
 **   - 2021-04-16  CDT  First version for Device Driver Library of
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

/* FCM refer Port/Pin definition */
#define FCM_REF_PORT            (PortH)
#define FCM_REF_PIN             (Pin02)

/* FCM windows lower/upper limitition */
#define FCM_WINDOWS_LOWER       (0x0000)
#define FCM_WINDOWS_UPPER       (0x7000)

#define  DLY_MS                 (1000u)
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
 ** \brief  Fcm error interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void FcmErr_IrqHandler(void)
{
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_RED);
    Ddl_Delay1ms(DLY_MS);
    CLK_ClearFcmFlag(ClkFcmFlagErrf);
}

/**
 *******************************************************************************
 ** \brief  Fcm  init
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Fcm_RefPortInit(void)
{
    PORT_Unlock();

    PORT_SetFunc(FCM_REF_PORT, FCM_REF_PIN, Func_Fcmref,Disable);

    PORT_Lock();
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
    stc_clk_fcm_cfg_t           stcFcmCfg;
    stc_clk_fcm_window_cfg_t    stcFcmWinCfg;
    stc_clk_fcm_measure_cfg_t   stcFcmMeasCfg;
    stc_clk_fcm_reference_cfg_t stcFcmRefCfg;
    stc_clk_fcm_interrupt_cfg_t stcFcmIntCfg;

    MEM_ZERO_STRUCT(stcFcmCfg);
    MEM_ZERO_STRUCT(stcFcmWinCfg);
    MEM_ZERO_STRUCT(stcFcmMeasCfg);
    MEM_ZERO_STRUCT(stcFcmRefCfg);
    MEM_ZERO_STRUCT(stcFcmIntCfg);

    BSP_LED_Init();

    Fcm_RefPortInit();

    /* Enable HRC. */
    CLK_XtalCmd(Enable);

    /* Enable Fcm Clk. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_FCM,Enable);

    /* Fcm measurement clock xtal, 1 div. */
    stcFcmMeasCfg.enSrc = ClkFcmSrcXtal;
    stcFcmMeasCfg.enSrcDiv = ClkFcmMeaDiv1;
    /* Fmc reference config, use external clock. */
    stcFcmRefCfg.enEdge = ClkFcmEdgeRising;
    stcFcmRefCfg.enExtRef = Enable;
    stcFcmRefCfg.enFilterClk = ClkFcmFilterClkNone;
    stcFcmRefCfg.enRefSel = ClkFcmExtRef;
    /* Fcm windows config. */
    stcFcmWinCfg.windowLower = (uint16_t)FCM_WINDOWS_LOWER;
    stcFcmWinCfg.windowUpper = (uint16_t)FCM_WINDOWS_UPPER;
    /* Fcm interrupt config. */
    stcFcmIntCfg.enHandleSel = ClkFcmHandleInterrupt;
    stcFcmIntCfg.enHandleInterrupt = Enable;
    stcFcmIntCfg.enEndInterrupt = Disable;
    stcFcmIntCfg.enHandleReset = Disable;
    stcFcmIntCfg.enOvfInterrupt = Disable;
    /* Fcm Config. */
    stcFcmCfg.pstcFcmWindowCfg = &stcFcmWinCfg;
    stcFcmCfg.pstcFcmMeaCfg = &stcFcmMeasCfg;
    stcFcmCfg.pstcFcmRefCfg = &stcFcmRefCfg;
    stcFcmCfg.pstcFcmIntCfg = &stcFcmIntCfg;
    CLK_FcmConfig(&stcFcmCfg);

    /* Set Fcm interrupt. */
    enShareIrqEnable(INT_FCMFERRI);

    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(FCM_IRQn);
    NVIC_SetPriority(FCM_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(FCM_IRQn);

    /* Enable Fcm. */
    CLK_FcmCmd(Enable);

    BSP_LED_On(LED_GREEN);
    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
