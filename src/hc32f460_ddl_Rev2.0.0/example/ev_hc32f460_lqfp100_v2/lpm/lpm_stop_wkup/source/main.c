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

#define KEY1_IN_PORT            (PortD)
#define KEY1_IN_PIN             (Pin12)
#define KEY4_IN_PORT            (PortD)
#define KEY4_IN_PIN             (Pin13)

#define KEY1_OUT_PORT           (PortA)
#define KEY1_OUT_PIN            (Pin04)
#define KEY4_OUT_PORT           (PortA)
#define KEY4_OUT_PIN            (Pin05)

#define KEY1_EXINT_CH           (ExtiCh12)
#define KEY4_EXINT_CH           (ExtiCh13)

#define KEY1_INT_SRC            (INT_PORT_EIRQ12)
#define KEY4_INT_SRC            (INT_PORT_EIRQ13)

#define KEY1_INT_IRQn           (Int000_IRQn)
#define KEY4_INT_IRQn           (Int001_IRQn)

#define DLY_MS                  (1000u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void ExtInt12_Callback(void);
static void ExtInt13_Callback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint32_t u32ExtInt12Count = 0ul;
static uint32_t u32ExtInt13Count = 0ul;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Port init.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Key_Init(void)
{
    stc_port_init_t     stcPortInit;
    stc_exint_config_t  stcExintCfg;
    stc_irq_regi_conf_t stcPortIrqCfg;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExintCfg);
     MEM_ZERO_STRUCT(stcPortIrqCfg);

    PORT_Unlock();

    M4_PORT->PODRD |= (0x11u << 12u);
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(KEY1_OUT_PORT, KEY1_OUT_PIN, &stcPortInit);
    PORT_Init(KEY4_OUT_PORT, KEY4_OUT_PIN, &stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(KEY1_IN_PORT, KEY1_IN_PIN, &stcPortInit);
    PORT_Init(KEY4_IN_PORT, KEY4_IN_PIN, &stcPortInit);

    PORT_Lock();

    /* K1 Exint config. */
    stcExintCfg.enExitCh = KEY1_EXINT_CH;
    stcExintCfg.enFilterEn = Disable;
    stcExintCfg.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExintCfg);

    /* K4 Exint config. */
    stcExintCfg.enExitCh = KEY4_EXINT_CH;
    EXINT_Init(&stcExintCfg);

    /* Register EIRQ12.*/
    stcPortIrqCfg.enIntSrc = KEY1_INT_SRC;
    stcPortIrqCfg.enIRQn = KEY1_INT_IRQn;
    stcPortIrqCfg.pfnCallback = &ExtInt12_Callback;
    enIrqRegistration(&stcPortIrqCfg);

    /* Enable EIRQ12. */
    NVIC_ClearPendingIRQ(KEY1_INT_IRQn);
    NVIC_SetPriority(KEY1_INT_IRQn,DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(KEY1_INT_IRQn);

    /* Register EIRQ13.*/
    stcPortIrqCfg.enIntSrc = KEY4_INT_SRC;
    stcPortIrqCfg.enIRQn = KEY4_INT_IRQn;
    stcPortIrqCfg.pfnCallback = &ExtInt13_Callback;
    enIrqRegistration(&stcPortIrqCfg);

    /* Enable EIRQ13. */
    NVIC_ClearPendingIRQ(KEY4_INT_IRQn);
    NVIC_SetPriority(KEY4_INT_IRQn,DDL_IRQ_PRIORITY_14);
    NVIC_EnableIRQ(KEY4_INT_IRQn);
}

static void ExtInt12_Callback(void)
{
    /* Recover clock. */
    PWC_IrqClkRecover();

    /* To show the times the interrupt occured. */
    u32ExtInt12Count++;
    /* Clear the interrupt flag. */
    EXINT_IrqFlgClr(KEY1_EXINT_CH);

    BSP_LED_Toggle(LED_BLUE);
    Ddl_Delay1ms(DLY_MS);
    BSP_LED_Toggle(LED_BLUE);
    Ddl_Delay1ms(DLY_MS);

    /* Switch system clock as MRC. */
    PWC_IrqClkBackup();
}

static void ExtInt13_Callback(void)
{
    /* Recover clock. */
    PWC_IrqClkRecover();

    /* To show the times the interrupt occured. */
    u32ExtInt13Count++;
    /* Clear the interrupt flag. */
    EXINT_IrqFlgClr(KEY4_EXINT_CH);

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
    /* Key1 & Key4 initialization */
    Key_Init();

    /* Config stop mode. */
    stcPwcStopCfg.enStpDrvAbi = StopHighspeed;
    stcPwcStopCfg.enStopClk = ClkFix;
    stcPwcStopCfg.enStopFlash = Wait;
    stcPwcStopCfg.enPll = Enable;

    while(Ok != PWC_StopModeCfg(&stcPwcStopCfg))
    {
        ;
    }

    /* Set wake up source EIRQ12, EIRQ13. */
    enIntWakeupEnable(Extint12WU);
    enIntWakeupEnable(Extint13WU);

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

    /* key10 */
    while(Reset != PORT_GetBit(KEY10_PORT, KEY10_PIN))
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
