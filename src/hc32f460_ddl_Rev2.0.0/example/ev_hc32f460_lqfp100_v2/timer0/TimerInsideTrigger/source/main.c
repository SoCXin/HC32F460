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
 ** \brief This sample demonstrates how to use timer0.
 **
 **   - 2021-04-16  CDT first version for inside trigger function of Timer0.
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
/* Define Timer Unit for example */
#define TMR_UNIT            (M4_TMR02)
#define TMR_INI_GCMA        (INT_TMR02_GCMA)
#define TMR_INI_GCMB        (INT_TMR02_GCMB)

#define ENABLE_TMR0()       (PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02, Enable))
/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint16_t u16CmpLast = 0u;
static __IO uint16_t u16Campture = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Callback function of timer0
 **
 ******************************************************************************/
void Timer0_TriggerCallBack(void)
{
    uint16_t tmp;

    tmp = TIMER0_GetCmpReg(TMR_UNIT,Tim0_ChannelB);

    u16Campture = tmp - u16CmpLast;
    u16CmpLast = tmp;

    BSP_LED_Toggle(LED_GREEN);
}

 /*******************************************************************************
 ** \brief K10 init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void K10_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcPortInit);

    /* External Int Ch.1 */
    stcExtiConfig.enExitCh = ExtiCh01;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set PB1 as External Int Ch.1 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(PortB, Pin01, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  Main function of example project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_tim0_trigger_init_t stcInputCaptruecfg;
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcTimerCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcInputCaptruecfg);
    MEM_ZERO_STRUCT(stcPortInit);

    /* system clock intialize */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    /*Init K10*/
    K10_Init();

    /* Timer0 peripheral enable and config */
    ENABLE_TMR0();
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv32;
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelB,&stcTimerCfg);
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);

    /* Input captrue enable */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
    /*config timer0 trigger function*/
    stcInputCaptruecfg.Tim0_OCMode = Tim0_InputCaptrue;
    stcInputCaptruecfg.Tim0_SelTrigSrc = EVT_PORT_EIRQ1;
    stcInputCaptruecfg.Tim0_InTrigEnable = true;
    stcInputCaptruecfg.Tim0_InTrigClear = false;
    stcInputCaptruecfg.Tim0_InTrigStart = true;
    stcInputCaptruecfg.Tim0_InTrigStop = false;
    TIMER0_HardTriggerInit(TMR_UNIT,Tim0_ChannelB,&stcInputCaptruecfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;
    /* Select TIMER channalB interrupt as source */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = &Timer0_TriggerCallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    while(1)
    {
        if(u16CmpLast != 0u)
        {
            BSP_LED_On(LED_BLUE);
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
