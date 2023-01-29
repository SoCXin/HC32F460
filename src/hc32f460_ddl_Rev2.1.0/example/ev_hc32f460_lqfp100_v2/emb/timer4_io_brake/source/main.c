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
 ** \brief This sample demonstrates how to use EMB for Timer4.
 **
 **   - 2018-12-07 CDT first version for Device Driver Library of EMB.
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
/* Timer4 CNT */
#define TIMER4_UNIT                     (M4_TMR41)
#define TIMER4_CNT_CYCLE_VAL            (50000u)       /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_HIGH_CH              (Timer4OcoOuh)   /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               (PortE)          /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                (Pin09)

/* Timer4 PWM */
#define TIMER4_PWM_CH                   (Timer4PwmU)

/* EMB unit */
#define EMB_UNIT                        (M4_EMB2)

/* EMB unit interrupt number */
#define EMB_INT_NUM                     (INT_EMB_GR1)

/* EMB Port/Pin definition */
#define EMB_PORT                        (PortB)
#define EMB_PIN                         (Pin12)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void EmbCallBack(void);
static void Timer4PwmConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static bool m_bEmbBraking = false;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Callback function of EMB interrupt
 **
 ******************************************************************************/
static void EmbCallBack(void)
{
    if(true == EMB_GetStatus(EMB_UNIT, EMBFlagPortIn))
    {
        BSP_LED_On(LED_RED);

        EMB_SwBrake(EMB_UNIT, true);  /* Software brake Enable, still shunt down PWM after Clear Port In Brake */

        if(0 == EMB_GetStatus(EMB_UNIT, EMBPortInState))
        {
            EMB_ClrStatus(EMB_UNIT, EMBPortInFlagClr);  /* Clear Port In Brake */

            m_bEmbBraking = true;
        }
    }
}

/**
 ******************************************************************************
 ** \brief Timer4 PWM function configuration
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
static void Timer4PwmConfig(void)
{
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_timer4_emb_init_t stcEmbInit;
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcEmbInit);
    MEM_ZERO_STRUCT(stcHighChCmpMode);

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;  /* CNT clock divide */
    stcCntInit.u16Cycle = TIMER4_CNT_CYCLE_VAL;
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntCmd = Disable;
    stcCntInit.enPeakIntCmd = Disable;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */

    /* Timer4 OCO : Initialize OCO configuration structure */
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcoIntCmd = Disable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit); /* Initialize OCO high channel */

    if (!(TIMER4_OCO_HIGH_CH %2))
    {
        /* ocmr[15:0] = 0x0FFF */
        stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputReverse;
        stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;
        stcHighChCmpMode.enCntDownCntMatchOpState = OcoOpOutputReverse;

        stcHighChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;
        stcHighChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;
        stcHighChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;
        stcHighChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet;

        stcHighChCmpMode.enMatchConditionExtendCmd = Disable;

        TIMER4_OCO_SetHighChCompareMode(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcHighChCmpMode);  /* Set OCO high channel compare mode */
    }

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, TIMER4_CNT_CYCLE_VAL/2u);

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, Enable);

    /* Initialize PWM I/O */
    PORT_SetFunc(TIMER4_PWM_H_PORT, TIMER4_PWM_H_PIN, Func_Tim4, Disable);

    /* Timer4 PWM: Initialize PWM configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv1;
    stcPwmInit.enOutputState = PwmHPwmLHold;
    stcPwmInit.enMode = PwmThroughMode;
    TIMER4_PWM_Init(TIMER4_UNIT, TIMER4_PWM_CH, &stcPwmInit); /* Initialize timer4 pwm */

    /* Timer4 EMB: Initialize EMB configuration structure */
    stcEmbInit.enPwmHold = EmbChangePwm;
    stcEmbInit.enEmbState = EmbTrigPwmOutputLowLevel;
    TIMER4_EMB_Init(TIMER4_UNIT, &stcEmbInit); /* Initialize timer4 pwm */
}

/**
 *******************************************************************************
 ** \brief  Main function of project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_emb_ctrl_timer4_t   stcEmbConfig;
    stc_irq_regi_conf_t     stcIrqRegiConf;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcEmbConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_EMB, Enable);

    /* Initialize LED port */
    BSP_LED_Init();

    /* Configure Timer4 PWM function */
    Timer4PwmConfig();

    /* Set EMB port */
    PORT_SetFunc(EMB_PORT, EMB_PIN, Func_Emb, Disable);      /* EMB_IN */

    /* Initialize EMB for Timer4 */
    stcEmbConfig.bEnPortBrake = true;
    stcEmbConfig.bEnPorInFlt  = true;
    stcEmbConfig.enPortInFltClkSel = EMBPortFltDiv32;
    stcEmbConfig.bEnPortInLevelSel_Low = true;
    EMB_Config_CR_Timer4(EMB_UNIT, &stcEmbConfig);
    EMB_ClrStatus(EMB_UNIT, EMBPortInFlagClr);  /* Clear Port In Brake */
    EMB_ConfigIrq(EMB_UNIT, PORTBrkIrq, true);

    stcIrqRegiConf.enIRQn = Int000_IRQn;
    stcIrqRegiConf.enIntSrc = EMB_INT_NUM;
    stcIrqRegiConf.pfnCallback = &EmbCallBack;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_01);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    while(1)
    {
        if (true == m_bEmbBraking)
        {
            /* Add brake process code */

            Ddl_Delay1ms(3000ul);  /* only for demo using */

            EMB_SwBrake(EMB_UNIT, false); /* Disable software brake, Enable PWM output */

            BSP_LED_Off(LED_RED);

            m_bEmbBraking = false;
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
