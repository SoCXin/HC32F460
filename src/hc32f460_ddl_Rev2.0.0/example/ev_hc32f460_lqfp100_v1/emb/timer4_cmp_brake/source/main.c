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
 ** \brief This sample demonstrates how to use EMB compare function for Timer4.
 **
 **   - 2018-12-11 CDT first version for Device Driver Library of EMB.
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

/* Timer4 PWM */
#define TIMER4_PWM_CH                   (Timer4PwmU)

/* Define port and pin for Timer4Pwm */
#define TIMER4_PWM_H_PORT               (PortE)          /* TIM4_1_OUH_B:PE9   TIM4_1_OVH_B:PE11   TIM4_1_OWH_B:PE13 */
#define TIMER4_PWM_H_PIN                (Pin09)

/* EMB unit */
#define EMB_UNIT                        (M4_EMB2)

/* EMB unit interrupt number */
#define EMB_INT_NUM                     (INT_EMB_GR1)

#define DAC_Enable

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void EmbCallBack(void);
static void Timer4PwmConfig(void);
static void EmbCallBack(void);
static void M4_CMP_Init(void);

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
    if(true == EMB_GetStatus(EMB_UNIT, EMBFlagCmp))
    {
        BSP_LED_On(LED_RED);

        EMB_SwBrake(EMB_UNIT, true);  /* Software brake Enable, still shunt down PWM after Clear Port In Brake */

        EMB_ClrStatus(EMB_UNIT, EMBCmpFlagClr);  /* Clear CMP In Brake */

        m_bEmbBraking = true;
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

 /*******************************************************************************
 ** \brief CMP init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void M4_CMP_Init(void)
{
    stc_cmp_init_t         stcCmpConfig;
    stc_irq_regi_conf_t    stcIrqRegiConf;
    stc_port_init_t        stcPortInit;
    stc_cmp_input_sel_t    stcCmpInput;
    stc_cmp_dac_init_t     stcDacInitCfg;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcCmpConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcCmpInput);
    MEM_ZERO_STRUCT(stcDacInitCfg);

    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);

#ifdef DAC_Enable
    /* Set DAC */
    //DAC1 for CMP1(INM3); DAC2 for CMP2(INM3); DAC1 for CMP3(INM3) / DAC2 for CMP3(INM4)
    stcDacInitCfg.u8DacData = 0x80u;
    stcDacInitCfg.enCmpDacEN = Enable;
    CMP_DAC_Init(CmpDac1, &stcDacInitCfg);
    CMP_DAC_Init(CmpDac2, &stcDacInitCfg);
#endif


//CMP1
#if 1
    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PA0 as CMP1_INP1 input */
    PORT_Init(PortA, Pin00, &stcPortInit);
    /* Set PC3 as CMP1_INM2 input */
    PORT_Init(PortC, Pin03, &stcPortInit);

    /* Set PB12 as CH1  output */
    PORT_SetFunc(PortB, Pin12, Func_Vcout, Disable);

    /* mode set */
    stcCmpConfig.enCmpIntEN = Disable;            //intrrupt disable
    stcCmpConfig.enCmpInvEn = Disable;            //CMP INV sel for output
    stcCmpConfig.enCmpOutputEn = Enable;          //CMP Output enable
    stcCmpConfig.enCmpVcoutOutputEn = Enable;     //CMP output result enable
    stcCmpConfig.enEdgeSel = CmpRisingEdge;       //Fall edge active brake
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div32;  //PCLK3/32
    CMP_Init(M4_CMP1, &stcCmpConfig);

    //gpio set for cmp
    stcCmpInput.enInp4Sel = CmpInp4None;
    stcCmpInput.enInpSel = CmpInp1;
    stcCmpInput.enInmSel = CmpInm2;
  #ifdef DAC_Enable
     stcCmpInput.enInmSel = CmpInm3;
  #endif
    CMP_InputSel(M4_CMP1, &stcCmpInput);

  #if 0
    stcIrqRegiConf.enIntSrc = INT_ACMP1;          //Select CMP
    stcIrqRegiConf.enIRQn = Int112_IRQn;          //Register CMP
    stcIrqRegiConf.pfnCallback = ACMP1_Callback;  //Callback function
    enIrqRegistration(&stcIrqRegiConf);           //Registration IRQ


    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);  //Clear pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);       //Enable NVIC
  #endif

    CMP_Cmd(M4_CMP1,Enable);    //Enable CMP1
#endif


//CMP2
#if 0
    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PA6 as CMP2_INP3 input */
    PORT_Init(PortA, Pin06, &stcPortInit);
    /* Set PC4 as CMP2_INN2 input */
    PORT_Init(PortC, Pin04, &stcPortInit);

    /* Set PB13 as CH2  output */
    PORT_SetFunc(PortB, Pin13, Func_Vcout, Disable);

    /* mode set */
    stcCmpConfig.enCmpIntEN = Disable;            //intrrupt disable
    stcCmpConfig.enCmpInvEn = Disable;            //CMP INV sel for output
    stcCmpConfig.enCmpOutputEn = Enable;          //CMP Output enable
    stcCmpConfig.enCmpVcoutOutputEn = Enable;     //CMP output result enable
    stcCmpConfig.enEdgeSel = CmpRisingEdge;       //Fall edge active brake
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div32;  //PCLK3/32
    CMP_Init(M4_CMP2, &stcCmpConfig);

    stcCmpInput.enInpSel = CmpInp3;
    stcCmpInput.enInmSel = CmpInm2;
  #ifdef DAC_Enable
     stcCmpInput.enInmSel = CmpInm3;
  #endif
    CMP_InputSel(M4_CMP2, &stcCmpInput);

    CMP_Cmd(M4_CMP2,Enable);    //Enable CMP2
#endif


//CMP3
#if 0
    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PB1 as CMP3_INP2 input */
    PORT_Init(PortB, Pin01, &stcPortInit);
    /* Set PC5 as CMP3_INM2 input */
    PORT_Init(PortC, Pin05, &stcPortInit);

    /* Set PB14 as CH3  output */
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enPinSubFunc = Enable;
    PORT_Init(PortB,  Pin14, &stcPortInit);
    PORT_SetFunc(PortB, Pin14, Func_Vcout, Disable);

    /* mode set */
    stcCmpConfig.enCmpIntEN = Disable;            //intrrupt disable
    stcCmpConfig.enCmpInvEn = Disable;            //CMP INV sel for output
    stcCmpConfig.enCmpOutputEn = Enable;          //CMP Output enable
    stcCmpConfig.enCmpVcoutOutputEn = Enable;     //CMP output result enable
    stcCmpConfig.enEdgeSel = CmpRisingEdge;       //Fall edge active brake
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div32;  //PCLK3/32
    CMP_Init(M4_CMP3, &stcCmpConfig);

    stcCmpInput.enInpSel = CmpInp2;
    stcCmpInput.enInmSel = CmpInm2;
  #ifdef DAC_Enable
    stcCmpInput.enInmSel = CmpInm3;
  #endif
    CMP_InputSel(M4_CMP3, &stcCmpInput);

    CMP_Cmd(M4_CMP3, Enable);   //Enable CMP3
#endif

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
    stc_irq_regi_conf_t   stcIrqRegiConf;
    stc_emb_ctrl_timer4_t stcEMBConfigCR;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcEMBConfigCR);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_EMB, Enable);

    /* Initialize LED port */
    BSP_LED_Init();

    /* Configure Timer4 PWM function */
    Timer4PwmConfig();

    /* Configure Compare function */
    M4_CMP_Init();

    /* Configure EMB function */
    stcEMBConfigCR.bEnCmp1Brake = true;
    EMB_Config_CR_Timer4(EMB_UNIT, &stcEMBConfigCR);
    EMB_ClrStatus(EMB_UNIT, EMBCmpFlagClr);  /* Clear Port In Brake */
    EMB_ConfigIrq(EMB_UNIT, CMPBrkIrq, true);

    /* Configure EMB interrupt */
    stcIrqRegiConf.enIRQn = Int000_IRQn;                    /* Register INT_TMR61_GUDF Int to Vect.No.002 */
    stcIrqRegiConf.enIntSrc = EMB_INT_NUM;                  /* Select Event interrupt function */
    stcIrqRegiConf.pfnCallback = &EmbCallBack;              /* Callback function */
    enIrqRegistration(&stcIrqRegiConf);                     /* Registration IRQ */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            /* Clear Pending */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_01); /* Set priority */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                  /* Enable NVIC */

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
