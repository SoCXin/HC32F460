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
 ** \brief This sample demonstrates how to use Timer6.
 **
 **   - 2021-04-16  CDT  first version for Device Driver Library of Timer6.
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

//#define SCMA_ValidPeriod  1

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
 ** \brief Callback function of external interrupt ch.0
 **
 ******************************************************************************/

void Timer6_CallBack(void)
{
    static uint8_t i;

    if( 0u == i)
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x3000u);
       #ifdef SCMA_ValidPeriod
        Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompC, 0x3000u);
       #endif

        i = 1u;
    }
    else
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x6000u);
      #ifdef SCMA_ValidPeriod
        Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompC, 0x6000u);
      #endif
        i = 0u;
    }

    M4_TMR61->STFLR_f.UDFF = 0u;
}

void ADC1A_CallBack(void)
{

    BSP_LED_Toggle(LED_GREEN);

}

void Config_Adc(void)
{
    en_result_t             enIrqRegResult;
    stc_adc_init_t          stcAdcInit;
    stc_adc_ch_cfg_t        stcBaseCfg;
    stc_irq_regi_conf_t     stcAdcIrqCfg;
    stc_adc_trg_cfg_t       stcAdcTrigCfg;
    stc_port_init_t         stcPortInit;
    uint8_t                 u8Adc1SampTime[3];
    uint8_t                 u8Adc2SampTime[3];

    MEM_ZERO_STRUCT(stcAdcInit);
    MEM_ZERO_STRUCT(stcBaseCfg);
    MEM_ZERO_STRUCT(stcAdcIrqCfg);
    MEM_ZERO_STRUCT(stcAdcTrigCfg);
    MEM_ZERO_STRUCT(stcPortInit);

    memset(u8Adc1SampTime, 32, sizeof(u8Adc1SampTime));
    memset(u8Adc2SampTime, 32, sizeof(u8Adc2SampTime));

    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_ADC1, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    stcPortInit.enPinMode = Pin_Mode_Ana;
    stcPortInit.enPullUp = Disable;
    PORT_Init(PortA, Pin00, &stcPortInit);
    PORT_Init(PortA, Pin01, &stcPortInit);
    PORT_Init(PortA, Pin02, &stcPortInit);

    /* Init ADC1 */
    stcAdcInit.enResolution   = AdcResolution_12Bit;
    stcAdcInit.enDataAlign    = AdcDataAlign_Right;
    stcAdcInit.enAutoClear    = AdcClren_Disable;
    stcAdcInit.enScanMode     = AdcMode_SAOnce;
    ADC_Init(M4_ADC1 ,&stcAdcInit);

    stcBaseCfg.u32Channel     = ADC1_CH0 | ADC1_CH1 | ADC1_CH2;
    stcBaseCfg.u8Sequence     = ADC_SEQ_A;
    stcBaseCfg.pu8SampTime    = u8Adc1SampTime;
    ADC_AddAdcChannel(M4_ADC1, &stcBaseCfg);

#ifdef SCMA_ValidPeriod
    stcAdcTrigCfg.u8Sequence  = ADC_SEQ_A;
    stcAdcTrigCfg.enTrgSel    = AdcTrgsel_TRGX0;
    stcAdcTrigCfg.enInTrg0    = EVT_TMR61_SCMA;
    ADC_ConfigTriggerSrc(M4_ADC1, &stcAdcTrigCfg);
    ADC_TriggerSrcCmd(M4_ADC1,ADC_SEQ_A, Enable);
#else
    stcAdcTrigCfg.u8Sequence  = ADC_SEQ_A;
    stcAdcTrigCfg.enTrgSel    = AdcTrgsel_TRGX0;
    stcAdcTrigCfg.enInTrg0    = EVT_TMR61_GUDF;
    ADC_ConfigTriggerSrc(M4_ADC1, &stcAdcTrigCfg);
    ADC_TriggerSrcCmd(M4_ADC1,ADC_SEQ_A, Enable);
#endif

    /* Enable ADC1 sequence A interrupt */
    ADC_SeqITCmd(M4_ADC1, ADC_SEQ_A, Enable);

    /* Config ADC1 interrupt */
    stcAdcIrqCfg.enIntSrc    = INT_ADC1_EOCA;
    stcAdcIrqCfg.enIRQn      = Int004_IRQn;        ///< [Int000_IRQn, Int031_IRQn] [Int116_IRQn, Int121_IRQn] [Int142_IRQn]
    stcAdcIrqCfg.pfnCallback = &ADC1A_CallBack;
    enIrqRegResult = enIrqRegistration(&stcAdcIrqCfg);

    if (Ok != enIrqRegResult)
    {
        while (1u)
        {
            ;
        }
    }

    NVIC_ClearPendingIRQ(stcAdcIrqCfg.enIRQn);
    NVIC_SetPriority(stcAdcIrqCfg.enIRQn, DDL_IRQ_PRIORITY_03);
    NVIC_EnableIRQ(stcAdcIrqCfg.enIRQn);
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
    uint16_t                         u16Period;
    uint16_t                         u16Compare;
    stc_timer6_basecnt_cfg_t         stcTIM6BaseCntCfg;
    stc_timer6_port_output_cfg_t     stcTIM6PWMxCfg;
    stc_timer6_gcmp_buf_cfg_t        stcGCMPBufCfg;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    stc_timer6_deadtime_cfg_t        stcDeadTimeCfg;
    stc_timer6_validper_cfg_t        stcValidPerCfg;
    stc_timer6_spcl_buf_cfg_t        stcSpclBufCfg;
    stc_port_init_t                  stcPortInit;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcTIM6PWMxCfg);
    MEM_ZERO_STRUCT(stcGCMPBufCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcDeadTimeCfg);
    MEM_ZERO_STRUCT(stcValidPerCfg);
    MEM_ZERO_STRUCT(stcSpclBufCfg);
    MEM_ZERO_STRUCT(stcPortInit);

    /* Initialize Clock */
    BSP_CLK_Init();

    /* Initialize LED */
    BSP_LED_Init();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);

    PORT_SetFunc(PortA, Pin08, Func_Tim6, Disable);    //Timer61 PWMA
    PORT_SetFunc(PortE, Pin08, Func_Tim6, Disable);    //Timer61 PWMB


    stcTIM6BaseCntCfg.enCntMode   = Timer6CntTriangularModeA;           //Triangular wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;                     //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1;                     //Count clock: pclk
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);                           //timer6 PWM frequency, count mode and clk config

    u16Period = 0x8340u;
    Timer6_SetPeriod(M4_TMR61, Timer6PeriodA, u16Period);                //period set

    u16Compare = 0x3000u;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareA, u16Compare);  //Set General Compare RegisterA Value
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, u16Compare);  //Set General Compare RegisterC Value as buffer register of GCMAR

    /*PWMA/PWMB output buf config*/
    stcGCMPBufCfg.bEnGcmpTransBuf = true;
    stcGCMPBufCfg.enGcmpBufTransType = Timer6GcmpPrdSingleBuf;          //Single buffer transfer
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMA, &stcGCMPBufCfg);          //GCMAR buffer transfer set
    //Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMB, &stcGCMPBufCfg);        //GCMBR buffer transfer set


    u16Compare = 0x3000u;
    Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompA, u16Compare);
    Timer6_SetSpecialCmpValue(M4_TMR61, Timer6SpclCompC, u16Compare);

    stcSpclBufCfg.bEnSpclTransBuf    = true;
    stcSpclBufCfg.enSpclBufTransType = Timer6SpclSingleBuf;
    stcSpclBufCfg.enSpclBufOptType   = Timer6SplcOptUnderFlow;
    Timer6_SetSpecialBuf(M4_TMR61, Timer6SpclCompA, &stcSpclBufCfg);


    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMA port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMA port output inverse level when CNTER value match with GCMAR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMA output status is decide by STACA STPCA when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMA, &stcTIM6PWMxCfg);

    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMB port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMB port output inverse level when CNTER value match with GCMBR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMB output status is decide by STACB STPCB when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutHigh;      //PWMB port output set high level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMB port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMB, &stcTIM6PWMxCfg);

    Timer6_SetDeadTimeValue(M4_TMR61, Timer6DeadTimUpAR, 3360u);     // Set dead time value (up count)
    //Timer6_SetDeadTimeValue(M4_TMR61, Timer6DeadTimDwnAR, 3360u);  // Set dead time value (down count)

    stcDeadTimeCfg.bEnDeadtime     = true;  //Enable Hardware DeadTime
    stcDeadTimeCfg.bEnDtBufUp      = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtBufDwn     = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtEqualUpDwn = true;  //Make the down count dead time value equal to the up count dead time setting
    Timer6_ConfigDeadTime(M4_TMR61, &stcDeadTimeCfg);        // Hardware dead time function config

#ifdef SCMA_ValidPeriod
    /* Enable timer61 under flow interrupt */
    stcValidPerCfg.enValidCntNum = Timer6PeriodCnts1;  //Valid period: Enable every other one period
    stcValidPerCfg.enValidCdtEn = Timer6PeriodCnteMax; //Count condition: voer flow point of Triangular wave mode
    stcValidPerCfg.bPeriodSCMA  = true;
    Timer6_SetValidPeriod(M4_TMR61, &stcValidPerCfg);

    Timer6_ConfigIrq(M4_TMR61, Timer6INTENSAU, true);

#else

    /* Enable timer61 under flow interrupt */
    stcValidPerCfg.enValidCntNum = Timer6PeriodCnts1;  //Valid period: Enable every other one period
    stcValidPerCfg.enValidCdtEn = Timer6PeriodCnteMax; //Count condition: voer flow point of Triangular wave mode
    Timer6_SetValidPeriod(M4_TMR61, &stcValidPerCfg);

    //Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);
#endif

    /*config interrupt*/
    /* Enable timer61 under flow interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);

    stcIrqRegiConf.enIRQn = Int002_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GUDF;               //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = &Timer6_CallBack;           //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC

    Config_Adc();

    /*start timer6*/
    Timer6_StartCount(M4_TMR61);

    while(1)
    {

    }

}




/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
