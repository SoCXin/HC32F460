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

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint16_t u16CaptureA;
uint16_t u16CaptureC;
uint16_t u16CaptureE;

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

void Timer61_OverFlow_CallBack(void)
{
    static uint8_t i;

    if( 0u == i)
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareE, 0x4000u);
        i = 1u;
    }
    else if(1u == i)
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareE, 0x6000u);
        i = 2u;
    }
    else
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareE, 0x2000u);
        i = 0u;
    }
}


void Timer62_CapInputCallBack(void)
{
    u16CaptureA = Timer6_GetGeneralCmpValue(M4_TMR62, Timer6GenCompareA);
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
    stc_timer6_port_input_cfg_t      stcTIM6CapxCfg;
    stc_timer6_gcmp_buf_cfg_t        stcGCMPBufCfg;
    stc_port_init_t                  stcPortInit;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    stc_timer6_sw_sync_t             stcSwSyncStart;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcTIM6PWMxCfg);
    MEM_ZERO_STRUCT(stcTIM6CapxCfg);
    MEM_ZERO_STRUCT(stcGCMPBufCfg);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcSwSyncStart);

    /* Initialize Clock */
    BSP_CLK_Init();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM62, Enable);

    PORT_SetFunc(PortA, Pin08, Func_Tim6, Disable);    //Timer61 PWMA
    //PORT_SetFunc(PortE, Pin08, Func_Tim6, Disable);  //Timer61 PWMB

    PORT_SetFunc(PortE, Pin11, Func_Tim6, Disable);    //Timer62 PWMA
    //PORT_SetFunc(PortE, Pin10, Func_Tim6, Disable);  //Timer62 PWMB

    //PORT_SetFunc(PortE, Pin13, Func_Tim6, Disable);  //Timer63 PWMA
    //PORT_SetFunc(PortB, Pin15, Func_Tim6, Disable);  //Timer63 PWMB


    /******************Config Timer61 as output signal*************************/

    stcTIM6BaseCntCfg.enCntMode   = Timer6CntSawtoothMode;              //Sawtooth wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;                     //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1024;                  //Count clock: pclk0/1024
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);                          //timer6 PWM frequency, count mode and clk config

    u16Period = 0x8000u;
    Timer6_SetPeriod(M4_TMR61, Timer6PeriodA, u16Period);                //period set

    u16Compare = 0x2000u;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareA, u16Compare);  //Set General Compare RegisterA Value
    u16Compare = 0x4000u;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, u16Compare);  //Set General Compare RegisterC Value as buffer register of GCMAR
    u16Compare = 0x6000u;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareE, u16Compare);  //Set General Compare RegisterE Value as buffer register of GCMCR

    /*PWMA/PWMB output buffer config*/
    stcGCMPBufCfg.bEnGcmpTransBuf = true;
    stcGCMPBufCfg.enGcmpBufTransType = Timer6GcmpPrdDoubleBuf;          //Double buffer transfer
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMA, &stcGCMPBufCfg);         //GCMAR buffer transfer set

    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareLow;       //PWMA port output low level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareHigh;      //PWMA port output high level when CNTER value match with GCMAR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMA output status is decide by STACA STPCA when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMA, &stcTIM6PWMxCfg); //Output config

    /*config interrupt*/
    /* Enable timer61 under flow interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENOVF, true);

    stcIrqRegiConf.enIRQn = Int002_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GOVF;               //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = &Timer61_OverFlow_CallBack; //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC


    /******************Config Timer62 as output signal*************************/

    stcTIM6BaseCntCfg.enCntMode   = Timer6CntSawtoothMode;              //Sawtooth wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;                     //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1024;                  //Count clock: pclk0/1024
    Timer6_Init(M4_TMR62, &stcTIM6BaseCntCfg);                          //timer6 PWM frequency, count mode and clk config

    u16Period = 0xFFFFu;
    Timer6_SetPeriod(M4_TMR62, Timer6PeriodA, u16Period);               //Period set

    stcTIM6CapxCfg.enPortSel  = Timer6xCHA;                            //Capture Input Port: PWM A port
    stcTIM6CapxCfg.enPortMode = Timer6ModeCaptureInput;                //Capture input function
    stcTIM6CapxCfg.bFltEn     = true;                                  //Input filter enable
    stcTIM6CapxCfg.enFltClk   = Timer6FltClkPclk0Div16;                //Filter clock
    Timer6_PortInputConfig(M4_TMR62, &stcTIM6CapxCfg);                 //Input config


    Timer6_ConfigHwCaptureA(M4_TMR62, Timer6HwTrigPWMARise);       //HW Capture: Timer6 PWMA port rise trig

    Timer6_ConfigHwClear(M4_TMR62, Timer6HwTrigPWMAFall);          //HW Clear: Timer6 PWMA port fall trig
    Timer6_EnableHwClear(M4_TMR62);

    /*config interrupt*/
    /* Enable timer62 GCMAR interrupt */
    Timer6_ConfigIrq(M4_TMR62, Timer6INTENA, true);

    stcIrqRegiConf.enIRQn = Int003_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR62_GCMA;               //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = &Timer62_CapInputCallBack;  //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_04);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC


    /*start timer6*/
    stcSwSyncStart.bTimer61 = true;
    stcSwSyncStart.bTimer62 = true;
    Timer6_SwSyncStart(&stcSwSyncStart);

    while(1)
    {
        ;
    }

}




/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
