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
 **   - 2018-11-30  CDT  first version for Device Driver Library of Timer6.
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

void Timer6_UnderFlow_CallBack(void)
{
    static uint8_t i;

    if( 0u == i)
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x2000u);
        Timer6_SetGeneralCmpValue(M4_TMR62, Timer6GenCompareC, 0x4000u);
        Timer6_SetGeneralCmpValue(M4_TMR63, Timer6GenCompareC, 0x6000u);
        i = 1u;
    }
    else
    {
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, 0x6000u);
        Timer6_SetGeneralCmpValue(M4_TMR62, Timer6GenCompareC, 0x4000u);
        Timer6_SetGeneralCmpValue(M4_TMR63, Timer6GenCompareC, 0x2000u);
        i = 0u;
    }
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
    stc_timer6_sw_sync_t             stcSwSyncCfg;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcTIM6PWMxCfg);
    MEM_ZERO_STRUCT(stcGCMPBufCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcDeadTimeCfg);
    MEM_ZERO_STRUCT(stcSwSyncCfg);

    /* Initialize Clock */
    BSP_CLK_Init();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM62, Enable);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM63, Enable);

    PORT_SetFunc(PortA, Pin08, Func_Tim6, Disable);    //Timer61 PWMA
    PORT_SetFunc(PortE, Pin08, Func_Tim6, Disable);    //Timer61 PWMB

    PORT_SetFunc(PortE, Pin11, Func_Tim6, Disable);    //Timer62 PWMA
    PORT_SetFunc(PortE, Pin10, Func_Tim6, Disable);    //Timer62 PWMB

    PORT_SetFunc(PortE, Pin13, Func_Tim6, Disable);    //Timer63 PWMA
    PORT_SetFunc(PortE, Pin12, Func_Tim6, Disable);    //Timer63 PWMB

    stcTIM6BaseCntCfg.enCntMode   = Timer6CntTriangularModeA;           //Triangular wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;                     //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1;                     //Count clock: pclk
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);                       //timer6 PWM frequency, count mode and clk config
    Timer6_Init(M4_TMR62, &stcTIM6BaseCntCfg);                       //timer6 PWM frequency, count mode and clk config
    Timer6_Init(M4_TMR63, &stcTIM6BaseCntCfg);                       //timer6 PWM frequency, count mode and clk config

    u16Period = 0x8340u;
    Timer6_SetPeriod(M4_TMR61, Timer6PeriodA, u16Period);                //period set
    Timer6_SetPeriod(M4_TMR62, Timer6PeriodA, u16Period);                //period set
    Timer6_SetPeriod(M4_TMR63, Timer6PeriodA, u16Period);                //period set

    u16Compare = 0x3000u;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareA, u16Compare);  //Set General Compare RegisterA Value
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, u16Compare);  //Set General Compare RegisterC Value as buffer register of GCMAR

    Timer6_SetGeneralCmpValue(M4_TMR62, Timer6GenCompareA, u16Compare);  //Set General Compare RegisterA Value
    Timer6_SetGeneralCmpValue(M4_TMR62, Timer6GenCompareC, u16Compare);  //Set General Compare RegisterC Value as buffer register of GCMAR

    Timer6_SetGeneralCmpValue(M4_TMR63, Timer6GenCompareA, u16Compare);  //Set General Compare RegisterA Value
    Timer6_SetGeneralCmpValue(M4_TMR63, Timer6GenCompareC, u16Compare);  //Set General Compare RegisterC Value as buffer register of GCMAR

    u16Compare = 0x6000u;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareB, u16Compare);  //Set General Compare RegisterB Value
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareD, u16Compare);  //Set General Compare RegisterD Value as buffer register of GCMDR

    Timer6_SetGeneralCmpValue(M4_TMR62, Timer6GenCompareB, u16Compare);  //Set General Compare RegisterB Value
    Timer6_SetGeneralCmpValue(M4_TMR62, Timer6GenCompareD, u16Compare);  //Set General Compare RegisterD Value as buffer register of GCMDR

    Timer6_SetGeneralCmpValue(M4_TMR63, Timer6GenCompareB, u16Compare);  //Set General Compare RegisterB Value
    Timer6_SetGeneralCmpValue(M4_TMR63, Timer6GenCompareD, u16Compare);  //Set General Compare RegisterD Value as buffer register of GCMDR


    /*PWMA/PWMB output buf config*/
    stcGCMPBufCfg.bEnGcmpTransBuf = true;
    stcGCMPBufCfg.enGcmpBufTransType = Timer6GcmpPrdSingleBuf;          //Single buffer transfer
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMA, &stcGCMPBufCfg);          //GCMAR buffer transfer set
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMB, &stcGCMPBufCfg);          //GCMBR buffer transfer set

    Timer6_SetGeneralBuf(M4_TMR62, Timer6PWMA, &stcGCMPBufCfg);          //GCMAR buffer transfer set
    Timer6_SetGeneralBuf(M4_TMR62, Timer6PWMB, &stcGCMPBufCfg);          //GCMBR buffer transfer set

    Timer6_SetGeneralBuf(M4_TMR63, Timer6PWMA, &stcGCMPBufCfg);          //GCMAR buffer transfer set
    Timer6_SetGeneralBuf(M4_TMR63, Timer6PWMB, &stcGCMPBufCfg);          //GCMBR buffer transfer set


    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMA port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMA port output inverse level when CNTER value match with GCMAR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMA output status is decide by STACA STPCA when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMA port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMA, &stcTIM6PWMxCfg);
    Timer6_PortOutputConfig(M4_TMR62, Timer6PWMA, &stcTIM6PWMxCfg);
    Timer6_PortOutputConfig(M4_TMR63, Timer6PWMA, &stcTIM6PWMxCfg);

    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMB port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMB port output inverse level when CNTER value match with GCMBR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMB output status is decide by STACB STPCB when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutHigh;      //PWMB port output set high level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMB port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMB, &stcTIM6PWMxCfg);
    Timer6_PortOutputConfig(M4_TMR62, Timer6PWMB, &stcTIM6PWMxCfg);
    Timer6_PortOutputConfig(M4_TMR63, Timer6PWMB, &stcTIM6PWMxCfg);

    Timer6_SetDeadTimeValue(M4_TMR61, Timer6DeadTimUpAR, 3360u);     // Set dead time value (up count)
    //Timer6_SetDeadTimeValue(M4_TMR61, Timer6DeadTimDwnAR, 3360u);  // Set dead time value (down count)
    Timer6_SetDeadTimeValue(M4_TMR62, Timer6DeadTimUpAR, 3360u);     // Set dead time value (up count)
    //Timer6_SetDeadTimeValue(M4_TMR62, Timer6DeadTimDwnAR, 3360u);  // Set dead time value (down count)
    Timer6_SetDeadTimeValue(M4_TMR63, Timer6DeadTimUpAR, 3360u);     // Set dead time value (up count)
    //Timer6_SetDeadTimeValue(M4_TMR63, Timer6DeadTimDwnAR, 3360u);  // Set dead time value (down count)

    stcDeadTimeCfg.bEnDeadtime     = true;  //Enable Hardware DeadTime
    stcDeadTimeCfg.bEnDtBufUp      = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtBufDwn     = false; //Disable buffer transfer
    stcDeadTimeCfg.bEnDtEqualUpDwn = true;  //Make the down count dead time value equal to the up count dead time setting
    Timer6_ConfigDeadTime(M4_TMR61, &stcDeadTimeCfg);        // Hardware dead time function config
    Timer6_ConfigDeadTime(M4_TMR62, &stcDeadTimeCfg);        // Hardware dead time function config
    Timer6_ConfigDeadTime(M4_TMR63, &stcDeadTimeCfg);        // Hardware dead time function config

    /*config interrupt*/
    /* Enable timer61 under flow interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);

    stcIrqRegiConf.enIRQn = Int002_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GUDF;               //Select Event interrupt function
    stcIrqRegiConf.pfnCallback = &Timer6_UnderFlow_CallBack; //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC

    stcSwSyncCfg.bTimer61 = true;
    stcSwSyncCfg.bTimer62 = true;
    stcSwSyncCfg.bTimer63 = true;

    while(1)
    {
        Timer6_SwSyncStart(&stcSwSyncCfg);
        Ddl_Delay1ms(1000ul);

        Timer6_SwSyncStop(&stcSwSyncCfg);
        Ddl_Delay1ms(1000ul);

        Timer6_SwSyncClear(&stcSwSyncCfg);
        Ddl_Delay1ms(1000ul);
    }

}




/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
