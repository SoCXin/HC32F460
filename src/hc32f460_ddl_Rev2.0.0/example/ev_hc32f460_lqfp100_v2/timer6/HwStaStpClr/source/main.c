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

void Timer6_OverFlow_CallBack(void)
{
    PORT_Toggle(PortB, Pin05);
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
    stc_timer6_basecnt_cfg_t         stcTIM6BaseCntCfg;
    stc_timer6_port_output_cfg_t     stcTIM6PWMxCfg;
    stc_timer6_gcmp_buf_cfg_t        stcGCMPBufCfg;
    stc_port_init_t                  stcPortInit;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    stc_timer6_port_input_cfg_t      stcPortInputCfg;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcTIM6PWMxCfg);
    MEM_ZERO_STRUCT(stcGCMPBufCfg);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInputCfg);

    /* Initialize Clock */
    BSP_CLK_Init();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);

    PORT_SetFunc(PortA, Pin08, Func_Tim6, Disable);    //Timer61 PWMA
    PORT_SetFunc(PortE, Pin08, Func_Tim6, Disable);    //Timer61 PWMB

    PORT_Unlock();
    PORT_OE(PortE, Pin06, Enable);
    PORT_OE(PortD, Pin07, Enable);
    PORT_OE(PortB, Pin05, Enable);


    stcTIM6BaseCntCfg.enCntMode   = Timer6CntSawtoothMode;      //Sawtooth wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;             //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1;             //Count clock: pclk
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);                  //timer6 PWM frequency, count mode and clk config

    u16Period = 0x8340u;
    Timer6_SetPeriod(M4_TMR61, Timer6PeriodA, u16Period);        //period set

    stcPortInputCfg.enPortSel  = Timer6xCHA;
    stcPortInputCfg.enPortMode = Timer6ModeCaptureInput;
    stcPortInputCfg.bFltEn     = true;
    stcPortInputCfg.enFltClk   = Timer6FltClkPclk0Div16;
    Timer6_PortInputConfig(M4_TMR61, &stcPortInputCfg);          //PWMA port set as input mode

    stcPortInputCfg.enPortSel  = Timer6xCHB;
    stcPortInputCfg.enPortMode = Timer6ModeCaptureInput;
    stcPortInputCfg.bFltEn     = true;
    stcPortInputCfg.enFltClk   = Timer6FltClkPclk0Div16;
    Timer6_PortInputConfig(M4_TMR61, &stcPortInputCfg);          //PWMB port set as input mode

    Timer6_ConfigHwStart(M4_TMR61, Timer6HwTrigPWMARise);        //HW START: PWMA Rising Input
    Timer6_ConfigHwStop(M4_TMR61, Timer6HwTrigPWMAFall);         //HW START: PWMA falling Input
    Timer6_ConfigHwClear(M4_TMR61, Timer6HwTrigPWMBRise);        //HW START: PWMB Rising Input

    Timer6_EnableHwStart(M4_TMR61);
    Timer6_EnableHwStop(M4_TMR61);
    Timer6_EnableHwClear(M4_TMR61);

    /*config interrupt*/
    /* Enable timer61 undf interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENOVF, true);

    stcIrqRegiConf.enIRQn = Int002_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GOVF;               //Select I2C Error or Event interrupt function
    stcIrqRegiConf.pfnCallback = &Timer6_OverFlow_CallBack;  //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ

    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC


    /*start timer6*/
    //Timer6_StartCount(M4_TMR61);

    while(1)
    {
        Ddl_Delay1ms(1000ul);

        M4_PORT->PODRE_f.POUT06 = 1u;  //Hw start Timer61

        Ddl_Delay1ms(2000ul);

        M4_PORT->PODRE_f.POUT06 = 0u;  //Hw stop Timer61
        M4_PORT->PODRB_f.POUT05 = 0u;

        Ddl_Delay1ms(2000ul);

        M4_PORT->PODRD_f.POUT07 = 1u;  //Hw Clear Timer61 CNTER

        Ddl_Delay1ms(500ul);

        M4_PORT->PODRD_f.POUT07 = 0u;
    }

}




/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
