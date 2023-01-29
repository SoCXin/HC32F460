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
//uint16_t u16Timer6Cnt0 = 0u;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static volatile uint16_t u16Timer6Cnt0 = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
static void genClkIn(void)
{
    uint32_t i;

    uint8_t bAin[17] = {0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1};
    uint8_t bBin[17] = {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1};


    for (i = 0ul; i < 17ul; i++)
    {
        u16Timer6Cnt0 = Timer6_GetCount(M4_TMR61);//M4_TMR61->CNTER;

        M4_PORT->PODRE_f.POUT06 = bAin[i];
        M4_PORT->PODRD_f.POUT07 = bBin[i];

        Ddl_Delay1ms(1000ul);
    }

    u16Timer6Cnt0 = Timer6_GetCount(M4_TMR61);//M4_TMR60->CNTER;
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
    stc_timer6_basecnt_cfg_t         stcTIM6BaseCntCfg;
    stc_port_init_t                  stcPortInit;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    stc_timer6_port_input_cfg_t      stcPortInputCfg;

    MEM_ZERO_STRUCT(stcTIM6BaseCntCfg);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInputCfg);

    /* Initialize Clock */
    BSP_CLK_Init();

    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61, Enable);

    PORT_Unlock();
    PORT_OE(PortE, Pin06, Enable);
    PORT_OE(PortD, Pin07, Enable);

    PORT_SetFunc(PortA, Pin08, Func_Tim6, Disable);    //Timer61 PWMA
    PORT_SetFunc(PortE, Pin08, Func_Tim6, Disable);    //Timer61 PWMB

    stcTIM6BaseCntCfg.enCntMode   = Timer6CntSawtoothMode;      //Sawtooth wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;             //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1;             //Count clock: pclk
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);                  //timer6 PWM frequency, count mode and clk config

    stcPortInputCfg.enPortSel  = Timer6xCHA;                    //Capture Input Port: PWM A port
    stcPortInputCfg.enPortMode = Timer6ModeCaptureInput;        //Capture input function
    stcPortInputCfg.bFltEn     = true;                          //Input filter enable
    stcPortInputCfg.enFltClk   = Timer6FltClkPclk0Div64;        //Filter clock
    Timer6_PortInputConfig(M4_TMR61, &stcPortInputCfg);         //Input config

    stcPortInputCfg.enPortSel  = Timer6xCHB;                    //Capture Input Port: PWM A port
    stcPortInputCfg.enPortMode = Timer6ModeCaptureInput;        //Capture input function
    stcPortInputCfg.bFltEn     = true;                          //Input filter enable
    stcPortInputCfg.enFltClk   = Timer6FltClkPclk0Div64;        //Filter clock
    Timer6_PortInputConfig(M4_TMR61, &stcPortInputCfg);         //Input config

    while(1)
    {
        M4_PORT->PODRE_f.POUT06 = 0u;
        M4_PORT->PODRD_f.POUT07 = 1u;

        Ddl_Delay1ms(1000ul);

        Timer6_ClearHwCntUp(M4_TMR61);
        Timer6_ClearHwCntDwn(M4_TMR61);

        Timer6_ConfigHwCntUp(M4_TMR61, Timer6HwCntPWMBHighPWMARise);   // PWMA Rising trigger when PWMB is high level
        Timer6_ConfigHwCntDwn(M4_TMR61, Timer6HwCntPWMBLowPWMARise);   // PWMA Rising trigger when PWMB is low level

        Timer6_StartCount(M4_TMR61);

        genClkIn();

        Timer6_StopCount(M4_TMR61);

        Timer6_ClearCount(M4_TMR61);

        M4_PORT->PODRE_f.POUT06 = 0u;
        M4_PORT->PODRD_f.POUT07 = 1u;

        Ddl_Delay1ms(2000ul);

        Timer6_ClearHwCntUp(M4_TMR61);
        Timer6_ClearHwCntDwn(M4_TMR61);

        Timer6_ConfigHwCntUp(M4_TMR61, Timer6HwCntPWMBHighPWMARise);   // PWMA Rising trigger when PWMB is high level
        Timer6_ConfigHwCntUp(M4_TMR61, Timer6HwCntPWMBLowPWMAFall);    // PWMA falling trigger when PWMB is low level

        Timer6_ConfigHwCntDwn(M4_TMR61, Timer6HwCntPWMAHighPWMBRise);  // PWMB Rising trigger when PWMA is high level
        Timer6_ConfigHwCntDwn(M4_TMR61, Timer6HwCntPWMALowPWMBFall);   // PWMB falling trigger when PWMA is low level

        Timer6_StartCount(M4_TMR61);

        genClkIn();

        Timer6_StopCount(M4_TMR61);

        Timer6_ClearCount(M4_TMR61);

    }

}




/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
