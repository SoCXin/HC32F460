#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

uint16_t duty = 500;
void Timer6_UnderFlow_CallBack(void)
{
    static uint16_t cnt;
    if(duty >1000)
    {
        duty = 0;
    } 
    if(cnt>1000)
    {
        cnt = 0;
        duty++;
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareC, duty);
        Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareD, duty);
    }
    cnt++;
}
void user_timer6_init(void)
{
    uint16_t                         u16Compare;
    stc_timer6_basecnt_cfg_t         stcTIM6BaseCntCfg;
    stc_timer6_port_output_cfg_t     stcTIM6PWMxCfg;
    stc_timer6_gcmp_buf_cfg_t        stcGCMPBufCfg;
    stc_port_init_t                  stcPortInit;
    stc_irq_regi_conf_t              stcIrqRegiConf;
    
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM61,Enable);
    PORT_SetFunc(PortB, Pin13, Func_Tim6, Disable);
    PORT_SetFunc(PortA, Pin08, Func_Tim6, Disable);
    stcTIM6BaseCntCfg.enCntMode   = Timer6CntTriangularModeB;           //Triangular wave mode
    stcTIM6BaseCntCfg.enCntDir    = Timer6CntDirUp;                     //Counter counting up
    stcTIM6BaseCntCfg.enCntClkDiv = Timer6PclkDiv1;                     //Count clock: pclk
    Timer6_Init(M4_TMR61, &stcTIM6BaseCntCfg);
    Timer6_SetPeriod(M4_TMR61,Timer6PeriodA,1000);
    u16Compare = 100;
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareB, u16Compare);  //Set General Compare RegisterB Value
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareD, u16Compare);  //Set General Compare RegisterD Value as buffer register of GCMDR
    Timer6_SetGeneralCmpValue(M4_TMR61, Timer6GenCompareF, u16Compare);  //Set General Compare RegisterF Value as buffer register of GCMFR
    
     /*PWMA/PWMB output buffer config*/
    stcGCMPBufCfg.bEnGcmpTransBuf = true;
    stcGCMPBufCfg.enGcmpBufTransType = Timer6GcmpPrdSingleBuf;          //Double buffer transfer
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMA, &stcGCMPBufCfg);          //GCMAR buffer transfer set
    Timer6_SetGeneralBuf(M4_TMR61, Timer6PWMB, &stcGCMPBufCfg);          //GCMBR buffer transfer set
    
    stcTIM6PWMxCfg.enPortMode = Timer6ModeCompareOutput;    //Compare output function
    stcTIM6PWMxCfg.bOutEn     = true;                       //Output enable
    stcTIM6PWMxCfg.enPerc     = Timer6PWMxCompareKeep;      //PWMB port output keep former level when CNTER value match PERAR
    stcTIM6PWMxCfg.enCmpc     = Timer6PWMxCompareInv;       //PWMB port output inverse level when CNTER value match with GCMxR
    stcTIM6PWMxCfg.enStaStp   = Timer6PWMxStateSelSS;       //PWMB output status is decide by STACB STPCB when CNTER start and stop
    stcTIM6PWMxCfg.enStaOut   = Timer6PWMxPortOutLow;       //PWMB port output set low level when CNTER start
    stcTIM6PWMxCfg.enStpOut   = Timer6PWMxPortOutLow;       //PWMB port output set low level when CNTER stop
    stcTIM6PWMxCfg.enDisVal   = Timer6PWMxDisValLow;
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMB, &stcTIM6PWMxCfg);
    Timer6_PortOutputConfig(M4_TMR61, Timer6PWMA, &stcTIM6PWMxCfg);

    /*config interrupt*/
    /* Enable timer61 undf interrupt */
    Timer6_ConfigIrq(M4_TMR61, Timer6INTENUDF, true);
    
    stcIrqRegiConf.enIRQn = TIMER61_IRQn;                    //Register INT_TMR61_GUDF Int to Vect.No.002
    stcIrqRegiConf.enIntSrc = INT_TMR61_GUDF;               //Select I2C Error or Event interrupt function
    stcIrqRegiConf.pfnCallback = Timer6_UnderFlow_CallBack; //Callback function
    enIrqRegistration(&stcIrqRegiConf);                     //Registration IRQ
    
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);            //Clear Pending
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_03);//Set priority
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);                   //Enable NVIC
    
    
    /*start timer6*/
    Timer6_StartCount(M4_TMR61);
}


