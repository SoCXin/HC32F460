#include "User_TimerA_32bit.h"
uint32_t CapTimerL,CapTimerH;
static void TimerA_L16_Callback(void)
{
    M4_TMRA1->BCSTR_f.OVFF = 0;
    PORT_Toggle(PortE, Pin06);
}
static void TimerA_L16_Capture_Callback(void)
{
    CapTimerL = M4_TMRA1->CNTER;
    CapTimerH = M4_TMRA2->CNTER;
}
static void TimerA_H16_Callback(void)
{
    M4_TMRA2->BCSTR_f.OVFF = 0;
}
void TimerA_L16Bit_config(void)
{
   stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_hw_startup_cofig_t stcTimeraHwConfig;
    stc_timera_capture_init_t stcTimeraCaptureInit;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    MEM_ZERO_STRUCT(stcTimeraHwConfig);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcTimeraCaptureInit);
    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_L16_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);

    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH2_PORT, TIMERA_UNIT1_CH2_PIN, TIMERA_UNIT1_CH2_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv256;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 0x0FFF;        
    TIMERA_BaseInit(TIMERA_UnitL, &stcTimeraInit);


/* Configuration timera unit 1 capture structure */
    stcTimeraCaptureInit.enCapturePwmRisingEn = Enable;
    stcTimeraCaptureInit.enCapturePwmFallingEn = Disable;
    stcTimeraCaptureInit.enCaptureSpecifyEventEn = Enable;
    stcTimeraCaptureInit.enPwmClkDiv = TimeraFilterPclkDiv4;
    stcTimeraCaptureInit.enPwmFilterEn = Enable;
    stcTimeraCaptureInit.enCaptureTrigRisingEn = Disable;
    stcTimeraCaptureInit.enCaptureTrigFallingEn = Disable;
    stcTimeraCaptureInit.enTrigClkDiv = TimeraFilterPclkDiv1;
    stcTimeraCaptureInit.enTrigFilterEn = Disable;
    /* Enable channel 1 capture and interrupt */
    TIMERA_CaptureInit(TIMERA_UnitL, TIMERA_UNIT1_CH1, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(TIMERA_UnitL, TIMERA_UNIT1_CH1_INT, Enable);

    /* Enable channel 2 capture and interrupt */
    TIMERA_CaptureInit(TIMERA_UnitL, TIMERA_UNIT1_CH2, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(TIMERA_UnitL, TIMERA_UNIT1_CH2_INT, Enable);

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UnitL, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNITL_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = TIMERA1_IRQn;
    stcIrqRegiConf.pfnCallback = TimerA_L16_Callback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    
    /* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_COMPARE_INT;
    stcIrqRegiConf.enIRQn = Int022_IRQn;
    stcIrqRegiConf.pfnCallback = TimerA_L16_Capture_Callback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure timera unit 1 startup */
    stcTimeraHwConfig.enSpecifyEventStartupEn = Enable;
    stcTimeraHwConfig.enTrigFallingStartupEn = Disable;
    stcTimeraHwConfig.enTrigRisingStartupEn = Disable;
    TIMERA_HwStartupConfig(TIMERA_UnitL, &stcTimeraHwConfig);

//    TIMERA_Cmd(TIMERA_UnitL,Enable);
}

void TimerA_H16Bit_config(void)
{
   stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_hw_startup_cofig_t stcTimeraHwConfig;
    stc_port_init_t stcPortInit;
    stc_timera_orthogonal_coding_init_t stcTimerORthoCFG;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    MEM_ZERO_STRUCT(stcTimeraHwConfig);
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcTimerORthoCFG);
    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_H16_CLOCK, Enable);
//    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS, Enable);//AOS时钟

    /* Configuration TIMERA compare pin */
//    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
//    PORT_SetFunc(TIMERA_UNIT1_CH3_PORT, TIMERA_UNIT1_CH3_PIN, TIMERA_UNIT1_CH3_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
//    stcTimeraInit.enClkDiv = TimeraPclkDiv1;//此处时钟无效，不需要配置
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Enable;
    stcTimeraInit.u16PeriodVal = 10;        //freq: 100Hz
//    
    TIMERA_BaseInit(TIMERA_UnitH, &stcTimeraInit);
    
    /* Configuration timera unit 1 compare structure */
//    stcTimerCompareInit.u16CompareVal = stcTimeraInit.u16PeriodVal * 4 / 5;
//    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;
//    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
//    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputHigh;
//    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputLow;
//    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
//    stcTimerCompareInit.enCacheEn = Disable;
//    stcTimerCompareInit.enTriangularTroughTransEn = Enable;
//    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
//    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
//    TIMERA_CompareInit(TIMERA_UnitH, TIMERA_UNIT1_CH1, &stcTimerCompareInit);
//    TIMERA_CompareCmd(TIMERA_UnitH, TIMERA_UNIT1_CH1, Enable);

//    /* Configure channel 3 */
//    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;//TimeraCountStartOutputHigh;
//    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;//TimeraCountStopOutputHigh;
//    TIMERA_CompareInit(TIMERA_UnitH, TIMERA_UNIT1_CH3, &stcTimerCompareInit);
//    TIMERA_CompareCmd(TIMERA_UnitH, TIMERA_UNIT1_CH3, Enable);

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UnitH, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNITH_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = TIMERA2_IRQn;
    stcIrqRegiConf.pfnCallback = TimerA_H16_Callback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure timera unit 1 startup */
//    stcTimeraHwConfig.enSpecifyEventStartupEn = Enable;
//    stcTimeraHwConfig.enTrigFallingStartupEn = Disable;
//    stcTimeraHwConfig.enTrigRisingStartupEn = Disable;
//    TIMERA_HwStartupConfig(TIMERA_UnitL, &stcTimeraHwConfig);
    stcTimerORthoCFG.enIncAnotherUnitOverflowEn = Enable;
    TIMERA_OrthogonalCodingInit(TIMERA_UnitH,&stcTimerORthoCFG);

    TIMERA_Cmd(TIMERA_UnitH,Enable);
}




















