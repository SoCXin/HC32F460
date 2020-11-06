#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* LED0 Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED0_TOGGLE()                   PORT_Toggle(LED0_PORT, LED0_PIN)

/* KEY0 Port/Pin definition */
#define KEY0_PORT                       PortD
#define KEY0_PIN                        Pin03

/* KEY1 Port/Pin definition */
#define KEY1_PORT                       PortD
#define KEY1_PIN                        Pin04
#define KEY1_TRIGGER_EVENT              EVT_PORT_EIRQ4
/* TIMERA unit and clock definition */
#define TIMERA_UNIT1                    M4_TMRA1
#define TIMERA_UNIT1_CLOCK              PWC_FCG2_PERIPH_TIMA1
#define TIMERA_UNIT1_OVERFLOW_INT       INT_TMRA1_OVF

/* TIMERA channel 1 Port/Pin definition */
#define TIMERA_UNIT1_CH1                TimeraCh1
#define TIMERA_UNIT1_CH1_PORT           PortE
#define TIMERA_UNIT1_CH1_PIN            Pin09
#define TIMERA_UNIT1_CH1_FUNC           Func_Tima0

/* TIMERA channel 3 Port/Pin definition */
#define TIMERA_UNIT1_CH3                TimeraCh3
#define TIMERA_UNIT1_CH3_PORT           PortE
#define TIMERA_UNIT1_CH3_PIN            Pin13
#define TIMERA_UNIT1_CH3_FUNC           Func_Tima0

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint32_t u8ExIntFlag = 0, u8TmraUnit1Cnt = 0;

/**
 *******************************************************************************
 ** \brief ExtInt3 callback function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
//void ExtInt03_Callback(void)
//{
//    if (Set == EXINT_IrqFlgGet(ExtiCh03))
//    {
//        u8ExIntFlag = 1u;
//        EXINT_IrqFlgClr(ExtiCh03);
//    }
//}

///**
// *******************************************************************************
// ** \brief KEY0(SW2) init function
// **
// ** \param [in] None
// **
// ** \retval None
// **
// ******************************************************************************/
//void Sw2_Init(void)
//{
//    stc_port_init_t stcPortInit;
//    stc_exint_config_t stcExtiConfig;
//    stc_irq_regi_conf_t stcIrqRegiConf;

//    /* configure structure initialization */
//    MEM_ZERO_STRUCT(stcPortInit);
//    MEM_ZERO_STRUCT(stcExtiConfig);
//    MEM_ZERO_STRUCT(stcIrqRegiConf);

//    /* Set PD03 as External Int Ch.3 input */
//    stcPortInit.enExInt = Enable;
//    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

//    stcExtiConfig.enExitCh = ExtiCh03;
//    /* Filter setting */
//    stcExtiConfig.enFilterEn = Enable;
//    stcExtiConfig.enFltClk = Pclk3Div8;
//    /* Both edge */
//    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
//    EXINT_Init(&stcExtiConfig);

//    /* Select External Int Ch.3 */
//    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
//    /* Register External Int to Vect.No.007 */
//    stcIrqRegiConf.enIRQn = Int007_IRQn;
//    /* Callback function */
//    stcIrqRegiConf.pfnCallback = ExtInt03_Callback;
//    /* Registration IRQ */
//    enIrqRegistration(&stcIrqRegiConf);

//    /* Clear pending */
//    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    /* Set priority */
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    /* Enable NVIC */
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
//    stcPortInit.enPinMode = Pin_Mode_Out;
//    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
//}

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Timera unit 1 callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void TimeraUnit1_IrqCallback(void)
{
    u8TmraUnit1Cnt++;
    if (u8TmraUnit1Cnt >= 2000)      //1s
    {
        u8TmraUnit1Cnt = 0u;
        
        //LED0_TOGGLE();
    }
    TIMERA_SetCompareValue(TIMERA_UNIT1,TIMERA_UNIT1_CH1,u8TmraUnit1Cnt);
    TIMERA_SetCompareValue(TIMERA_UNIT1,TIMERA_UNIT1_CH3,u8TmraUnit1Cnt);
    TIMERA_ClearFlag(TIMERA_UNIT1, TimeraFlagOverflow);
}

void TimerA_config(void)
{
   stc_timera_base_init_t stcTimeraInit;
    stc_timera_compare_init_t stcTimerCompareInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_timera_hw_startup_cofig_t stcTimeraHwConfig;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimerCompareInit);
    MEM_ZERO_STRUCT(stcTimeraHwConfig);
    MEM_ZERO_STRUCT(stcPortInit);

    /* Configuration peripheral clock */
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Configuration TIMERA compare pin */
    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH3_PORT, TIMERA_UNIT1_CH3_PIN, TIMERA_UNIT1_CH3_FUNC, Disable);

    /* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv1024;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = 0x7D0;        //freq: 100Hz
    TIMERA_BaseInit(TIMERA_UNIT1, &stcTimeraInit);

    /* Configuration timera unit 1 compare structure */
    stcTimerCompareInit.u16CompareVal = stcTimeraInit.u16PeriodVal * 4 / 5;
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;
    stcTimerCompareInit.enCompareMatchOutput = TimeraCompareMatchOutputHigh;
    stcTimerCompareInit.enPeriodMatchOutput = TimeraPeriodMatchOutputLow;
    stcTimerCompareInit.enSpecifyOutput = TimeraSpecifyOutputInvalid;
    stcTimerCompareInit.enCacheEn = Disable;
    stcTimerCompareInit.enTriangularTroughTransEn = Enable;
    stcTimerCompareInit.enTriangularCrestTransEn = Disable;
    stcTimerCompareInit.u16CompareCacheVal = stcTimerCompareInit.u16CompareVal;
    /* Configure Channel 1 */
    TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH1, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH1, Enable);

    /* Configure channel 3 */
    stcTimerCompareInit.enStartCountOutput = TimeraCountStartOutputLow;//TimeraCountStartOutputHigh;
    stcTimerCompareInit.enStopCountOutput = TimeraCountStopOutputLow;//TimeraCountStopOutputHigh;
    TIMERA_CompareInit(TIMERA_UNIT1, TIMERA_UNIT1_CH3, &stcTimerCompareInit);
    TIMERA_CompareCmd(TIMERA_UNIT1, TIMERA_UNIT1_CH3, Enable);

    /* Enable period count interrupt */
    TIMERA_IrqCmd(TIMERA_UNIT1, TimeraIrqOverflow, Enable);
    /* Interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_OVERFLOW_INT;
    stcIrqRegiConf.enIRQn = TIMERA1_IRQn;
    stcIrqRegiConf.pfnCallback = TimeraUnit1_IrqCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Configure timera unit 1 startup */
    stcTimeraHwConfig.enSpecifyEventStartupEn = Enable;
    stcTimeraHwConfig.enTrigFallingStartupEn = Disable;
    stcTimeraHwConfig.enTrigRisingStartupEn = Disable;
    TIMERA_HwStartupConfig(TIMERA_UNIT1, &stcTimeraHwConfig);

    /* Set external Int Ch.4 trigger timera startup */
//    stcPortInit.enExInt = Enable;
//    PORT_Init(KEY1_PORT, KEY1_PIN, &stcPortInit);
//    TIMERA_SetCountTriggerSrc(KEY1_TRIGGER_EVENT);
//    Sw2_Init();
//    LED0_OFF();
    M4_TMRA1->BCSTR_f.START = Enable;
    
}
