#include "User_Timer.h"
//#include "cmsis_os.h"
/* Define Timer Unit for example */
#define TMR_UNIT            M4_TMR02
#define TMR_INI_GCMA        INT_TMR02_GCMA
#define TMR_INI_GCMB        INT_TMR02_GCMB

#define TMR01_UNIT            M4_TMR01
#define TMR01_INI_GCMA        INT_TMR01_GCMA
#define TMR01_INI_GCMB        INT_TMR01_GCMB
#define TIMER02_B_IRQn          Int007_IRQn
#define TIMER01_B_IRQn          Int000_IRQn
volatile bool TIMER0_TIF_FLAG = 0;

static void Timer02A_CallBack(void)
{
    
}
static void Timer02B_CallBack(void)
{
    TIMER0_TIF_FLAG = 1;
}

void Timer02_Init(void)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
////        ENABLE_TMR0();
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM02,Enable);
//    /*config register for channel A */
//    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
//    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
//    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv0;
//    stcTimerCfg.Tim0_CmpValue = 32000/4 - 1;
//    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelA,&stcTimerCfg);

//    /* Enable channel A interrupt */
//    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelA,Enable);
//    /* Register TMR_INI_GCMA Int to Vect.No.001 */
//    stcIrqRegiConf.enIRQn = Int001_IRQn;
//    /* Select I2C Error or Event interrupt function */
//    stcIrqRegiConf.enIntSrc = TMR_INI_GCMA;
//    /* Callback function */
//    stcIrqRegiConf.pfnCallback = Timer0A_CallBack;
//    /* Registration IRQ */
//    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
//    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    /* Set priority */
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    /* Enable NVIC */
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /*config register for channel B */
    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv4;
    stcTimerCfg.Tim0_AsyncClockSource = Tim0_LRC;
    stcTimerCfg.Tim0_CmpValue = 21;
    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelB,&stcTimerCfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = TIMER02_B_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TMR_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = Timer02B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    /*start timer0*/
//    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelA,Enable);
    
}
/**
 *******************************************************************************
 ** \brief Timer02B_Start
 **
 ** \param  Count 定时计数值 单位1us
 **
 ** \retval None
 **
 ******************************************************************************/
en_result_t Timer02B_Start(uint16_t Count)
{
    if(Count<3120)
    {
        TMR_UNIT->CMPBR_f.CMPB = Count*21;
    }
    else
    {
        return Error;
    }
    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelB,Enable);
    return Ok;
}
/**
 *******************************************************************************
 ** \brief Timer02B_Stop
 **
 ** \param  None
 **
 ** \retval Ok/Error
 **
 ******************************************************************************/
en_result_t Timer02B_Stop(void)
{
    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelB,Disable);
}
en_result_t Timer_Delay_us(uint16_t us)
{
    TIMER0_TIF_FLAG = 0;
    if(us<=1000)
    {
        Timer02B_Start(us);
    }
    else
    {
        return Error;
    }
    while(TIMER0_TIF_FLAG ==0);
    Timer02B_Stop();
    return Ok;
}
void Delay_ms(uint16_t ms)
{
    while(ms--)
    {
        Timer_Delay_us(1000);
    }
}
void Delay_us(uint16_t us)
{
    Timer_Delay_us(us);
}


static void Timer01A_CallBack(void)
{
    
}
static void Timer01B_CallBack(void)
{
    TIMER0_TIF_FLAG = 1;
}

void Timer01_Init(void)
{
    stc_tim0_base_init_t stcTimerCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;
////        ENABLE_TMR0();
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM01,Enable);
//    /*config register for channel A */
//    stcTimerCfg.Tim0_CounterMode = Tim0_Async;
//    stcTimerCfg.Tim0_AsyncClockSource = Tim0_XTAL32;
//    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv0;
//    stcTimerCfg.Tim0_CmpValue = 32000/4 - 1;
//    TIMER0_BaseInit(TMR_UNIT,Tim0_ChannelA,&stcTimerCfg);

//    /* Enable channel A interrupt */
//    TIMER0_IntCmd(TMR_UNIT,Tim0_ChannelA,Enable);
//    /* Register TMR_INI_GCMA Int to Vect.No.001 */
//    stcIrqRegiConf.enIRQn = Int001_IRQn;
//    /* Select I2C Error or Event interrupt function */
//    stcIrqRegiConf.enIntSrc = TMR_INI_GCMA;
//    /* Callback function */
//    stcIrqRegiConf.pfnCallback = Timer0A_CallBack;
//    /* Registration IRQ */
//    enIrqRegistration(&stcIrqRegiConf);
//    /* Clear Pending */
//    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
//    /* Set priority */
//    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    /* Enable NVIC */
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /*config register for channel B */
    stcTimerCfg.Tim0_CounterMode = Tim0_Sync;
    stcTimerCfg.Tim0_SyncClockSource = Tim0_Pclk1;
    stcTimerCfg.Tim0_ClockDivision = Tim0_ClkDiv2;
    stcTimerCfg.Tim0_CmpValue = 21;
    TIMER0_BaseInit(TMR01_UNIT,Tim0_ChannelB,&stcTimerCfg);

    /* Enable channel B interrupt */
    TIMER0_IntCmd(TMR01_UNIT,Tim0_ChannelB,Enable);
    /* Register TMR_INI_GCMB Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = TIMER01_B_IRQn;
    /* Select I2C Error or Event interrupt function */
    stcIrqRegiConf.enIntSrc = TMR01_INI_GCMB;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = Timer01B_CallBack;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
    /*start timer0*/
//    TIMER0_Cmd(TMR_UNIT,Tim0_ChannelA,Enable);
    
}
/**
 *******************************************************************************
 ** \brief Timer01B_Start
 **
 ** \param  Count 定时计数值 单位0.5us
 **
 ** \retval None
 **
 ******************************************************************************/
en_result_t Timer01B_Start(uint16_t Count)
{
    if(Count<3120)
    {
        TMR01_UNIT->CMPBR_f.CMPB = Count*21;
    }
    else
    {
        return Error;
    }
    TIMER0_Cmd(TMR01_UNIT,Tim0_ChannelB,Enable);
    return Ok;
}
/**
 *******************************************************************************
 ** \brief Timer01B_Stop
 **
 ** \param  None
 **
 ** \retval Ok/Error
 **
 ******************************************************************************/
en_result_t Timer01B_Stop(void)
{
    TIMER0_Cmd(TMR01_UNIT,Tim0_ChannelB,Disable);
}


