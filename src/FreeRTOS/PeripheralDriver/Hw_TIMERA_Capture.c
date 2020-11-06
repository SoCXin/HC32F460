#include "Hw_TIMERA_Capture.h"
static uint32_t period_cnt,cap_cnt=0;
uint16_t Cap_data[11];
uint32_t CapTime;
#define TimerPeriod 0x802C

void TimerACaptureCallback(void)
{
	if(Set==UNIT_CAPTIM->STFLR_f.CMPF1)
	{
		UNIT_CAPTIM->STFLR_f.CMPF1 = 0;
		if(cap_cnt == 0)
		{
			period_cnt = 0;
		}
		Cap_data[cap_cnt] = UNIT_CAPTIM->CMPAR1;
		cap_cnt++;
		if(cap_cnt>11)
		{
			cap_cnt = 0;
			CapTime = Cap_data[10]+(period_cnt*TimerPeriod)-Cap_data[0];
		}			
	}
}
void TimerA_OVF_Callback(void)
{
	period_cnt++;
}
uint32_t GetTime(void)
{
	return CapTime;
}
uint32_t GetFrequence(void)
{
	return 840000000/CapTime;//84MHZ	PCLK1, ±÷”£¨64∑÷∆µ
}
void TimerACaptureInit(void)
{
	stc_timera_base_init_t stcTimeraInit;
    stc_timera_capture_init_t stcTimeraCaptureInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcTimeraInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcTimeraCaptureInit);
    MEM_ZERO_STRUCT(stcPortInit);
	/* Configuration peripheral clock */
	
    PWC_Fcg2PeriphClockCmd(TIMERA_UNIT1_CLOCK, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Configuration TIMERA capture pin */
    PORT_SetFunc(TIMERA_UNIT1_CH1_PORT, TIMERA_UNIT1_CH1_PIN, TIMERA_UNIT1_CH1_FUNC, Disable);
    PORT_SetFunc(TIMERA_UNIT1_CH2_PORT, TIMERA_UNIT1_CH2_PIN, TIMERA_UNIT1_CH2_FUNC, Disable);
	/* Configuration timera unit 1 base structure */
    stcTimeraInit.enClkDiv = TimeraPclkDiv1;
    stcTimeraInit.enCntMode = TimeraCountModeSawtoothWave;
    stcTimeraInit.enCntDir = TimeraCountDirUp;
    stcTimeraInit.enSyncStartupEn = Disable;
    stcTimeraInit.u16PeriodVal = TimerPeriod;        //100ms
    TIMERA_BaseInit(UNIT_CAPTIM, &stcTimeraInit);

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
    TIMERA_CaptureInit(UNIT_CAPTIM, TIMERA_UNIT1_CH1, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(UNIT_CAPTIM, TIMERA_UNIT1_CH1_INT, Enable);

    /* Enable channel 2 capture and interrupt */
    TIMERA_CaptureInit(UNIT_CAPTIM, TIMERA_UNIT1_CH2, &stcTimeraCaptureInit);
    TIMERA_IrqCmd(UNIT_CAPTIM, TIMERA_UNIT1_CH2_INT, Enable);
	
	TIMERA_IrqCmd(UNIT_CAPTIM, TimeraIrqOverflow, Enable);

    /* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = TIMERA_UNIT1_COMPARE_INT;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &TimerACaptureCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
	
	/* Configure interrupt of timera unit 1 */
    stcIrqRegiConf.enIntSrc = INT_TMRA1_OVF;
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    stcIrqRegiConf.pfnCallback = &TimerA_OVF_Callback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
//    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
//    /* Set external Int Ch.4 trigger timera compare */
//    stcPortInit.enExInt = Enable;
//    PORT_Init(KEY1_PORT, KEY1_PIN, &stcPortInit);
//    TIMERA_SetCaptureTriggerSrc(KEY1_TRIGGER_EVENT);
    /* Timera unit 1 startup */
    TIMERA_Cmd(UNIT_CAPTIM, Enable);
}
