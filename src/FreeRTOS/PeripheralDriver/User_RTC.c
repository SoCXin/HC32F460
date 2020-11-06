#include "hc32_ddl.h"
#include "System_InterruptCFG_Def.h"

uint8_t secdata;
void RTC_GetTime(uint8_t *sec);
volatile bool flag_RTC_interrupt;
typedef enum
{
    RTC_PRDS_NONE = 0,
    RTC_PRDS_05S  = 1,
    RTC_PRDS_1S   = 2,
    RTC_PRDS_1Min = 3,
    RTC_PRDS_1Hor = 4,
    RTC_PRDS_1Day = 5,
    RTC_PRDS_1Mon = 6, 
}en_rtc_prds_t;

typedef struct
{
    bool oneHZSEL;
    bool oenHZOE;
    bool AMPM;
    en_rtc_prds_t PRDS;   
    bool ALME;
    bool ALMIE;
    bool PRDIE;
    bool WAIT;
    bool RCKSEL;
    bool LRCEN;
}stc_rtc_cfg_t;

void RTC_PRDI_Callback(void)
{
//    printf("RTC 1S\r\n");
    RTC_GetTime(&secdata);
//    printf("Second:%d",secdata);
}
void RTC_Reset(void)
{
    M4_RTC->CR0_f.RESET = 0;
    M4_RTC->CR0_f.RESET = 1;
}
void RTC_Start(bool status)
{
    M4_RTC->CR1_f.START = status;
}

void RTC_SetTime(uint8_t sec,uint8_t minute, uint8_t hour, uint8_t day, uint8_t month, uint8_t year)
{
    M4_RTC->SEC_f.SECD = sec/10;
    M4_RTC->SEC_f.SECU = sec%10;
    M4_RTC->MIN_f.MIND = minute/10;
    M4_RTC->MIN_f.MINU = minute%10;
    M4_RTC->HOUR_f.HOURD = hour/10;
    M4_RTC->HOUR_f.HOURU = hour%10;
    M4_RTC->DAY_f.DAYD = day/10;
    M4_RTC->DAY_f.DAYU = day%10;
    M4_RTC->MON_f.MON = month;
    M4_RTC->YEAR_f.YEARD = year/10;
    M4_RTC->YEAR_f.YEARU = year%10;
}
void RTC_GetTime(uint8_t *sec)
{
    *sec = M4_RTC->SEC_f.SECD*10;
    *sec += M4_RTC->SEC_f.SECU;
}
void USER_RTC_Init(stc_rtc_cfg_t *p_stc_rtc_cfg)
{
    M4_RTC->CR3_f.LRCEN = p_stc_rtc_cfg->LRCEN;
    M4_RTC->CR3_f.RCKSEL = p_stc_rtc_cfg->RCKSEL;
  
    M4_RTC->CR1_f.PRDS = p_stc_rtc_cfg->PRDS;
    M4_RTC->CR1_f.AMPM = p_stc_rtc_cfg->AMPM;
    M4_RTC->CR1_f.ONEHZOE = p_stc_rtc_cfg->oenHZOE;
    M4_RTC->CR1_f.ONEHZSEL = p_stc_rtc_cfg->oneHZSEL;
    M4_RTC->CR2_f.ALME = p_stc_rtc_cfg->ALME;
    M4_RTC->CR2_f.ALMIE = p_stc_rtc_cfg->ALMIE;
    //M4_RTC->CR2_f.CRIE = //数据手册这里是Reserved
    M4_RTC->CR2_f.PRDIE = p_stc_rtc_cfg->PRDIE;
//    M4_RTC->CR2_f.WAIT = p_stc_rtc_cfg->WAIT;
}
void User_RTC_Init(void)
{
    stc_rtc_cfg_t STCRTCCFG;
    stc_irq_regi_conf_t stcIrqRegiConf;
    MEM_ZERO_STRUCT(STCRTCCFG);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    
//    M4_SYSREG->MPU_IPPR_f.RTCWRP = 0;//允许写RTC寄存器
//    M4_SYSREG->MPU_IPPR_f.RTCRDP = 0;//允许读RTC寄存器
    
    RTC_Reset();
    M4_RTC->CR1_f.START = 0;//停止计数
    
    STCRTCCFG.oenHZOE = 0;//禁止1HZ输出
    STCRTCCFG.AMPM = 1;//24时制
    STCRTCCFG.PRDS = RTC_PRDS_1S;//中断周期1s
    STCRTCCFG.LRCEN = 1;//低速时钟工作
    STCRTCCFG.PRDIE = 1;//周期中断启动
    if(RTC_DeInit() == ErrorTimeout)
    {
        ;
    }
    USER_RTC_Init(&STCRTCCFG);
    stcIrqRegiConf.enIntSrc = INT_RTC_PRD;
	stcIrqRegiConf.pfnCallback = RTC_PRDI_Callback;
	stcIrqRegiConf.enIRQn = RTC_PRDI_IRQn;
	enIrqRegistration(&stcIrqRegiConf);
	
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
	NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);//Enable Interrupt
    
    RTC_Start(Enable);
}



