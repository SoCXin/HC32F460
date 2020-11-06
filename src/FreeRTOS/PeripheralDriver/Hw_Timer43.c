#include "hc32_ddl.h"
#include "Hw_Timer43.h"

static uint8_t m_CntOcoMatchIrq = 0;
static uint16_t m_au16OccrVal[ ] = {6250, 12500, 25000, 25000};

/**
 *******************************************************************************
 ** \brief oco match interrupt handler
 **
 ******************************************************************************/
static void OcoIrqCallback(void)
{
    m_CntOcoMatchIrq++;

    if(m_CntOcoMatchIrq >= ARRAY_SZ(m_au16OccrVal))
    {
        m_CntOcoMatchIrq = 0;
    }
    else
    {
        /* nop */
    }

    TIMER4_OCO_ClearIrqFlag(TIMER43_UNIT, TIMER43_OCO_LOW_UCH);
//    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_LOW_CH, m_au16OccrVal[m_CntOcoMatchIrq]);
}
///**
// *******************************************************************************
// ** \brief Get Timer4 OCO interrupt number.
// **
// ** \param [in] TMR4x                   Pointer to Timer4 instance register base
// ** \arg M4_TMR41                       Timer4 unit 1 instance register base
// ** \arg M4_TMR42                       Timer4 unit 2 instance register base
// ** \arg M4_TMR43                       Timer4 unit 3 instance register base
// ** \param [in] enCh                    Channel of Timer4 OCO
// ** \arg Timer4OcoOuh                   Timer4 OCO channel:OUH
// ** \arg Timer4OcoOul                   Timer4 OCO channel:OUL
// ** \arg Timer4OcoOvh                   Timer4 OCO channel:OVH
// ** \arg Timer4OcoOvl                   Timer4 OCO channel:OVL
// ** \arg Timer4OcoOwh                   Timer4 OCO channel:OWH
// ** \arg Timer4OcoOwl                   Timer4 OCO channel:OWL
// **
// ** \retval                             Timer4 OCO interrupt number
// **
// ******************************************************************************/
//static en_int_src_t GetTimer4OcoIntNum(M4_TMR4_TypeDef *TMR4x,
//                                  en_timer4_oco_ch_t enCh)
//{
//    uint8_t u8Timer4Unit = 0u;
//    uint8_t u8Timer4OcoCh = (uint8_t)enCh;
//    static const en_int_src_t aenOcoIntNum[3][6] = {
//    {INT_TMR41_GCMUH, INT_TMR41_GCMUL, INT_TMR41_GCMVH, INT_TMR41_GCMVL, INT_TMR41_GCMWH, INT_TMR41_GCMWL},
//    {INT_TMR42_GCMUH, INT_TMR42_GCMUL, INT_TMR42_GCMVH, INT_TMR42_GCMVL, INT_TMR42_GCMWH, INT_TMR42_GCMWL},
//    {INT_TMR43_GCMUH, INT_TMR43_GCMUL, INT_TMR43_GCMVH, INT_TMR43_GCMVL, INT_TMR43_GCMWH, INT_TMR43_GCMWL}};

//    DDL_ASSERT(IS_VALID_OCO_CH(enCh));
//    DDL_ASSERT(IS_VALID_TIMER4(TMR4x));

//    if(M4_TMR41 == TMR4x)
//    {
//        u8Timer4Unit = 0u;
//    }
//    else if(M4_TMR42 == TMR4x)
//    {
//        u8Timer4Unit = 1u;
//    }
//    else if(M4_TMR43 == TMR4x)
//    {
//        u8Timer4Unit = 2u;
//    }
//    else
//    {
//        /* nop */
//    }

//    return aenOcoIntNum[u8Timer4Unit][u8Timer4OcoCh];
//}
void Hw_Timer43_init(void)
{
//    en_timer4_pwm_ch_t enPwmCh;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_pwm_init_t stcPwmInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;
    stc_timer4_emb_init_t stcEmbInit;
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcPwmInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    MEM_ZERO_STRUCT(stcLowChCmpMode);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43 , Enable);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_EMB, Enable);
        /* Initialize PWM I/O */
    PORT_SetFunc(TIMER43_PWM_UH_PORT, TIMER43_PWM_UH_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER43_PWM_UL_PORT, TIMER43_PWM_UL_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER43_PWM_VH_PORT, TIMER43_PWM_VH_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER43_PWM_VL_PORT, TIMER43_PWM_VL_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER43_PWM_WH_PORT, TIMER43_PWM_WH_PIN, Func_Tim4, Disable);
    PORT_SetFunc(TIMER43_PWM_WL_PORT, TIMER43_PWM_WL_PIN, Func_Tim4, Disable);
    
    stcCntInit.enBufferCmd = Disable;//Ω˚÷πCPRSª∫¥Ê
    stcCntInit.enClk = Timer4CntPclk;// ±÷”‘¥—°‘Ò
    stcCntInit.enClkDiv = Timer4CntPclkDiv1;//
    stcCntInit.enCntMode = Timer4CntTriangularWave;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;//≤ª∆¡±Œ…œ“Á÷–∂œ
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;//≤ª∆¡±Œœ¬“Á÷–∂œ
    stcCntInit.u16Cycle = TIMER43_CNT_CYCLE_VAL;
    
    TIMER4_CNT_Init(TIMER43_UNIT, &stcCntInit);
    
    stcOcoInit.enOcoIntCmd = Enable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOccrBufMode = OccrBufTrsfByCntZero;
    stcOcoInit.enOcmrBufMode = OcmrBufTrsfByCntZero;
    TIMER4_OCO_Init(TIMER43_UNIT,TIMER43_OCO_LOW_UCH,&stcOcoInit);
    TIMER4_OCO_Init(TIMER43_UNIT,TIMER43_OCO_LOW_VCH,&stcOcoInit);
    TIMER4_OCO_Init(TIMER43_UNIT,TIMER43_OCO_LOW_WCH,&stcOcoInit);
     if ((Timer4OcoOul == TIMER43_OCO_LOW_UCH) || (Timer4OcoOvl == TIMER43_OCO_LOW_VCH) || (Timer4OcoOwl == TIMER43_OCO_LOW_WCH))
    {
        /* OCMR[31:0] Ox 0FF0 0FFF    0000 1111 1111 0000   0000 1111 1111 1111 */
        stcLowChCmpMode.enCntZeroLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[27:26]  11 */
        stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[11:10]  11 */
        stcLowChCmpMode.enCntZeroLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;         /* bit[31:30]  00 */
        stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;      /* bit[15:14]  00 */

        stcLowChCmpMode.enCntUpCntLowMatchHighMatchLowChOpState = OcoOpOutputReverse;        /* bit[25:24]  11 */
        stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;     /* bit[9:8]    11 */
        stcLowChCmpMode.enCntUpCntLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;        /* bit[19:18]  00 */

        stcLowChCmpMode.enCntPeakLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[23:22]  11 */
        stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[7:6]    11 */
        stcLowChCmpMode.enCntPeakLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;         /* bit[29:28]  00 */
        stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;      /* bit[13:12]  00 */

        stcLowChCmpMode.enCntDownLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[21:20]  11 */
        stcLowChCmpMode.enCntDownLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[5:4]    11 */
        stcLowChCmpMode.enCntDownLowNotMatchHighMatchLowChOpState = OcoOpOutputHold;         /* bit[17:16]  00 */

        stcLowChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;    /* bit[3] 1 */
        stcLowChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;   /* bit[2] 1 */
        stcLowChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;    /* bit[1] 1 */
        stcLowChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet; /* bit[0] 1 */

        TIMER4_OCO_SetLowChCompareMode(TIMER43_UNIT, TIMER43_OCO_LOW_UCH, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
        TIMER4_OCO_SetLowChCompareMode(TIMER43_UNIT, TIMER43_OCO_LOW_VCH, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
        TIMER4_OCO_SetLowChCompareMode(TIMER43_UNIT, TIMER43_OCO_LOW_WCH, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
    }
    else
    {
        /* nop */
    }

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER43_UNIT, TIMER43_OCO_LOW_UCH, 100); /* Set OCO low channel compare value */
    TIMER4_OCO_WriteOccr(TIMER43_UNIT, TIMER43_OCO_LOW_VCH, 200); /* Set OCO low channel compare value */
    TIMER4_OCO_WriteOccr(TIMER43_UNIT, TIMER43_OCO_LOW_WCH, 100); /* Set OCO low channel compare value */

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER43_UNIT, TIMER43_OCO_LOW_UCH, Enable);
    TIMER4_OCO_OutputCompareCmd(TIMER43_UNIT, TIMER43_OCO_LOW_VCH, Enable);
    TIMER4_OCO_OutputCompareCmd(TIMER43_UNIT, TIMER43_OCO_LOW_WCH, Enable);


    stcEmbInit.enPwmHold = EmbChangePwm;
    stcEmbInit.enEmbState = EmbTrigPwmOutputLowLevel;
    TIMER4_EMB_Init(TIMER43_UNIT, &stcEmbInit);

    /* Timer4 PWM: Initialize OCO configuration structure */
    stcPwmInit.enRtIntMaskCmd = Enable;
    stcPwmInit.enClkDiv = PwmPlckDiv1;
    stcPwmInit.enOutputState = PwmHPwmLHold;  /* change: PwmHPwmLHold  PwmHPwmLReverse  PwmHReversePwmLHold  PwmHHoldPwmLReverse */
    stcPwmInit.enMode = PwmDeadTimerMode;
    
    TIMER4_PWM_SetFilterCountValue(TIMER43_UNIT, TIMER43_PWM_UCH, (1000)/2);
    TIMER4_PWM_WriteDeadRegionValue(TIMER43_UNIT, TIMER43_PWM_UCH, 1u, 1u);
    TIMER4_PWM_Init(TIMER43_UNIT, TIMER43_PWM_UCH, &stcPwmInit); /* Initialize timer4 pwm */
    
    TIMER4_PWM_SetFilterCountValue(TIMER43_UNIT, TIMER43_PWM_VCH, (1000)/2);
    TIMER4_PWM_WriteDeadRegionValue(TIMER43_UNIT, TIMER43_PWM_VCH, 1u, 1u);
    TIMER4_PWM_Init(TIMER43_UNIT, TIMER43_PWM_VCH, &stcPwmInit); /* Initialize timer4 pwm */
    
    TIMER4_PWM_SetFilterCountValue(TIMER43_UNIT, TIMER43_PWM_WCH, (1000)/2);
    TIMER4_PWM_WriteDeadRegionValue(TIMER43_UNIT, TIMER43_PWM_WCH, 1u, 1u);
    TIMER4_PWM_Init(TIMER43_UNIT, TIMER43_PWM_WCH, &stcPwmInit); /* Initialize timer4 pwm */
    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER43_UNIT);
    TIMER4_CNT_Start(TIMER43_UNIT);
}

void Timer43_EMB_Break(void)
{
    EMB_SwBrake(TIMER43_EMB_UNIT,true);
}

void Timer43_EMB_UnBreak(void)
{
    EMB_SwBrake(TIMER43_EMB_UNIT,false);
}
