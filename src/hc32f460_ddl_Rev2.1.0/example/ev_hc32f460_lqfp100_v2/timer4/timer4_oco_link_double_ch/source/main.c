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
 ** \brief This example demonstrates how to use the link double channels of
 **        Timer4Oco.
 **
 **   - 2021-04-16 CDT First version for Device Driver Library of Timer4Oco
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
/* Timer4 CNT */
#define TIMER4_UNIT                     (M4_TMR41)
#define TIMER4_CNT_CYCLE_VAL            (50000u)        /* Timer4 counter cycle value */

/* Timer4 OCO */
#define TIMER4_OCO_HIGH_CH              (Timer4OcoOuh)  /* only Timer4OcoOuh  Timer4OcoOvh  Timer4OcoOwh */

/* Timer4 OCO interrupt number */
#define TIMER4_OCO_HIGH_CH_INT_NUM      (INT_TMR41_GCMUH)
#define TIMER4_OCO_LOW_CH_INT_NUM       (INT_TMR41_GCMUL)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void OcoIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static int8_t m_OcoLowCh = TIMER4_OCO_HIGH_CH + 1;
static en_timer4_oco_port_level_t m_enOcoLowChLastOpOutState = OcPortLevelLow;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief oco match interrupt handler
 **
 ******************************************************************************/
static void OcoIrqCallback(void)
{
    en_timer4_oco_port_level_t enOcoOpOutState;

    enOcoOpOutState = TIMER4_OCO_GetOpPinLevel(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh);
    if (m_enOcoLowChLastOpOutState != enOcoOpOutState)
    {
        BSP_LED_Toggle(LED_GREEN);
        m_enOcoLowChLastOpOutState = enOcoOpOutState;
        TIMER4_OCO_ClearIrqFlag(TIMER4_UNIT, TIMER4_OCO_HIGH_CH);
        TIMER4_OCO_ClearIrqFlag(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh);
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
    stc_irq_regi_conf_t stcIrqRegiCfg;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_oco_low_ch_compare_mode_t stcLowChCmpMode;
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;
    uint16_t OcoHighChOccrVal = TIMER4_CNT_CYCLE_VAL / 4u;
    uint16_t OcoLowChOccrVal  = TIMER4_CNT_CYCLE_VAL * 3u /4u;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcLowChCmpMode);
    MEM_ZERO_STRUCT(stcHighChCmpMode);

    /* Initialize LED */
    BSP_LED_Init();
    /* Enable peripheral clock */
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41 | PWC_FCG2_PERIPH_TIM42 | PWC_FCG2_PERIPH_TIM43, Enable);

    /* Timer4 CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv16;   /* CNT clock divide */
    stcCntInit.enCntMode = Timer4CntSawtoothWave;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit);                       /* Initialize CNT */
    TIMER4_CNT_SetCycleVal(TIMER4_UNIT, TIMER4_CNT_CYCLE_VAL);      /* Set CNT Cycle value */

    /* Timer4 OCO : Initialize OCO channel configuration structure */
    stcOcoInit.enOcoIntCmd = Enable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enOccrBufMode = OccrBufTrsfByCntZero;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit);  /* Initialize OCO high channel */
    TIMER4_OCO_Init(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh, &stcOcoInit);          /* Initialize OCO low channel */

    if (!(TIMER4_OCO_HIGH_CH%2))
    {
        /* ocmr[15:0] = 0x000F     0000 0000 0000 1111   */
        stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputHold;     /* Bit[11:10]  00 */
        stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHold;  /* Bit[15:14]  00 */
        stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputHold;    /* Bit[9:8]    00 */
        stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputHold;     /* Bit[7:6]    00 */
        stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;  /* Bit[13:12]  00 */
        stcHighChCmpMode.enCntDownCntMatchOpState = OcoOpOutputHold;  /* Bit[5:4]    00 */

        stcHighChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;     /* bit[3] 1 */
        stcHighChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;    /* bit[2] 1 */
        stcHighChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;     /* bit[1] 1 */
        stcHighChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet;  /* bit[0] 1 */

        stcHighChCmpMode.enMatchConditionExtendCmd = Disable;

        TIMER4_OCO_SetHighChCompareMode(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcHighChCmpMode);  /* Set OCO high channel compare mode */
    }

    if (m_OcoLowCh%2)
    {
        /* OCMR[31:0] Ox FFFF 0FFF    1111 1111 1111 1111  0000 1111 1111 1111 */
        stcLowChCmpMode.enCntZeroLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[27:26]  11  */
        stcLowChCmpMode.enCntZeroLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[11:10]  11  */
        stcLowChCmpMode.enCntZeroLowNotMatchHighMatchLowChOpState = OcoOpOutputReverse;      /* bit[31:30]  11 */
        stcLowChCmpMode.enCntZeroLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;      /* bit[15:14]  00 */

        stcLowChCmpMode.enCntUpCntLowMatchHighMatchLowChOpState = OcoOpOutputReverse;        /* bit[25:24]  11 */
        stcLowChCmpMode.enCntUpCntLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;     /* bit[9:8]    11 */
        stcLowChCmpMode.enCntUpCntLowNotMatchHighMatchLowChOpState = OcoOpOutputReverse;     /* bit[19:18]  11 */

        stcLowChCmpMode.enCntPeakLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[23:22]  11 */
        stcLowChCmpMode.enCntPeakLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[7:6]    11 */
        stcLowChCmpMode.enCntPeakLowNotMatchHighMatchLowChOpState = OcoOpOutputReverse;      /* bit[29:28]  11 */
        stcLowChCmpMode.enCntPeakLowNotMatchHighNotMatchLowChOpState = OcoOpOutputHold;      /* bit[13:12]  00 */

        stcLowChCmpMode.enCntDownLowMatchHighMatchLowChOpState = OcoOpOutputReverse;         /* bit[21:20]  11 */
        stcLowChCmpMode.enCntDownLowMatchHighNotMatchLowChOpState = OcoOpOutputReverse;      /* bit[5:4]    11 */
        stcLowChCmpMode.enCntDownLowNotMatchHighMatchLowChOpState = OcoOpOutputReverse;      /* bit[17:16]  11 */

        stcLowChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;    /* bit[3] 1 */
        stcLowChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;   /* bit[2] 1 */
        stcLowChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;    /* bit[1] 1 */
        stcLowChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet; /* bit[0] 1 */

        TIMER4_OCO_SetLowChCompareMode(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh, &stcLowChCmpMode);  /* Set OCO low channel compare mode */
    }

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, OcoHighChOccrVal);
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh, OcoLowChOccrVal);

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, Enable);
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh, Enable);

    m_enOcoLowChLastOpOutState = TIMER4_OCO_GetOpPinLevel(TIMER4_UNIT, (en_timer4_oco_ch_t)m_OcoLowCh);

    /* Set Timer4 OCO IRQ */
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = &OcoIrqCallback;
    stcIrqRegiCfg.enIntSrc = TIMER4_OCO_HIGH_CH_INT_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    stcIrqRegiCfg.enIRQn = Int001_IRQn;
    stcIrqRegiCfg.pfnCallback = &OcoIrqCallback;
    stcIrqRegiCfg.enIntSrc = TIMER4_OCO_LOW_CH_INT_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
