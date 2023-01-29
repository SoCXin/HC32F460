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
 ** \brief This example demonstrates how to use the compare trigger function of
 **        Timer4Sevt.
 **
 **   - 2018-10-30 CDT First version for Device Driver Library of Timer4Sevt
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
/* DCU unit */
#define DCU_UNIT                        (M4_DCU1)
#define DCU_DATA0_VAL                   (0x0000u)
#define DCU_DATA1_VAL                   (0x4444u)

/* Timer4 unit */
#define TIMER4_UNIT                     (M4_TMR41)

/* Timer4 CNT cycle */
#define TIMER4_CNT_CYCLE_VAL            (50000u)                    /*< Timer4 counter cycle value */

/* Timer4 OCO channel */
#define TIMER4_OCO_CH                   (Timer4OcoOuh)
#define TIMER4_OCO_OCCR_VAL             (TIMER4_CNT_CYCLE_VAL*3u/4u)  /*< OCO cycle value */

/* Timer4 SEVT channel */
#define TIMER4_SEVT_CH                  (Timer4SevtCh0)
#define TIMER4_SEVT_CMP_VAL             (TIMER4_CNT_CYCLE_VAL*3u/4u)  /*< Timer4 counter compare value */
#define TIMER4_SEVT_TRG_EVT             (SevtTrgEvtSCMUH)
#define TIMER4_SEVT_EVT_NUM             (EVT_TMR41_SCMUH)

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
 ** \brief  Main function of project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    en_result_t enTestResult = Ok;
    stc_dcu_init_t stcDcuInit;
    stc_timer4_cnt_init_t stcCntInit;
    stc_timer4_oco_init_t stcOcoInit;
    stc_timer4_sevt_init_t stcSevtInit;
    stc_timer4_sevt_trigger_cond_t stcSevtTrgCond;
    static uint16_t m_au16Data0Val = 0u;
    static uint16_t m_au16Data2Val = 0u;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcDcuInit);
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcSevtInit);
    MEM_ZERO_STRUCT(stcSevtTrgCond);

    /* Initialize LED */
    BSP_LED_Init();
    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS|PWC_FCG0_PERIPH_DCU1, Enable);
    PWC_Fcg2PeriphClockCmd(PWC_FCG2_PERIPH_TIM41|PWC_FCG2_PERIPH_TIM42|PWC_FCG2_PERIPH_TIM43, Enable);

    /* Initialize DCU */
    stcDcuInit.u32IntSel = 0ul;
    stcDcuInit.enIntWinMode = DcuIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit16;
    stcDcuInit.enOperation = DcuHwTrigOpAdd;
    DCU_Init(DCU_UNIT, &stcDcuInit);
    DCU_WriteDataHalfWord(DCU_UNIT, DcuRegisterData0, DCU_DATA0_VAL);
    DCU_WriteDataHalfWord(DCU_UNIT, DcuRegisterData1, DCU_DATA1_VAL);
    DCU_SetTriggerSrc(DCU_UNIT, TIMER4_SEVT_EVT_NUM);

    /* Timer4-CNT : Initialize CNT configuration structure */
    stcCntInit.enBufferCmd = Disable;
    stcCntInit.enClk = Timer4CntPclk;
    stcCntInit.enClkDiv = Timer4CntPclkDiv1;  /* CNT clock divide */
    stcCntInit.enCntMode = Timer4CntTriangularWave;
    stcCntInit.enZeroIntMsk = Timer4CntIntMask0;
    stcCntInit.enPeakIntMsk = Timer4CntIntMask0;
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit); /* Initialize CNT */
    TIMER4_CNT_SetCycleVal(TIMER4_UNIT, TIMER4_CNT_CYCLE_VAL);

    /*  Timer4-OCO: Initialize OCO configuration structure */
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcoIntCmd = Disable;
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_CH, &stcOcoInit);     /* Initialize OCO */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_CH, TIMER4_OCO_OCCR_VAL); /* Set OCO channel compare value */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_CH, Enable);       /* Enable OCO channel */

    /*  Timer4-SEVT: Initialize SEVT configuration structure */
    stcSevtInit.enBuf = SevtBufDisable;
    stcSevtInit.enOccrSel = SevtSelOCCRxh;
    stcSevtInit.enMode = SevtDelayTrigMode;
    stcSevtInit.enTrigEvt = TIMER4_SEVT_TRG_EVT;
    TIMER4_SEVT_Init(TIMER4_UNIT, TIMER4_SEVT_CH, &stcSevtInit); /* Initialize SEVT */
    TIMER4_SEVT_WriteSCCR(TIMER4_UNIT, TIMER4_SEVT_CH, TIMER4_SEVT_CMP_VAL); /* Set SEVT compare value */

    stcSevtTrgCond.enUpMatchCmd = Disable;
    stcSevtTrgCond.enDownMatchCmd = Enable;
    stcSevtTrgCond.enZeroMatchCmd = Disable;
    stcSevtTrgCond.enPeakMatchCmd = Disable;
    TIMER4_SEVT_SetTriggerCond(TIMER4_UNIT, TIMER4_SEVT_CH, &stcSevtTrgCond); /* Set SEVT operation condition */

    /* Clear && Start CNT */
    TIMER4_CNT_ClearCountVal(TIMER4_UNIT);
    TIMER4_CNT_Start(TIMER4_UNIT);

    /* Wait DCU add operation */
    while (DCU_DATA0_VAL == m_au16Data0Val)
    {
        m_au16Data0Val = DCU_ReadDataHalfWord(DCU_UNIT, DcuRegisterData0);
    }

    /* Stop CNT */
    TIMER4_CNT_Stop(TIMER4_UNIT);

    m_au16Data2Val = DCU_ReadDataHalfWord(DCU_UNIT, DcuRegisterData2);
    if (m_au16Data0Val != (2u * m_au16Data2Val))
    {
        enTestResult = Error;
    }
    else
    {
    }

    if (Ok == enTestResult)
    {
        BSP_LED_On(LED_GREEN);  /* Test pass && meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);    /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
