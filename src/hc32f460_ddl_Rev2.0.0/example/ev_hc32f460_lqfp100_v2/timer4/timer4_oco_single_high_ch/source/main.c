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
 ** \brief This example demonstrates how to use the single high channel of
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
    BSP_LED_Toggle(LED_GREEN);

    TIMER4_OCO_ClearIrqFlag(TIMER4_UNIT, TIMER4_OCO_HIGH_CH);
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
    stc_oco_high_ch_compare_mode_t stcHighChCmpMode;
    uint16_t OcoHighChOccrVal = TIMER4_CNT_CYCLE_VAL / 2u;

    /* Clear structures */
    MEM_ZERO_STRUCT(stcCntInit);
    MEM_ZERO_STRUCT(stcOcoInit);
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
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
    TIMER4_CNT_Init(TIMER4_UNIT, &stcCntInit);                      /* Initialize CNT */
    TIMER4_CNT_SetCycleVal(TIMER4_UNIT, TIMER4_CNT_CYCLE_VAL);      /* Set CNT Cycle value */

    /* Timer4 OCO : Initialize OCO channel configuration structure */
    stcOcoInit.enOcoIntCmd = Enable;
    stcOcoInit.enPortLevel = OcPortLevelLow;
    stcOcoInit.enOcmrBufMode = OcmrBufDisable;
    stcOcoInit.enOccrBufMode = OccrBufDisable;
    TIMER4_OCO_Init(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcOcoInit);  /* Initialize OCO high channel */

    if (!(TIMER4_OCO_HIGH_CH % 2))
    {
        /* ocmr[15:0] = 0x0FFF     0000 1111 1111 1111   */
        stcHighChCmpMode.enCntZeroMatchOpState = OcoOpOutputReverse;     /* Bit[11:10]  11 */
        stcHighChCmpMode.enCntZeroNotMatchOpState = OcoOpOutputHold;     /* Bit[15:14]  00 */
        stcHighChCmpMode.enCntUpCntMatchOpState = OcoOpOutputReverse;    /* Bit[9:8]    11 */
        stcHighChCmpMode.enCntPeakMatchOpState = OcoOpOutputReverse;     /* Bit[7:6]    11 */
        stcHighChCmpMode.enCntPeakNotMatchOpState = OcoOpOutputHold;     /* Bit[13:12]  00 */
        stcHighChCmpMode.enCntDownCntMatchOpState = OcoOpOutputReverse;  /* Bit[5:4]    11 */

        stcHighChCmpMode.enCntZeroMatchOcfState = OcoOcfSet;     /* bit[3] 1 */
        stcHighChCmpMode.enCntUpCntMatchOcfState = OcoOcfSet;    /* bit[2] 1 */
        stcHighChCmpMode.enCntPeakMatchOcfState = OcoOcfSet;     /* bit[1] 1 */
        stcHighChCmpMode.enCntDownCntMatchOcfState = OcoOcfSet;  /* bit[0] 1 */

        stcHighChCmpMode.enMatchConditionExtendCmd = Disable;

        TIMER4_OCO_SetHighChCompareMode(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, &stcHighChCmpMode);  /* Set OCO high channel compare mode */
    }

    /* Set OCO compare value */
    TIMER4_OCO_WriteOccr(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, OcoHighChOccrVal);

    /* Enable OCO */
    TIMER4_OCO_OutputCompareCmd(TIMER4_UNIT, TIMER4_OCO_HIGH_CH, Enable);

    /* Set Timer4 OCO IRQ */
    MEM_ZERO_STRUCT(stcIrqRegiCfg);
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = &OcoIrqCallback;
    stcIrqRegiCfg.enIntSrc = TIMER4_OCO_HIGH_CH_INT_NUM;
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
