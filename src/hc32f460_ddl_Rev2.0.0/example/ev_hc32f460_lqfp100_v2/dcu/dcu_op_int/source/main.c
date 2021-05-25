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
 ** \brief This example demonstrates how to use DCU operation interrupt function.
 **
 **   - 2021-04-16 CDT First version for Device Driver Library of DCU
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
#define DCU_UNIT                        (M4_DCU1)
#define DCU_INT_NUM                     (INT_DCU1)

/* Parameter valid check for DCU Instances. */
#define IS_VALID_DCU(__DCUx__)                                                 \
(   (M4_DCU1 == (__DCUx__))             ||                                     \
    (M4_DCU2 == (__DCUx__))             ||                                     \
    (M4_DCU3 == (__DCUx__))             ||                                     \
    (M4_DCU4 == (__DCUx__)))

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void DcuIrqCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static en_int_status_t m_enOpIntStatus = Reset;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief DCU irq callback function.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void DcuIrqCallback(void)
{
    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntOp))
    {
        m_enOpIntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntOp);
    }
    else
    {
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
    stc_dcu_init_t stcDcuInit;
    en_result_t enTestResult = Ok;
    stc_irq_regi_conf_t stcIrqRegiCfg;
    uint8_t u8ReadData0Val;
    uint8_t u8ReadData2Val;
    uint8_t u8WriteData0Val = 0xFFu;
    uint8_t u8WriteData1Val = 0xFFu;

    /* Initialize LED */
    BSP_LED_Init();

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DCU1, Enable);

    /* Initialize DCU */
    MEM_ZERO_STRUCT(stcDcuInit);
    stcDcuInit.u32IntSel = DcuIntOp;
    stcDcuInit.enIntCmd = Enable;
    stcDcuInit.enIntWinMode = DcuWinIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit8;
    stcDcuInit.enOperation = DcuOpAdd;
    DCU_Init(DCU_UNIT, &stcDcuInit);

    /* Set DCU IRQ */
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = &DcuIrqCallback;
    stcIrqRegiCfg.enIntSrc = DCU_INT_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* overflow */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, u8WriteData0Val);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, u8WriteData1Val);
    if (Set != m_enOpIntStatus)
    {
        enTestResult = Error;
    }
    else
    {
    }

    u8ReadData0Val = DCU_ReadDataByte(DCU_UNIT, DcuRegisterData0);
    u8ReadData2Val = DCU_ReadDataByte(DCU_UNIT, DcuRegisterData2);

    /* Compare DCU regisger DATA0 && DATA2 value: DATA0 value == 2 * DATA2 value */
    if ((Ok == enTestResult) && (u8ReadData0Val == u8ReadData2Val * 2u))
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
