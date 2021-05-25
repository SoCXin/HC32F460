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
 ** \brief This example demonstrates how to use DCU compare interrupt function.
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
static en_int_status_t m_enEq1IntStatus = Reset;
static en_int_status_t m_enGt1IntStatus = Reset;
static en_int_status_t m_enLs1IntStatus = Reset;
static en_int_status_t m_enEq2IntStatus = Reset;
static en_int_status_t m_enGt2IntStatus = Reset;
static en_int_status_t m_enLs2IntStatus = Reset;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief DCU irq callback function.
 **
 ** \param [in]                         None
 **
 ** \retval                             None
 **
 ******************************************************************************/
static void DcuIrqCallback(void)
{
    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntEq1))
    {
        m_enEq1IntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq1);
    }
    else
    {
    }

    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntGt1))
    {
        m_enGt1IntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntGt1);
    }
    else
    {
    }

    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntLs1))
    {
        m_enLs1IntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntLs1);
    }

    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntEq2))
    {
        m_enEq2IntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq2);
    }
    else
    {
    }

    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntGt2))
    {
        m_enGt2IntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntGt2);
    }
    else
    {
    }

    if (Set == DCU_GetIrqFlag(DCU_UNIT, DcuIntLs2))
    {
        m_enLs2IntStatus = Set;
        DCU_ClearIrqFlag(DCU_UNIT, DcuIntLs2);
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
    uint8_t au8Data0Val[4] = {0x00u, 0x22u, 0x44u, 0x88u};
    uint8_t au8Data1Val[4] = {0x00u, 0x11u, 0x55u, 0x88u};
    uint8_t au8Data2Val[4] = {0x00u, 0x11u, 0x55u, 0x88u};

    /* Initialize LED */
    BSP_LED_Init();

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DCU1, Enable);

    /* Initialize DCU */
    MEM_ZERO_STRUCT(stcDcuInit);
    stcDcuInit.u32IntSel = (DcuIntGt1 | DcuIntEq1 | DcuIntLs1 | DcuIntGt2 | DcuIntEq2 | DcuIntLs2);
    stcDcuInit.enIntCmd = Enable;
    stcDcuInit.enIntWinMode = DcuWinIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit8;
    stcDcuInit.enOperation = DcuOpCompare;
    stcDcuInit.enCmpTriggerMode = DcuCmpTrigbyData0;
    DCU_Init(DCU_UNIT, &stcDcuInit);

    /* Set DCU IRQ */
    stcIrqRegiCfg.enIRQn = Int000_IRQn;
    stcIrqRegiCfg.pfnCallback = &DcuIrqCallback;
    stcIrqRegiCfg.enIntSrc = DCU_INT_NUM;
    enIrqRegistration(&stcIrqRegiCfg);
    NVIC_SetPriority(stcIrqRegiCfg.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
    NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);

    /* DATA0 = DATA1  &&  DATA0 = DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[0]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[0]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[0]);
    if ((Set != m_enEq1IntStatus) || (Set != m_enEq2IntStatus))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* DATA0 > DATA1  &&  DATA0 > DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[1]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[1]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[1]);
    if ((Set != m_enGt1IntStatus) || (Set != m_enGt2IntStatus))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* DATA0 < DATA1  &&  DATA0 < DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[2]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[2]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[2]);
    if ((Set != m_enLs1IntStatus) || (Set != m_enLs2IntStatus))
    {
        enTestResult = Error;
    }
    else
    {
    }

    m_enEq1IntStatus = Reset;
    m_enEq2IntStatus = Reset;
    /* DATA0 = DATA1  &&  DATA0 = DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[3]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[3]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[3]);
    if ((Set != m_enEq1IntStatus) || (Set != m_enEq2IntStatus))
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
