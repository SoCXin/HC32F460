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
 ** \brief This example demonstrates how to use DCU compare function.
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
    stc_dcu_init_t stcDcuInit;
    en_result_t enTestResult = Ok;
    en_flag_status_t enDcuFlag1 = Reset;
    en_flag_status_t enDcuFlag2 = Reset;
    uint8_t au8Data0Val[4] = {0x00, 0x22, 0x44, 0x88};
    uint8_t au8Data1Val[4] = {0x00, 0x11, 0x55, 0x88};
    uint8_t au8Data2Val[4] = {0x00, 0x11, 0x55, 0x88};

    /* Initialize LED */
    BSP_LED_Init();

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DCU1, Enable);

    /* Initialize DCU */
    MEM_ZERO_STRUCT(stcDcuInit);
    stcDcuInit.u32IntSel = 0u;
    stcDcuInit.enIntWinMode = DcuIntInvalid;
    stcDcuInit.enDataSize = DcuDataBit8;
    stcDcuInit.enOperation = DcuOpCompare;
    stcDcuInit.enCmpTriggerMode = DcuCmpTrigbyData0;
    DCU_Init(DCU_UNIT, &stcDcuInit);

    /* DATA0 = DATA1  &&  DATA0 = DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[0]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[0]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[0]);

    enDcuFlag1 = DCU_GetIrqFlag(DCU_UNIT, DcuIntEq1);
    enDcuFlag2 = DCU_GetIrqFlag(DCU_UNIT, DcuIntEq2);
    if ((Set != enDcuFlag1) || (Set != enDcuFlag2))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq2);

    /* DATA0 > DATA1  &&  DATA0 > DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[1]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[1]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[1]);

    enDcuFlag1 = DCU_GetIrqFlag(DCU_UNIT, DcuIntGt1);
    enDcuFlag2 = DCU_GetIrqFlag(DCU_UNIT, DcuIntGt2);
    if ((Set != enDcuFlag1) || (Set != enDcuFlag2))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntGt1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntGt2);

    /* DATA0 < DATA1  &&  DATA0 < DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[2]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[2]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[2]);

    enDcuFlag1 = DCU_GetIrqFlag(DCU_UNIT, DcuIntLs1);
    enDcuFlag2 = DCU_GetIrqFlag(DCU_UNIT, DcuIntLs2);
    if ((Set != enDcuFlag1) || (Set != enDcuFlag2))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntLs1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntLs2);

    /* DATA0 = DATA1  &&  DATA0 = DATA2 */
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData1, au8Data1Val[3]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData2, au8Data2Val[3]);
    DCU_WriteDataByte(DCU_UNIT, DcuRegisterData0, au8Data0Val[3]);

    enDcuFlag1 = DCU_GetIrqFlag(DCU_UNIT, DcuIntEq1);
    enDcuFlag2 = DCU_GetIrqFlag(DCU_UNIT, DcuIntEq2);
    if ((Set != enDcuFlag1) || (Set != enDcuFlag2))
    {
        enTestResult = Error;
    }
    else
    {
    }

    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq1);
    DCU_ClearIrqFlag(DCU_UNIT, DcuIntEq2);

    if (Ok == enTestResult)
    {
        BSP_LED_On(LED_GREEN);  /* Test pass && meet the expected */
    }
    else
    {
        BSP_LED_On(LED_RED);  /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
