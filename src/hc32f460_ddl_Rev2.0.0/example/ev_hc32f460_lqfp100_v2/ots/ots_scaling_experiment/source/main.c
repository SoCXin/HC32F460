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
 ** \brief OTS sample
 **
 **   - 2021-04-16  CDT First version for Device Driver Library of
 **     OTS
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

/* This example will show how to do scaling experiment */

/* Average count. */
#define OTS_AVG_COUNT                       (10U)

/* Timeout value. */
#define OTS_TIMEOUT                         (1000UL)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void OtsInitConfig(void);

static void OtsClkConfig(en_ots_clk_sel_t enClk);
static void OtsScalingExperiment(const char *strClkSrc);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    BSP_KEY_Init();

    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    OtsInitConfig();
    /***************** Configuration end, application start **************/

    for (;;)
    {
        DDL_Printf("---> Press Key1 to start.\n");

        while (BSP_KEY_GetStatus(BSP_KEY_6) == Reset);

        /* 1. HRC 16MHz. */
        OtsClkConfig(OtsClkSel_Hrc);
        OtsScalingExperiment("HRC 16MHz");

        /* 2. Change clock to XTAL 8MHz. */
        OtsClkConfig(OtsClkSel_Xtal);
        OtsScalingExperiment("XTAL 8MHz");
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  OTS initialization configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsInitConfig(void)
{
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_OTS, Enable);

    /* Clock configuration. */
    CLK_HrcCmd(Enable);
    CLK_LrcCmd(Enable);
    CLK_Xtal32Cmd(Enable);
    CLK_XtalCmd(Enable);
}

/**
 *******************************************************************************
 ** \brief  Configures OTS clock.
 **
 ** \param  [in]  enClk                 Select the clock source of OTS.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsClkConfig(en_ots_clk_sel_t enClk)
{
    stc_ots_init_t stcOtsInit;

    stcOtsInit.enClkSel  = enClk;
    stcOtsInit.enAutoOff = OtsAutoOff_Disable;
    /* Initials OTS. */
    OTS_DeInit();
    OTS_Init(&stcOtsInit);
}

/**
 *******************************************************************************
 ** \brief  OTS scaling experiment entity.
 **
 ** \param  [in]  strClkSrc              String of OTS clock source information.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void OtsScalingExperiment(const char *strClkSrc)
{
    uint32_t i;
    uint16_t u16Dr1;
    uint16_t u16Dr2;
    uint16_t u16Ecr;

    float32_t f32A;
    float32_t f32SumA = 0.0f;
    DDL_Printf("---> Clock source is %s.\n", strClkSrc);
    for (i=0U; i<OTS_AVG_COUNT; i++)
    {
        Ddl_Delay1ms(100U);

        if (OTS_ScalingExperiment(&u16Dr1, &u16Dr2, &u16Ecr, &f32A, OTS_TIMEOUT) == Ok)
        {
            DDL_Printf("DR1 = %u, DR2 = %u, ECR = %u, A = %f\n",
                    u16Dr1, u16Dr2, u16Ecr, f32A);

            f32SumA += f32A;
        }
        else
        {
            DDL_Printf("OTS fault -- timeout.\n");
            while (1U)
            {

            }
        }
    }

    DDL_Printf("%s: SUM A = %f, AVG A = %f\n", strClkSrc, f32SumA, f32SumA/(float32_t)OTS_AVG_COUNT);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
