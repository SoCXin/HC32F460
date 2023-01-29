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
 ** \brief The example of ICG HRC function
 **
 **   - 2018-10-22  CDT  First version for Device Driver Library of ICG.
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
/* Clock output Port/Pin definition */
#define MCO_PORT                        (PortE)
#define MCO_PIN                         (Pin00)

/* Clock output channel definition */
#define MCO_CHANNEL                     (ClkOutputCh1)

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
 ** \brief System tick interrupt callback function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SysTick_IrqHandler(void)
{
    BSP_LED_Toggle(LED_RED);
}

/**
 *******************************************************************************
 ** \brief Clock output config
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Clock_OutputConfig(void)
{
    stc_clk_output_cfg_t stcClkOutputCfg;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcClkOutputCfg);

    /* Configuration clock output Port/Pin */
    PORT_SetFunc(MCO_PORT, MCO_PIN, Func_Mclkout, Disable);

    /* Configuration clock output structure */
    stcClkOutputCfg.enOutputSrc = ClkOutputSrcHrc;
    stcClkOutputCfg.enOutputDiv = ClkOutputDiv1;
    CLK_OutputClkConfig(MCO_CHANNEL, &stcClkOutputCfg);
    CLK_OutputClkCmd(MCO_CHANNEL, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for ICG HRC function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /**
     ***************************************************************************
     @verbatim
     ** Modify hc32f460_icg.h file of defines
     ** #define ICG1_HRC_HARDWARE_START     ICG_FUNCTION_ON
     **
     ** #define ICG1_HRC_FREQSEL            HRC_FREQUENCY_16MHZ
     ** #define ICG1_HRC_STOP               HRC_OSCILLATION_START
     @endverbatim
     **************************************************************************/
    /* BSP initialization */
    BSP_LED_Init();

    /* Configure clock output */
    Clock_OutputConfig();
    /* Switch system clock */
    CLK_SetSysClkSource(ClkSysSrcHRC);
    /* Init system tick */
    SysTick_Init(1u);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
