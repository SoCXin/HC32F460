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
 ** \brief The example of ICG NMI function
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
 ** \brief NMI interrupt callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Nmi_IrqCallback(void)
{
    /* NMI Pin */
    if (Set == NMI_IrqFlgGet(NmiSrcNmi))
    {
        NMI_IrqFlgClr(NmiSrcNmi);
        BSP_LED_Toggle(LED_RED);
    }
}

/**
 *******************************************************************************
 ** \brief  main function for ICG NMI function
 **
 ** \param [in] None
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
     ** #define ICG1_NMI_HARDWARE_START         ICG_FUNCTION_ON
     **
     ** #define ICG1_NMI_SMPCLK                 NMI_PIN_FILTER_PCLK3_DIV1
     ** #define ICG1_NMI_TRG                    NMI_PIN_TRIGGER_EDGE_FALLING
     ** #define ICG1_NMI_IMR                    NMI_PIN_IRQ_ENABLE
     ** #define ICG1_NMI_NFEN                   NMI_DIGITAL_FILTER_DISABLE
     ** #define ICG1_NMI_ICGENA                 NMI_PIN_ICG_FUNCTION_ENABLE
     @endverbatim
     **************************************************************************/
    stc_nmi_config_t stcNmiConfig;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcNmiConfig);

    /* BSP initialization */
    BSP_LED_Init();
    /* NMI(PB11) Int Callback function */
    stcNmiConfig.pfnNmiCallback = &Nmi_IrqCallback;
    stcNmiConfig.u16NmiSrc = NmiSrcNmi;
    NMI_Init(&stcNmiConfig);

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
