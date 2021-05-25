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
 ** \brief The example for SRAM function demonstration
 **
 **   - 2018-10-24 CDT First version for sample of SRAM module.
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
 ** \brief SRAM ECC/Parity error NMI IRQ handler
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SramErr_IrqHandler(void)
{
    /* SRAM3 1 bit ECC error */
    if (true == SRAM_GetStatus(Sram3EccErr1))
    {
        SRAM_ClrStatus(Sram3EccErr1);
    }

    /* SRAM3 2 bit ECC error */
    if (true == SRAM_GetStatus(Sram3EccErr2))
    {
        SRAM_ClrStatus(Sram3EccErr2);
        while (1)
        {
            BSP_LED_Toggle(LED_RED);
            Ddl_Delay1ms(100ul);
        }
    }

    /* SRAM12 parity error */
    if (true == SRAM_GetStatus(Sram12ParityErr))
    {
        SRAM_ClrStatus(Sram12ParityErr);
    }
    /* High speed SRAM parity error */
    if (true == SRAM_GetStatus(SramHSParityErr))
    {
        SRAM_ClrStatus(SramHSParityErr);
    }
    /* Retention SRAM parity error */
    if (true == SRAM_GetStatus(SramRetParityErr))
    {
        SRAM_ClrStatus(SramRetParityErr);
    }
}

/**
 *******************************************************************************
 ** \brief  main function for SRAM function
 **
 ** \param  None
 **
 ** \return int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    volatile uint32_t temp;
    stc_sram_config_t stcSramConfig;
    stc_nmi_config_t stcNmiConfig;

    /* BSP initialization */
    BSP_LED_Init();

    /* enable HS RAM source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_SRAMH, Enable);

    /* enable RAM0 source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_SRAM12, Enable);

    /* enable ECCRAM source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_SRAM3, Enable);

    /* enable Retention RAM source clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_SRAMRET, Enable);

    stcSramConfig.u8SramIdx = Sram12Idx | Sram3Idx | SramHsIdx | SramRetIdx;
    stcSramConfig.enSramRC = SramCycle5;
    stcSramConfig.enSramWC = SramCycle6;
    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramEccOp = SramNmi;
    stcSramConfig.enSramPyOp = SramNmi;

    SRAM_Init(&stcSramConfig);
    stcNmiConfig.pfnNmiCallback = &SramErr_IrqHandler;
    stcNmiConfig.u16NmiSrc = NmiSrcSramDE | NmiSrcSramPE;

    NMI_Init(&stcNmiConfig);
#if 1
    /* This section raw sample to generate the ECC error */
    M4_SRAMC->CKPR = 0x77ul;
    M4_SRAMC->CKCR = 0x03000000ul;
    SRAM3_BASE_ADDR = 0x1234567ul;
    temp = SRAM3_BASE_ADDR;

    M4_SRAMC->CKPR = 0x77ul;
    M4_SRAMC->CKCR = 0x00ul;
    SRAM3_BASE_ADDR = 0x1234562ul;

    M4_SRAMC->CKPR = 0x77ul;
    M4_SRAMC->CKCR = 0x03000000ul;
    temp = SRAM3_BASE_ADDR;
    if (temp == 0ul)
    {
        // avoid warning
    }
#endif
    while (1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
