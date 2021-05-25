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
 ** \brief efm sample
 **
 **   - 2018-11-01  CDT  First version for Device Driver Library of
 **     efm
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
#define EFM_IRQn                        (Int129_IRQn)

#define FLASH_SECTOR62_ADRR             (0x0007C000u)

#define FLASH_WIN_START_ADDR            (0x0007D000u)
#define FLASH_WIN_END_ADDR              (0x0007E000u)

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
void EfmPgmEraseErr_IrqHandler(void)
{
    BSP_LED_On(LED_RED);

    EFM_Unlock();
    EFM_ClearFlag(EFM_FLAG_PEPRTERR);
    EFM_Lock();
}

/**
 *******************************************************************************
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t u32Addr;
    stc_efm_win_protect_addr_t stcWinAddr;
    const uint32_t u32TestData = 0x5A5A5A5Au;

    /* Clk initialization */
    BSP_CLK_Init();
    /* Led initialization */
    BSP_LED_Init();

    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
        ;
    }

    /* Erase sector 62. */
    EFM_SectorErase(FLASH_SECTOR62_ADRR);

    /* Set windows protect address. */
    stcWinAddr.StartAddr = FLASH_WIN_START_ADDR;
    stcWinAddr.EndAddr = FLASH_WIN_END_ADDR;
    EFM_SetWinProtectAddr(stcWinAddr);

    /* Enable program & erase err interrupt. */
    EFM_InterruptCmd(PgmErsErrInt, Enable);

    /* Set EFM_PEERR interrupt. */
    enShareIrqEnable(INT_EFM_PEERR);

    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(EFM_IRQn);
    NVIC_SetPriority(EFM_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(EFM_IRQn);

    /* program between windows address. */
    u32Addr = FLASH_WIN_START_ADDR + 4ul;
    EFM_SingleProgram(u32Addr,u32TestData);

    /* Key2(SW2) */
    while(0 != PORT_GetBit(PortD, Pin03))
    {
        ;
    }


    /* program out of windows address. */
    u32Addr = FLASH_WIN_START_ADDR - 4ul;
    EFM_SingleProgram(u32Addr,u32TestData);

    EFM_ClearFlag(EFM_FLAG_PEPRTERR);

    /* Lock EFM. */
    EFM_Lock();

    while(1)
    {
        ;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
