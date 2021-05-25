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
 **   - 2021-04-16  CDT  First version for Device Driver Library of
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
#define FLASH_SECTOR62_ADRR             (0x0007C000u)
#define FLASH_SECTOR61_ADDR             (0x0007A000u)
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
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint8_t u8TestBuf[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
    uint8_t u8Len = 18u;

    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY))
    {
        ;
    }

    /* Erase sector 61 62. */
    EFM_SectorErase(FLASH_SECTOR61_ADDR);
    EFM_SectorErase(FLASH_SECTOR62_ADRR);

    /* Sequence program. */
    EFM_SequenceProgram(FLASH_SECTOR61_ADDR, (uint32_t)u8Len, u8TestBuf);
    EFM_SequenceProgram(FLASH_SECTOR62_ADRR, (uint32_t)u8Len, u8TestBuf);

    EFM_MassErase(FLASH_SECTOR61_ADDR);

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
