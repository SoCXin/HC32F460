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
 ** \brief This example demonstrates how to use SDIOC to read/write SDCard by
 **        high-speed(50MHz) && 4 bits && multiple blocks mode.
 **
 **   - 2018-11-08 CDT First version for Device Driver Library of SDIOC
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "sd_card.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* SDIOC Port/Pin definition */
#define SDIOC_CD_PORT                   (PortE)
#define SDIOC_CD_PIN                    (Pin14)

#define SDIOC_CK_PORT                   (PortC)
#define SDIOC_CK_PIN                    (Pin12)

#define SDIOC_CMD_PORT                  (PortD)
#define SDIOC_CMD_PIN                   (Pin02)

#define SDIOC_D0_PORT                   (PortC)
#define SDIOC_D0_PIN                    (Pin08)

#define SDIOC_D1_PORT                   (PortC)
#define SDIOC_D1_PIN                    (Pin09)

#define SDIOC_D2_PORT                   (PortC)
#define SDIOC_D2_PIN                    (Pin10)

#define SDIOC_D3_PORT                   (PortC)
#define SDIOC_D3_PIN                    (Pin11)

/* SD sector && count */
#define SD_SECTOR_START                 (0u)
#define SD_SECTOR_COUNT                 (4u)

/* SDIOC unit */
#define SDIOC_UNIT                      (M4_SDIOC1)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static en_result_t SdiocInitPins(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief Initialize SDIO pins
 **
 ** \param [in] None
 **
 ** \retval Ok  SDIO pins initialized successfully
 **
 ******************************************************************************/
static en_result_t SdiocInitPins(void)
{
    PORT_SetFunc(SDIOC_D0_PORT, SDIOC_D0_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D1_PORT, SDIOC_D1_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D2_PORT, SDIOC_D2_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D3_PORT, SDIOC_D3_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CD_PORT, SDIOC_CD_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CK_PORT, SDIOC_CK_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CMD_PORT, SDIOC_CMD_PIN, Func_Sdio, Disable);

    return Ok;
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
    uint32_t i;
    en_result_t enTestResult = Ok;
    static stc_sd_handle_t stcSdhandle;
    static uint32_t au32WriteBlocks[512];
    static uint32_t au32ReadBlocks[ARRAY_SZ(au32WriteBlocks)];
    stc_sdcard_init_t stcCardInitCfg =
    {
        SdiocBusWidth4Bit,
        SdiocClk50M,
        SdiocHighSpeedMode,
        NULL,
    };

    /* Write buffer data */
    for (i = 0u; i < ARRAY_SZ(au32WriteBlocks); i++)
    {
        au32WriteBlocks[i] = i;
    }

    MEM_ZERO_STRUCT(au32ReadBlocks);

    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();

    /* Initialize SDIOC pin */
    SdiocInitPins();

    /* Initialize SD card */
    stcSdhandle.SDIOCx = SDIOC_UNIT;
    if (Ok != SDCARD_Init(&stcSdhandle, &stcCardInitCfg))
    {
        enTestResult = Error;
    }

    /* Erase SD card */
    if (Ok != SDCARD_Erase(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, 20000u))
    {
        enTestResult = Error;
    }

    /* Read SD card */
    if (Ok != SDCARD_ReadBlocks(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)au32ReadBlocks, 2000u))
    {
        enTestResult = Error;
    }

    /* Check whether data value is OxFFFFFFFF or 0x00000000 after erase SD card */
    for (i = 0u; i < ARRAY_SZ(au32WriteBlocks); i++)
    {
        if ((au32ReadBlocks[i] != 0xFFFFFFFFul) &&
            (au32ReadBlocks[i] != 0x00000000ul))
        {
            enTestResult = Error;
            break;
        }
    }

    /* Write SD card */
    if (Ok != SDCARD_WriteBlocks(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)au32WriteBlocks, 2000u))
    {
        enTestResult = Error;
    }

    /* Read SD card */
    if (Ok != SDCARD_ReadBlocks(&stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)au32ReadBlocks, 20000u))
    {
        enTestResult = Error;
    }

    /* Compare read/write data */
    if (0 != memcmp(au32WriteBlocks, au32ReadBlocks, sizeof(au32ReadBlocks)))
    {
        enTestResult = Error;
    }

    if (Ok == enTestResult)
    {
        BSP_LED_On(LED_GREEN);    /* Test pass && meet the expected */
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
