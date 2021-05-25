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
 ** \brief CRC sample
 **
 **   - 2019-03-11  CDT First version for Device Driver Library of
 **     CRC
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
static void CrcConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None.
 **
 ** \retval int32_t return value, if needed.
 **
 ******************************************************************************/
int32_t main(void)
{
    bool bFlag;
    uint16_t u16InitVal;
    uint16_t u16Checksum;
    uint16_t au16Data[2u] = {0x1234, 0x5678};

    uint32_t u32InitVal;
    uint32_t u32Checksum;
    uint32_t au32Data[2u] = {0x12345678u, 0x87654321u};

    /* Config CRC. */
    CrcConfig();

    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    /***************** Configuration end, application start **************/
    while (1u)
    {
        /* CRC16 usage. */
        u16InitVal = 0x0u;
        CRC_Init(CRC_SEL_16B | CRC_REFIN_DISABLE | CRC_REFOUT_DISABLE | CRC_XOROUT_DISABLE);
        u16Checksum = CRC_Calculate16B(u16InitVal, au16Data, 2u);
        DDL_Printf("CRC16 result = %.4x\n", u16Checksum);
        bFlag = CRC_Check16B(u16InitVal, u16Checksum, au16Data, 2u);
        DDL_Printf("CRC16 flag = %d\n", bFlag);

        /* Change the CRC configuration. */
        /* The bits of the checksum will be transposed if CRC_REFOUT is enabled. */
        CRC_Init(CRC_SEL_16B | CRC_REFIN_DISABLE | CRC_REFOUT_ENABLE | CRC_XOROUT_DISABLE);
        u16Checksum = CRC_Calculate16B(u16InitVal, au16Data, 2u);
        DDL_Printf("CRC16 result = %.4x\n", u16Checksum);
        bFlag = CRC_Check16B(u16InitVal, u16Checksum, au16Data, 2u);
        DDL_Printf("CRC16 flag = %d\n", bFlag);

        /* CRC32 usage. */
        u32InitVal = 0xFFFFFFFFu;
        CRC_Init(CRC_SEL_32B | CRC_REFIN_ENABLE | CRC_REFOUT_ENABLE | CRC_XOROUT_DISABLE);
        u32Checksum = CRC_Calculate32B(u32InitVal, au32Data, 2u);
        DDL_Printf("CRC32 result = %.8lx\n", u32Checksum);
        bFlag = CRC_Check32B(u32InitVal, u32Checksum, au32Data, 2u);
        DDL_Printf("CRC32 flag = %d\n", bFlag);

        /* Changes the CRC configuration. */
        CRC_Init(CRC_SEL_32B | CRC_REFIN_DISABLE | CRC_REFOUT_DISABLE | CRC_XOROUT_ENABLE);
        u32Checksum = CRC_Calculate32B(u32InitVal, au32Data, 2u);
        DDL_Printf("CRC32 result = %.8lx\n", u32Checksum);
        bFlag = CRC_Check32B(u32InitVal, u32Checksum, au32Data, 2u);
        DDL_Printf("CRC32 flag = %d\n", bFlag);
    }
}

/**
 *******************************************************************************
 ** \brief  CRC initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void CrcConfig(void)
{
    /* 1. Enable CRC. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_CRC, Enable);

    /* 2. Initializes CRC here or before every CRC calculation. */
    CRC_Init(CRC_SEL_16B | CRC_REFIN_DISABLE | CRC_REFOUT_DISABLE | CRC_XOROUT_DISABLE);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
