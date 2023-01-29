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
 ** \brief HASH sample
 **
 **   - 2018-10-18  CDT First version for Device Driver Library of
 **     Hash
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
#define HASH_MSG_DIGEST_SIZE        (32u)

#define TIMEOUT_VAL                 (10u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void HashConfig(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t m_au8HashMsgDigest[HASH_MSG_DIGEST_SIZE];
const static char *m_su8SrcData = "abcde";

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
    /* Config HASH. */
    HashConfig();

    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    /***************** Configuration end, application start **************/

    while (1u)
    {
        /* Use HASH. */
        HASH_Start((uint8_t *)m_su8SrcData, strlen(m_su8SrcData), \
                    m_au8HashMsgDigest, TIMEOUT_VAL);
        DDL_Printf("String \"%s\" message digest:\n", m_su8SrcData);
        for (uint8_t i = 0u; i < sizeof(m_au8HashMsgDigest); i++)
        {
            DDL_Printf("%.2x ", m_au8HashMsgDigest[i]);
        }
        DDL_Printf("\n");

        /* Main loop cycle is 500ms. */
        Ddl_Delay1ms(500u);
    }
}

/**
 *******************************************************************************
 ** \brief  HASH initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void HashConfig(void)
{
    /* 1. Enable HASH peripheral clock. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_HASH, Enable);

    /* 2. Initialize HASH. */
    HASH_Init();
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
