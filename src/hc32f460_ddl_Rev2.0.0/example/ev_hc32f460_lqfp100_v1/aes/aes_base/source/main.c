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
 ** \brief AES sample
 **
 **   - 2018-10-20  CDT First version for Device Driver Library o
 **     AES
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
static void AesConfig(void);
static void AesFillData(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
const static uint8_t m_au8AesKey[AES_KEYLEN] =
{
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xCD, 0xEF,
    0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7
};

/* Word alignment. */
__ALIGN_BEGIN static uint8_t m_au8Plaintext[64u] = {0u};
__ALIGN_BEGIN static uint8_t m_au8Ciphertext[64u];

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
    uint32_t i;

    /* Config AES. */
    AesConfig();

    /* Config UART for printing. Baud rate 115200. */
    DDL_PrintfInit(BSP_PRINTF_DEVICE, BSP_PRINTF_BAUDRATE, BSP_PRINTF_PortInit);

    /***************** Configuration end, application start **************/

    AesFillData();

    while (1u)
    {
        /* AES encryption. */
        AES_Encrypt(m_au8Plaintext, sizeof(m_au8Plaintext), m_au8AesKey, m_au8Ciphertext);

        DDL_Printf("AES encryption.\n");
        DDL_Printf("Plaintext:\n");
        for (i = 0u; i < sizeof(m_au8Plaintext); i++)
        {
            DDL_Printf("%.2x ", m_au8Plaintext[i]);
        }
        DDL_Printf("Ciphertext:\n");
        for (i = 0u; i < sizeof(m_au8Ciphertext); i++)
        {
            DDL_Printf("%.2x ", m_au8Ciphertext[i]);
        }
        DDL_Printf("\n\n");

        /* AES decryption. */
        DDL_Printf("AES decryption.\n");
        AES_Decrypt(m_au8Ciphertext, sizeof(m_au8Ciphertext), m_au8AesKey, m_au8Plaintext);

        DDL_Printf("Ciphertext:\n");
        for (i = 0u; i < sizeof(m_au8Ciphertext); i++)
        {
            DDL_Printf("%.2x ", m_au8Ciphertext[i]);
        }
        DDL_Printf("Plaintext:\n");
        for (i = 0u; i < sizeof(m_au8Plaintext); i++)
        {
            DDL_Printf("%.2x ", m_au8Plaintext[i]);
        }
        DDL_Printf("\n\n");

        /* Main loop cycle 500ms. */
        Ddl_Delay1ms(500u);
    }
}

/**
 *******************************************************************************
 ** \brief  AES initial configuration.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void AesConfig(void)
{
    /* Enable AES peripheral clock. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AES, Enable);
}

/**
 *******************************************************************************
 ** \brief  Fill data.
 **
 ** \param  None.
 **
 ** \retval None.
 **
 ******************************************************************************/
static void AesFillData(void)
{
    uint32_t i;

    for (i = 0u; i < sizeof(m_au8Plaintext); i++)
    {
        m_au8Plaintext[i] = (uint8_t)(i + 1u);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
