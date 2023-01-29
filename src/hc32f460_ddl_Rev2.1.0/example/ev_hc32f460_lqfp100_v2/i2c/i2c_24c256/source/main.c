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
 ** \brief Bl24c256 write and read example.
 **
 **   - 2021-04-16  CDT First version for Device Driver Library example
 **                      for E2PROM Bl24c256
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
/* Define I2C unit used for the example */
#define I2C_UNIT                        (M4_I2C3)
/* Define E2PROM device address */
#define E2_ADDRESS                      (0x50u)
/* 24c256 page length */
#define E2_PAGE_LEN                     (64u)
#define E2_MEM_ADR_LEN                  (2u)
/* Define test address for read and write */
#define DATA_TEST_ADDR                  (0x0000u)
/* Define port and pin for SDA and SCL */
#define I2C_SCL_PORT                    (PortE)
#define I2C_SCL_PIN                     (Pin15)
#define I2C_SDA_PORT                    (PortB)
#define I2C_SDA_PIN                     (Pin05)
#define I2C_GPIO_SCL_FUNC               (Func_I2c3_Scl)
#define I2C_GPIO_SDA_FUNC               (Func_I2c3_Sda)

#define I2C_FCG_USE                     (PWC_FCG1_PERIPH_I2C3)

#define TIMEOUT                         (0x40000ul)

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
 ******************************************************************************
 ** \brief  I2C memory write
 **
 ** \param  u8DevAddr             The slave address
 ** \param  u16MemAddr            The memory address
 ** \param  pu8Data               Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - Error:              Receive NACK
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
static en_result_t I2C_Mem_Write(uint16_t u8DevAddr, uint16_t u16MemAddr, uint8_t *pu8Data, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;
    uint16_t u16MemAddrTemp;
    u16MemAddrTemp = ((u16MemAddr >> 8) & 0xFFul) + ((u16MemAddr << 8) & 0xFF00ul);

    I2C_Cmd(I2C_UNIT, Enable);

    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet)
    {
        enRet = I2C_TransAddr(I2C_UNIT, u8DevAddr, I2CDirTrans, u32TimeOut);

        if(Ok == enRet)
        {
            enRet = I2C_TransData(I2C_UNIT, (uint8_t *)&u16MemAddrTemp, E2_MEM_ADR_LEN, u32TimeOut);
            if(Ok == enRet)
            {
                enRet = I2C_TransData(I2C_UNIT, pu8Data, u32Size, u32TimeOut);
            }
        }
    }

    I2C_Stop(I2C_UNIT,u32TimeOut);
    I2C_Cmd(I2C_UNIT, Disable);

    return enRet;
}

/**
 ******************************************************************************
 ** \brief  I2C memory read
 **
 ** \param  u8DevAddr             The slave address
 ** \param  u16MemAddr            The memory address
 ** \param  pu8Data               Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
static en_result_t I2C_Mem_Read(uint8_t u8DevAddr, uint16_t u16MemAddr, uint8_t *pu8Data, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;
    uint16_t u16MemAddrTemp;
    u16MemAddrTemp = ((u16MemAddr >> 8) & 0xFFul) + ((u16MemAddr << 8) & 0xFF00ul);

    I2C_Cmd(I2C_UNIT, Enable);

    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    enRet = I2C_Start(I2C_UNIT,u32TimeOut);
    if(Ok == enRet)
    {
        enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u8DevAddr, I2CDirTrans, u32TimeOut);

        if(Ok == enRet)
        {
            enRet = I2C_TransData(I2C_UNIT, (uint8_t *)&u16MemAddrTemp, E2_MEM_ADR_LEN, u32TimeOut);
            if(Ok == enRet)
            {
                enRet = I2C_Restart(I2C_UNIT,u32TimeOut);
                if(Ok == enRet)
                {
                    if(1ul == u32Size)
                    {
                        I2C_AckConfig(I2C_UNIT, I2c_NACK);
                    }

                    enRet = I2C_TransAddr(I2C_UNIT, (uint8_t)u8DevAddr, I2CDirReceive, u32TimeOut);
                    if(Ok == enRet)
                    {
                        enRet = I2C_MasterDataReceiveAndStop(I2C_UNIT, pu8Data, u32Size, u32TimeOut);
                    }

                    I2C_AckConfig(I2C_UNIT, I2c_ACK);
                }

            }
        }
    }

    if(Ok != enRet)
    {
        I2C_Stop(I2C_UNIT,u32TimeOut);
    }

    I2C_Cmd(I2C_UNIT, Disable);
    return enRet;
}

/**
 ******************************************************************************
 ** \brief  Initialize the I2C peripheral for e2prom
 ** \param  None
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
static en_result_t E2_Initialize(void)
{
    stc_i2c_init_t stcI2cInit;
    en_result_t enRet;
    float32_t fErr;

    I2C_DeInit(I2C_UNIT);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV8;
    stcI2cInit.u32Baudrate = 100000ul;
    stcI2cInit.u32SclTime = 0ul;
    enRet = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    I2C_BusWaitCmd(I2C_UNIT, Enable);

    return enRet;
}

/**
 *******************************************************************************
 ** \brief  Main function of template project
 ** \param  None
 ** \retval int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    uint8_t u8TxBuf[E2_PAGE_LEN];
    uint8_t u8RxBuf[E2_PAGE_LEN];
    uint32_t i;

    for(i=0ul; i<E2_PAGE_LEN; i++)
    {
        u8TxBuf[i] = (uint8_t)i+1u;
    }
    memset(u8RxBuf, 0x00, E2_PAGE_LEN);

    /* LED initialization */
    BSP_CLK_Init();
    BSP_LED_Init();

    /* Initialize I2C port*/
    PORT_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC, Disable);
    PORT_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(I2C_FCG_USE, Enable);
    /* Initialize I2C peripheral and enable function*/
    E2_Initialize();

    /* E2prom byte write*/
    I2C_Mem_Write(E2_ADDRESS, DATA_TEST_ADDR, u8TxBuf, 1u, TIMEOUT);

    /* 5mS delay for e2prom*/
    Ddl_Delay1ms(5ul);

    /* E2prom ramdom read*/
    I2C_Mem_Read(E2_ADDRESS, DATA_TEST_ADDR, u8RxBuf, 1u, TIMEOUT);

    /* Compare the data */
    if(0x01u != u8RxBuf[0])
    {
        /* e2prom byte write error*/
        while(1)
        {
            BSP_LED_Toggle(LED_RED);
            Ddl_Delay1ms(500ul);
        }
    }

    /* 5mS delay for e2prom*/
    Ddl_Delay1ms(5ul);
    /* E2prom page write*/
    I2C_Mem_Write(E2_ADDRESS, DATA_TEST_ADDR, u8TxBuf, E2_PAGE_LEN, TIMEOUT);

    /* 5mS delay for e2prom*/
    Ddl_Delay1ms(5ul);

    /* E2prom sequential read*/
    I2C_Mem_Read(E2_ADDRESS, DATA_TEST_ADDR, u8RxBuf, E2_PAGE_LEN, TIMEOUT);

    /* Compare the data */
    for(i=0ul; i<E2_PAGE_LEN; i++)
    {
        if(u8TxBuf[i] != u8RxBuf[i])
        {
            /* e2prom page write error*/
            while(1)
            {
                BSP_LED_Toggle(LED_RED);
                Ddl_Delay1ms(500ul);
            }
        }
    }

    /* e2prom sample success*/
    while(1)
    {
        BSP_LED_Toggle(LED_GREEN);
        Ddl_Delay1ms(500ul);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
