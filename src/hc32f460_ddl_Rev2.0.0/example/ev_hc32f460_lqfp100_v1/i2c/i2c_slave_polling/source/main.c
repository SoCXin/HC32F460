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
 ** \brief I2C slave polling sample
 **
 **   - 2018-11-01  CDT First version for Device Driver Library of I2C
 **     Slave polling example
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
#define I2C_UNIT                        (M4_I2C1)
/* Define slave device address for example */
#define SLAVE_ADDRESS                   (0x06u)
//#define I2C_10BITS_ADDRESS              (1u)

/* Define port and pin for SDA and SCL */
#define I2C_SCL_PORT                    (PortC)
#define I2C_SCL_PIN                     (Pin04)
#define I2C_SDA_PORT                    (PortC)
#define I2C_SDA_PIN                     (Pin05)
#define I2C_GPIO_SCL_FUNC               (Func_I2c1_Scl)
#define I2C_GPIO_SDA_FUNC               (Func_I2c1_Sda)

#define I2C_FCG_USE                     (PWC_FCG1_PERIPH_I2C1)

#define TIMEOUT                         (0x10000ul)

/* Define Write and read data length for the example */
#define TEST_DATA_LEN                   (256u)
/* Define i2c baudrate */
#define I2C_BAUDRATE                    (400000ul)

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
 ** \brief  Slave receive data
 **
 ** \param  pu8RxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - Error:              Failed
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
static en_result_t I2C_Slave_Receive(uint8_t *pu8RxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;
    /* clear all status */

    I2C_Cmd(I2C_UNIT, Enable);

    /* Clear status */
    I2C_ClearStatus(I2C_UNIT,I2C_CLR_STOPFCLR | I2C_CLR_NACKFCLR );

    /* Wait slave address matched */
    while(Reset == I2C_GetStatus(I2C_UNIT, I2C_SR_SLADDR0F))
    {
        ;
    }
    I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR);

    if(Reset == I2C_GetStatus(I2C_UNIT, I2C_SR_TRA))
    {
        /* Slave receive data*/
        enRet = I2C_ReceiveData(I2C_UNIT, pu8RxData, u32Size, u32TimeOut);

        if((Ok == enRet) || (ErrorTimeout == enRet))
        {
            /* Wait stop condition */
            enRet = I2C_WaitStatus(I2C_UNIT, I2C_SR_STOPF, Set, u32TimeOut);
        }
    }
    else
    {
        enRet = Error;
    }

    I2C_Cmd(I2C_UNIT, Disable);
    return enRet;
}

/**
 ******************************************************************************
 ** \brief  Slave transmit data
 **
 ** \param  pu8TxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \param  u32TimeOut            Time out count
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                 Success
 **         - Error:              Failed
 **         - ErrorTimeout:       Time out
 ******************************************************************************/
static en_result_t I2C_Slave_Transmit(uint8_t *pu8TxData, uint32_t u32Size, uint32_t u32TimeOut)
{
    en_result_t enRet;

    I2C_Cmd(I2C_UNIT, Enable);

    /* Clear status */
    I2C_ClearStatus(I2C_UNIT,I2C_CLR_STOPFCLR | I2C_CLR_NACKFCLR );

    /* Wait slave address matched */
    while(Reset == I2C_GetStatus(I2C_UNIT, I2C_SR_SLADDR0F))
    {
        ;
    }
    I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR);

#ifdef I2C_10BITS_ADDRESS
    /* Wait slave address matched */
    while(Reset == I2C_GetStatus(I2C_UNIT, I2C_SR_SLADDR0F))
    {
        ;
    }
    I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR);
#endif

    if(Reset == I2C_GetStatus(I2C_UNIT, I2C_SR_TRA))
    {
        enRet = Error;
    }
    else
    {
        enRet = I2C_TransData(I2C_UNIT, pu8TxData, u32Size, u32TimeOut);

        if((Ok == enRet) || (ErrorTimeout == enRet))
        {
            /* Release SCL pin */
            (void)I2C_ReadData(I2C_UNIT);

            /* Wait stop condition */
            enRet = I2C_WaitStatus(I2C_UNIT, I2C_SR_STOPF, Set, u32TimeOut);
        }
    }

    I2C_Cmd(I2C_UNIT, Disable);
    return enRet;
}

/**
 ******************************************************************************
 ** \brief  Initialize the I2C peripheral for slave
 ** \param  None
 ** \return Process result
 **         - Ok:                     Success
 **         - ErrorInvalidParameter:  Invalid parameter
 ******************************************************************************/
static en_result_t Slave_Initialize(void)
{
    en_result_t enRet;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    I2C_DeInit(I2C_UNIT);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    stcI2cInit.u32SclTime = 0ul;
    enRet = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    if(Ok == enRet)
    {
         /* Set slave address */
#ifdef I2C_10BITS_ADDRESS
        I2C_SlaveAdr0Config(I2C_UNIT, Enable, Adr10bit, SLAVE_ADDRESS);
#else
        I2C_SlaveAdr0Config(I2C_UNIT, Enable, Adr7bit, SLAVE_ADDRESS);
#endif
    }
    return enRet;
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
int32_t  main(void)
{
    uint8_t u8RxBuf[TEST_DATA_LEN];

    memset(u8RxBuf, 0x00, TEST_DATA_LEN);

    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();

    /* Initialize I2C port*/
    PORT_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC, Disable);
    PORT_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(I2C_FCG_USE, Enable);
    /* Initialize I2C peripheral and enable function*/
    Slave_Initialize();

    while(1)
    {
        if(Ok == I2C_Slave_Receive(u8RxBuf, TEST_DATA_LEN, TIMEOUT))
        {
            if(Ok != I2C_Slave_Transmit(u8RxBuf, TEST_DATA_LEN, TIMEOUT))
            {
                /* Failed */
                break;
            }
        }
        else
        {
            /* Failed */
            break;
        }
    }

    /* Failed */
    while(1)
    {
        BSP_LED_Toggle(LED_RED);
        Ddl_Delay1ms(500ul);
    }

}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
