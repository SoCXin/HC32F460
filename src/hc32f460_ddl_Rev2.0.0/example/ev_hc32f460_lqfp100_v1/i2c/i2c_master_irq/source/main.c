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
 ** \brief I2C master irq sample
 **
 **   - 2020-11-19  CDT First version for Device Driver Library of I2C
 **     master irq example
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 * @brief I2c communication mode enum
 */
typedef enum
{
    Mode_Send = 0u,
    Mode_Rev = 1u,
}stc_i2c_com_mode_t;

/**
 * @brief I2c communication status enum
 */
typedef enum
{
    I2C_ComBusy = 0u,
    I2C_ComIdle = 1u,
}stc_i2c_com_status_t;

/**
 * @brief I2c communication structure
 */
typedef struct
{
    stc_i2c_com_mode_t    enMode;         /*!< I2C communication mode*/
    uint32_t              u32Length;      /*!< I2C communication data length*/
    uint8_t*              pBuf;           /*!< I2C communication data buffer pointer*/
    __IO uint32_t         u32DataIndex;   /*!< I2C communication data transfer index*/
    stc_i2c_com_status_t  enComStatus;    /*!< I2C communication status*/
}stc_i2c_communication_t;

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Define I2C unit used for the example */
#define I2C_UNIT                        (M4_I2C1)
/* Define slave device address for example */
#define SLAVE_ADDRESS                   (0x06u)

/* Define port and pin for SDA and SCL */
#define I2C_SCL_PORT                    (PortC)
#define I2C_SCL_PIN                     (Pin04)
#define I2C_SDA_PORT                    (PortC)
#define I2C_SDA_PIN                     (Pin05)
#define I2C_GPIO_SCL_FUNC               (Func_I2c1_Scl)
#define I2C_GPIO_SDA_FUNC               (Func_I2c1_Sda)

#define I2C_INT_EEI_DEF                 (INT_I2C1_EEI)
#define I2C_INT_RXI_DEF                 (INT_I2C1_RXI)
#define I2C_INT_TXI_DEF                 (INT_I2C1_TXI)
#define I2C_INT_TEI_DEF                 (INT_I2C1_TEI)

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
static void BufWrite(uint8_t au8Buf[], uint8_t u8Data);
static uint8_t BufRead(uint8_t au8Buf[]);
static void Master_Start(void);
void Master_SendAddr(uint8_t u8Addr, en_trans_direction_t enDir);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8TxBuf[TEST_DATA_LEN];
static uint8_t u8RxBuf[TEST_DATA_LEN];
static stc_i2c_communication_t stcI2cCom;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Master send data kick start via interrupt .
 ** \param  pu8TxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                    Success
 **         - OperationInProgress:   Busy
 ******************************************************************************/
static en_result_t I2C_Master_Transmit_IT(uint8_t *pu8TxData, uint32_t u32Size)
{
    en_result_t enRet = Ok;

    if(I2C_ComIdle == stcI2cCom.enComStatus)
    {
        stcI2cCom.enComStatus = I2C_ComBusy;

        stcI2cCom.u32DataIndex = 0u;
        stcI2cCom.enMode = Mode_Send;
        stcI2cCom.u32Length = u32Size;
        stcI2cCom.pBuf = pu8TxData;

        /* General start condition */
        Master_Start();
    }
    else
    {
        enRet = OperationInProgress;
    }

    return enRet;
}

/**
 ******************************************************************************
 ** \brief  Master Rev data kick start via interrupt .
 ** \param  pu8RxData             Pointer to the data buffer
 ** \param  u32Size               Data size
 ** \retval en_result_t           Enumeration value:
 **         - Ok:                    Success
 **         - OperationInProgress:   Busy
 ******************************************************************************/
static en_result_t I2C_Master_Receive_IT(uint8_t *pu8RxData, uint32_t u32Size)
{
    en_result_t enRet = Ok;

    if(I2C_ComIdle == stcI2cCom.enComStatus)
    {
        stcI2cCom.enComStatus = I2C_ComBusy;
        stcI2cCom.u32DataIndex = 0u;
        stcI2cCom.enMode = Mode_Rev;
        stcI2cCom.u32Length = u32Size;
        stcI2cCom.pBuf = pu8RxData;

        /* General start condition */
        Master_Start();
    }
    else
    {
        enRet = OperationInProgress;
    }

    return enRet;
}

/**
 ******************************************************************************
 ** \brief   I2C EEI(communication error or event) interrupt callback function
 ** \param   None
 ** \retval  None
 ******************************************************************************/
static void I2C_EEI_Callback(void)
{
    /* If starf flag valid */
    if(Set == I2C_GetStatus(I2C_UNIT, I2C_SR_STARTF))
    {
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_STARTFCLR | I2C_CLR_NACKFCLR);
        I2C_IntCmd(I2C_UNIT, I2C_CR2_STOPIE | I2C_CR2_NACKIE, Enable);

        if(Mode_Send == stcI2cCom.enMode)
        {
            /* Enable TEI interrupt which indicate address transfer end */
            I2C_IntCmd(I2C_UNIT, I2C_CR2_TENDIE, Enable);

            Master_SendAddr(SLAVE_ADDRESS, I2CDirTrans);
        }
        else
        {
            /* if read data length is 1 */
            if(stcI2cCom.u32Length == 1u)
            {
                I2C_AckConfig(I2C_UNIT, I2c_NACK);
            }

            /* Enable RXI interrupt which indicate data receive buffer full */
            I2C_IntCmd(I2C_UNIT, I2C_CR2_RFULLIE, Enable);

            Master_SendAddr(SLAVE_ADDRESS, I2CDirReceive);
        }
    }

    /* If NACK interrupt occurred */
    if(Set == I2C_GetStatus(I2C_UNIT, I2C_SR_NACKF))
    {
        /* clear NACK flag*/
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_NACKFCLR);
        /* Stop tx or rx process*/
        I2C_IntCmd(I2C_UNIT, I2C_CR2_TEMPTYIE | I2C_CR2_RFULLIE | I2C_CR2_TENDIE | I2C_CR2_NACKIE, Disable);

        /* Generate stop condition */
        I2C_GenerateStop(I2C_UNIT, Enable);
    }

    if(Set == I2C_GetStatus(I2C_UNIT, I2C_SR_STOPF))
    {
        /* Disable Stop flag interrupt */
        I2C_IntCmd(I2C_UNIT,                                                   \
                   I2C_CR2_TEMPTYIE | I2C_CR2_RFULLIE | I2C_CR2_TENDIE |       \
                   I2C_CR2_STOPIE | I2C_CR2_NACKIE,                            \
                   Disable);
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_STOPFCLR);
        I2C_Cmd(I2C_UNIT, Disable);

        /* Communication finished */
        stcI2cCom.enComStatus = I2C_ComIdle;
    }
}

/**
 ******************************************************************************
 ** \brief   I2C TXI(transfer buffer empty) interrupt callback function
 ** \param   None
 ** \retval  None
 ******************************************************************************/
static void I2C_TXI_Callback(void)
{
    if(stcI2cCom.u32DataIndex <= (stcI2cCom.u32Length - 1u))
    {
        I2C_WriteData(I2C_UNIT, BufRead(stcI2cCom.pBuf));
    }
    else
    {
        I2C_IntCmd(I2C_UNIT, I2C_CR2_TEMPTYIE, Disable);
        I2C_IntCmd(I2C_UNIT, I2C_CR2_TENDIE, Enable);
    }
}

/**
 ******************************************************************************
 ** \brief   I2C RXI(receive buffer full) interrupt callback function
 ** \param   None
 ** \retval  None
 ******************************************************************************/
static void I2C_RXI_Callback(void)
{
    if((stcI2cCom.u32Length >= 2u) && (stcI2cCom.u32DataIndex == (stcI2cCom.u32Length - 2u)))
    {
        I2C_AckConfig(I2C_UNIT, I2c_NACK);
    }

    if(stcI2cCom.u32DataIndex == (stcI2cCom.u32Length - 1u))
    {
        /* Enable Stop flag interrupt */
        I2C_IntCmd(I2C_UNIT, I2C_CR2_STOPIE, Enable);
        /* Disable RXI interrupt */
        I2C_IntCmd(I2C_UNIT, I2C_CR2_RFULLIE, Disable);
        /* Generate stop condition */
        I2C_GenerateStop(I2C_UNIT, Enable);

        I2C_AckConfig(I2C_UNIT, I2c_ACK);
    }

    BufWrite(stcI2cCom.pBuf, I2C_ReadData(I2C_UNIT));
}

/**
 ******************************************************************************
 ** \brief   I2C TEI(Transfer end) interrupt callback function
 ** \param   None
 ** \retval  None
 ******************************************************************************/
static void I2C_TEI_Callback(void)
{
    I2C_IntCmd(I2C_UNIT, I2C_CR2_TENDIE, Disable);

    if(stcI2cCom.u32DataIndex == 0u)
    {
        /* Indicate address send finished */
        if(Set == I2C_GetStatus(I2C_UNIT, I2C_SR_TRA))
        {
            /* If Address send receive ACK */
            if(Reset == I2C_GetStatus(I2C_UNIT, I2C_SR_ACKRF))
            {
                /* Config tx buffer empty interrupt function*/
                I2C_IntCmd(I2C_UNIT, I2C_CR2_TEMPTYIE, Enable);
                /* Need transfer first data here */
                I2C_WriteData(I2C_UNIT, BufRead(stcI2cCom.pBuf));
            }
            else
            {
                I2C_IntCmd(I2C_UNIT, I2C_CR2_NACKIE, Disable);
                /* Generate stop if receive NACK */
                I2C_IntCmd(I2C_UNIT, I2C_CR2_STOPIE, Enable);
                /* Generate stop condition */
                I2C_GenerateStop(I2C_UNIT, Enable);
            }
        }
    }
    else
    {
        /* Data Send finish */
        I2C_IntCmd(I2C_UNIT, I2C_CR2_STOPIE, Enable);
        /* Generate stop condition */
        I2C_GenerateStop(I2C_UNIT, Enable);
    }
}

/**
 ******************************************************************************
 ** \brief   Initialize the I2C peripheral for master
 ** \param   None
 ** \retval en_result_t                Enumeration value:
 **          - Ok:                     Success
 **          - ErrorInvalidParameter:  Invalid parameter
 ******************************************************************************/
static en_result_t Master_Initialize(void)
{
    en_result_t enRet;
    stc_i2c_init_t stcI2cInit;
    stc_irq_regi_conf_t stcIrqRegiConf;
    float32_t fErr;

    I2C_DeInit(I2C_UNIT);

    stcI2cCom.enComStatus = I2C_ComIdle;

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = 400000ul;
    stcI2cInit.u32SclTime = 5U;
    enRet = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);
    if(Ok == enRet)
    {
        /* Register EEI Int to Vect.No.001 */
        stcIrqRegiConf.enIRQn = Int001_IRQn;
        /* Select I2C Error or Event interrupt function */
        stcIrqRegiConf.enIntSrc = I2C_INT_EEI_DEF;
        /* Callback function */
        stcIrqRegiConf.pfnCallback = &I2C_EEI_Callback;
        /* Registration IRQ */
        enIrqRegistration(&stcIrqRegiConf);
        /* Clear Pending */
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        /* Set priority */
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        /* Enable NVIC */
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        /* Register RXI Int to Vect.No.002 */
        stcIrqRegiConf.enIRQn = Int002_IRQn;
        /* Select I2C receive full interrupt function */
        stcIrqRegiConf.enIntSrc = I2C_INT_RXI_DEF;
        /* Callback function */
        stcIrqRegiConf.pfnCallback = &I2C_RXI_Callback;
        /* Registration IRQ */
        enIrqRegistration(&stcIrqRegiConf);
        /* Clear Pending */
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        /* Set priority */
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        /* Enable NVIC */
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        /* Register TEI Int to Vect.No.003 */
        stcIrqRegiConf.enIRQn = Int003_IRQn;
        /* Select I2C TX end interrupt function */
        stcIrqRegiConf.enIntSrc = I2C_INT_TEI_DEF;
        /* Callback function */
        stcIrqRegiConf.pfnCallback = &I2C_TEI_Callback;
        /* Registration IRQ */
        enIrqRegistration(&stcIrqRegiConf);
        /* Clear Pending */
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        /* Set priority */
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        /* Enable NVIC */
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        /* Register TEI Int to Vect.No.004 */
        stcIrqRegiConf.enIRQn = Int004_IRQn;
        /* Select I2C TX buffer empty interrupt function */
        stcIrqRegiConf.enIntSrc = I2C_INT_TXI_DEF;
        /* Callback function */
        stcIrqRegiConf.pfnCallback = &I2C_TXI_Callback;
        /* Registration IRQ */
        enIrqRegistration(&stcIrqRegiConf);
        /* Clear Pending */
        NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
        /* Set priority */
        NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
        /* Enable NVIC */
        NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

        I2C_BusWaitCmd(I2C_UNIT, Enable);
    }
    return enRet;
}

/**
 ******************************************************************************
 ** \brief  Send Start condition.
 ** \param  None
 ** \retval None
 ******************************************************************************/
static void Master_Start(void)
{
    /* I2C function command */
    I2C_Cmd(I2C_UNIT, Enable);
    /* Config startf and slave address match interrupt function*/
    I2C_IntCmd(I2C_UNIT, I2C_CR2_STARTIE | I2C_CR2_SLADDR0EN, Enable);

    I2C_SoftwareResetCmd(I2C_UNIT, Enable);
    I2C_SoftwareResetCmd(I2C_UNIT, Disable);
    /* generate start signal */
    I2C_GenerateStart(I2C_UNIT, Enable);
}

/**
 ******************************************************************************
 ** \brief  Send slave address
 **
 ** \param  u8Addr            The slave address
 ** \param  enDir             Can be I2CDirTrans or I2CDirReceive
 ** \retval None
 ******************************************************************************/
void Master_SendAddr(uint8_t u8Addr, en_trans_direction_t enDir)
{
    /* Send I2C address */
    I2C_WriteData(I2C_UNIT, (u8Addr << 1u) | (uint8_t)enDir);
}

/**
 ******************************************************************************
 ** \brief   static function for buffer write.
 ** \param   [in]   au8Buf     Buffer pointer
 ** \param   [in]   u8Data     The data to be write.
 ** \retval  None
 ******************************************************************************/
static void BufWrite(uint8_t au8Buf[], uint8_t u8Data)
{
    if(stcI2cCom.u32DataIndex >= stcI2cCom.u32Length)
    {
        /* error */
        for(;;)
        {
            ;
        }
    }
    au8Buf[stcI2cCom.u32DataIndex] = u8Data;
    stcI2cCom.u32DataIndex++;
}

/**
 ******************************************************************************
 ** \brief   Static function for buffer read.
 ** \param   [in]   au8Buf[]   Buffer pointer
 ** \retval  uint8_t  The data read out from buffer.
 ******************************************************************************/
static uint8_t BufRead(uint8_t au8Buf[])
{
    uint8_t temp;
    if(stcI2cCom.u32DataIndex >= stcI2cCom.u32Length)
    {
        /* error */
        for(;;)
        {
            ;
        }
    }
    temp = au8Buf[stcI2cCom.u32DataIndex];
    stcI2cCom.u32DataIndex++;

    return temp;
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
    uint32_t i;

    for(i=0U; i<TEST_DATA_LEN; i++)
    {
        u8TxBuf[i] = (uint8_t)(i+1u);
    }
    memset(u8RxBuf, 0x00u, TEST_DATA_LEN);

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
    if(Ok != Master_Initialize())
    {
        /* Initialize error*/
        BSP_LED_Toggle(LED_RED);
        for(;;)
        {
            ;
        }
    }

    /* Wait key */
    while (Reset == BSP_KEY_GetStatus(BSP_KEY_2))
    {
        ;
    }

    /* I2C master data write */
    while(Ok != I2C_Master_Transmit_IT(u8TxBuf, TEST_DATA_LEN));

    /* Wait communication finished*/
    while(I2C_ComBusy == stcI2cCom.enComStatus)
    {
        if(TIMEOUT == i)
        {
            /* Communication time out*/
            BSP_LED_Toggle(LED_RED);
            for(;;)
            {
                ;
            }
        }
        i++;
    }

    /* 5mS delay for device*/
    Ddl_Delay1ms(500ul);

    while(Ok != I2C_Master_Receive_IT(u8RxBuf, TEST_DATA_LEN));

    i = 0UL;
    /* Wait communicaiton finished*/
    while(I2C_ComBusy == stcI2cCom.enComStatus)
    {
        if(TIMEOUT == i)
        {
            /* Communication time out*/
            BSP_LED_Toggle(LED_RED);
            for(;;)
            {
                ;
            }
        }
        i++;
    }

    /* Compare the data */
    for(i=0U; i<TEST_DATA_LEN; i++)
    {
        if(u8TxBuf[i] != u8RxBuf[i])
        {
            /* Data write error*/
            BSP_LED_Toggle(LED_RED);
            while(1)
            {
                BSP_LED_Toggle(LED_RED);
                Ddl_Delay1ms(500ul);
            }
        }
    }

    /* Communication finished */
    BSP_LED_Toggle(LED_GREEN);
    while(1)
    {
        BSP_LED_Toggle(LED_GREEN);
        Ddl_Delay1ms(500ul);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
