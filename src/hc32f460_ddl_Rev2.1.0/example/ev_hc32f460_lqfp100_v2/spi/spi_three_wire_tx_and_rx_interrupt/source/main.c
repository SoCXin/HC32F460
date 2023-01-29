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
 ** \brief The example of SPI three wire interrupt tx and rx function
 **
 **   - 2021-04-16  CDT  First version for Device Driver Library of SPI.
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
/* SPI_SCK Port/Pin definition */
#define SPI_SCK_PORT                    (PortB)
#define SPI_SCK_PIN                     (Pin15)
#define SPI_SCK_FUNC                    (Func_Spi3_Sck)

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   (PortE)
#define SPI_MOSI_PIN                    (Pin01)
#define SPI_MOSI_FUNC                   (Func_Spi3_Mosi)

/* SPI_MISO Port/Pin definition */
#define SPI_MISO_PORT                   (PortE)
#define SPI_MISO_PIN                    (Pin00)
#define SPI_MISO_FUNC                   (Func_Spi3_Miso)

/* SPI unit and clock definition */
#define SPI_UNIT                        (M4_SPI3)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI3)
#define SPI_TX_INT_SOURCE               (INT_SPI3_SPTI)
#define SPI_RX_INT_SOURCE               (INT_SPI3_SPRI)

/* Choose SPI master or slave mode */
#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static char u8TxBuffer[] = "SPI Master/Slave example: Communication between two boards using SPI interface!";
static char u8RxBuffer[128] = {0};
static uint8_t u8TxIndex = 0u, u8RxIndex = 0u;
static uint16_t u16BufferLen = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief SPI3 send callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void SpiTx_IrqCallback(void)
{
    if (u8TxIndex < u16BufferLen)
    {
        SPI_SendData8(SPI_UNIT, u8TxBuffer[u8TxIndex++]);
    }
    else
    {
        SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Disable);
    }
}

/**
 *******************************************************************************
 ** \brief SPI3 receive callback function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void SpiRx_IrqCallback(void)
{
    if (u8RxIndex < u16BufferLen)
    {
        u8RxBuffer[u8RxIndex++] = SPI_ReceiveData8(SPI_UNIT);
    }
    else
    {
        SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Disable);
    }
}

/**
 *******************************************************************************
 ** \brief Configure SPI peripheral function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Spi_Config(void)
{
    stc_spi_init_t stcSpiInit;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv64;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddChangeEvenSample;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode3Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;

#ifdef SPI_MASTER_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
#endif
    SPI_Init(SPI_UNIT, &stcSpiInit);

    /* SPI3 tx interrupt */
    stcIrqRegiConf.enIntSrc = SPI_TX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = Int005_IRQn;
    stcIrqRegiConf.pfnCallback = &SpiTx_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* SPI3 rx interrupt */
    stcIrqRegiConf.enIntSrc = SPI_RX_INT_SOURCE;
    stcIrqRegiConf.enIRQn = Int006_IRQn;
    stcIrqRegiConf.pfnCallback = &SpiRx_IrqCallback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief  main function for three wire SPI interrupt tx and rx function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Configure SPI */
    Spi_Config();
    /* Get tx buffer length */
    u16BufferLen = (uint16_t)sizeof(u8TxBuffer);

    while (1)
    {
        /* Wait key trigger in master mode */
#ifdef SPI_MASTER_MODE
        while (Reset == BSP_KEY_GetStatus(BSP_KEY_1))
        {
        }
#endif
        u8TxIndex = 0u;
        u8RxIndex = 0u;
        memset(u8RxBuffer, 0l, sizeof(u8RxBuffer));
        /* Enable tx and rx interrupt */
        SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Enable);
        SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Enable);
        SPI_Cmd(SPI_UNIT, Enable);
        /* Wait end of send and receive */
        while ((u8TxIndex < u16BufferLen) || (u8RxIndex < u16BufferLen))
        {
        }
        /* Disable tx and rx interrupt */
        SPI_IrqCmd(SPI_UNIT, SpiIrqReceive, Disable);
        SPI_IrqCmd(SPI_UNIT, SpiIrqSend, Disable);
        SPI_Cmd(SPI_UNIT, Disable);

        /* Compare u8TxBuffer and u8RxBuffer */
        if (memcmp(u8TxBuffer, u8RxBuffer, (uint32_t)u16BufferLen) != 0)
        {
            BSP_LED_On(LED_RED);
            BSP_LED_Off(LED_GREEN);
        }
        else
        {
            BSP_LED_Off(LED_RED);
            BSP_LED_On(LED_GREEN);
        }
#ifdef SPI_MASTER_MODE
        /* Wait for the slave to be ready */
        Ddl_Delay1ms(10);
#endif
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
