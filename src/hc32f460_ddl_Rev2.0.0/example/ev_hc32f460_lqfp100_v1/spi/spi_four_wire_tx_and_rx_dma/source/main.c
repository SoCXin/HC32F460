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
 ** \brief The example of SPI four wire DMA tx and rx function
 **
 **   - 2018-12-03  CDT  First version for Device Driver Library of SPI.
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
#define SPI_SCK_PORT                    (PortE)
#define SPI_SCK_PIN                     (Pin00)
#define SPI_SCK_FUNC                    (Func_Spi3_Sck)

/* SPI_NSS Port/Pin definition */
#define SPI_NSS_PORT                    (PortE)
#define SPI_NSS_PIN                     (Pin01)
#define SPI_NSS_FUNC                    (Func_Spi3_Nss0)

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   (PortE)
#define SPI_MOSI_PIN                    (Pin02)
#define SPI_MOSI_FUNC                   (Func_Spi3_Mosi)

/* SPI_MISO Port/Pin definition */
#define SPI_MISO_PORT                   (PortE)
#define SPI_MISO_PIN                    (Pin03)
#define SPI_MISO_FUNC                   (Func_Spi3_Miso)

/* SPI unit and clock definition */
#define SPI_UNIT                        (M4_SPI3)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI3)

/* SPI DMA unit and channel definition */
#define SPI_DMA_UNIT                    (M4_DMA1)
#define SPI_DMA_CLOCK_UNIT              (PWC_FCG0_PERIPH_DMA1)
#define SPI_DMA_TX_CHANNEL              (DmaCh1)
#define SPI_DMA_RX_CHANNEL              (DmaCh0)
#define SPI_DMA_TX_TRIG_SOURCE          (EVT_SPI3_SPTI)
#define SPI_DMA_RX_TRIG_SOURCE          (EVT_SPI3_SPRI)

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
static uint16_t u16BufferLen = 0u;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
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

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(SPI_NSS_PORT, SPI_NSS_PIN, SPI_NSS_FUNC, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);
    PORT_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv64;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode4Line;
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
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif

#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif
    SPI_Init(SPI_UNIT, &stcSpiInit);
}

/**
 *******************************************************************************
 ** \brief Configure SPI DMA function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Spi_DmaConfig(void)
{
    stc_dma_config_t stcDmaCfg;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Configuration peripheral clock */
    PWC_Fcg0PeriphClockCmd(SPI_DMA_CLOCK_UNIT, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Configure TX DMA */
    stcDmaCfg.u16BlockSize = 1u;
    stcDmaCfg.u16TransferCnt = u16BufferLen;
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u8TxBuffer[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&SPI_UNIT->DR);
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;
    stcDmaCfg.stcDmaChCfg.enIntEn = Disable;
    DMA_InitChannel(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, &stcDmaCfg);

    /* Configure RX DMA */
    stcDmaCfg.u16BlockSize = 1u;
    stcDmaCfg.u16TransferCnt = u16BufferLen;
    stcDmaCfg.u32SrcAddr = (uint32_t)(&SPI_UNIT->DR);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u8RxBuffer[0]);
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressFix;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;
    stcDmaCfg.stcDmaChCfg.enIntEn = Disable;
    DMA_InitChannel(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, &stcDmaCfg);

    DMA_SetTriggerSrc(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, SPI_DMA_TX_TRIG_SOURCE);
    DMA_SetTriggerSrc(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, SPI_DMA_RX_TRIG_SOURCE);

    /* Enable DMA. */
    DMA_Cmd(SPI_DMA_UNIT, Enable);
}

/**
 *******************************************************************************
 ** \brief  main function for four wire SPI DMA tx and rx function
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
    Spi_DmaConfig();

    while (1)
    {
        /* Wait key trigger in master mode */
#ifdef SPI_MASTER_MODE
        while (Reset == BSP_KEY_GetStatus(BSP_KEY_2))
        {
        }
#endif
        memset(u8RxBuffer, 0l, sizeof(u8RxBuffer));
        DMA_SetSrcAddress(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, (uint32_t)(&u8TxBuffer[0]));
        DMA_SetTransferCnt(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, u16BufferLen);
        DMA_SetDesAddress(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, (uint32_t)(&u8RxBuffer[0]));
        DMA_SetTransferCnt(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, u16BufferLen);
        /* Enable DMA channel */
        DMA_ChannelCmd(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, Enable);
        DMA_ChannelCmd(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, Enable);
        /* Enable SPI to start DMA */
        SPI_Cmd(SPI_UNIT, Enable);
        while (Reset == DMA_GetIrqFlag(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, TrnCpltIrq))
        {
        }
        while (Reset == DMA_GetIrqFlag(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, TrnCpltIrq))
        {
        }
        DMA_ClearIrqFlag(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, TrnCpltIrq);
        DMA_ClearIrqFlag(SPI_DMA_UNIT, SPI_DMA_RX_CHANNEL, TrnCpltIrq);
        /* Disable SPI */
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
