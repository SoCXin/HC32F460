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
 ** \brief The example of SPI write and read flash function
 **
 **   - 2018-11-09  CDT  First version for Device Driver Library of SPI.
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
#define SPI_SCK_PORT                    (PortC)
#define SPI_SCK_PIN                     (Pin06)
#define SPI_SCK_FUNC                    (Func_Spi3_Sck)

/* SPI_NSS Port/Pin definition */
#define SPI_NSS_PORT                    (PortC)
#define SPI_NSS_PIN                     (Pin07)
#define SPI_NSS_HIGH()                  (PORT_SetBits(SPI_NSS_PORT, SPI_NSS_PIN))
#define SPI_NSS_LOW()                   (PORT_ResetBits(SPI_NSS_PORT, SPI_NSS_PIN))

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   (PortD)
#define SPI_MOSI_PIN                    (Pin08)
#define SPI_MOSI_FUNC                   (Func_Spi3_Mosi)

/* SPI_MISO Port/Pin definition */
#define SPI_MISO_PORT                   (PortD)
#define SPI_MISO_PIN                    (Pin09)
#define SPI_MISO_FUNC                   (Func_Spi3_Miso)

/* SPI unit and clock definition */
#define SPI_UNIT                        (M4_SPI3)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI3)

/* FLASH parameters */
#define FLASH_PAGE_SIZE                 (0x100u)
#define FLASH_SECTOR_SIZE               (0x1000u)
#define FLASH_MAX_ADDR                  (0x800000ul)
#define FLASH_DUMMY_BYTE_VALUE          (0xffu)
#define FLASH_BUSY_BIT_MASK             (0x01u)

/* FLASH instruction */
#define FLASH_INSTR_WRITE_ENABLE        (0x06u)
#define FLASH_INSTR_PAGE_PROGRAM        (0x02u)
#define FLASH_INSTR_STANDARD_READ       (0x03u)
#define FLASH_INSTR_ERASE_4KB_SECTOR    (0x20u)
#define FLASH_INSTR_READ_SR1            (0x05u)

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
    stcSpiInit.enWorkMode = SpiWorkMode3Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;

    SPI_Init(SPI_UNIT, &stcSpiInit);
    SPI_Cmd(SPI_UNIT, Enable);
}

/**
 *******************************************************************************
 ** \brief SPI flash write byte function
 **
 ** \param [in] u8Data                      SPI write data to flash
 **
 ** \retval uint8_t                         SPI receive data from flash
 **
 ******************************************************************************/
static uint8_t SpiFlash_WriteReadByte(uint8_t u8Data)
{
    uint8_t u8Byte;

    /* Wait tx buffer empty */
    while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSendBufferEmpty))
    {
    }
    /* Send data */
    SPI_SendData8(SPI_UNIT, u8Data);
    /* Wait rx buffer full */
    while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagReceiveBufferFull))
    {
    }
    /* Receive data */
    u8Byte = SPI_ReceiveData8(SPI_UNIT);

    return u8Byte;
}

/**
 *******************************************************************************
 ** \brief SPI flash write enable function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SpiFlash_WriteEnable(void)
{
    SPI_NSS_LOW();
    SpiFlash_WriteReadByte(FLASH_INSTR_WRITE_ENABLE);
    while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSpiIdle))
    {
    }
    SPI_NSS_HIGH();
}

/**
 *******************************************************************************
 ** \brief SPI flash wait for write operation end function
 **
 ** \param [in] None
 **
 ** \retval Ok                              Flash internal operation finish
 ** \retval ErrorTimeout                    Flash internal operation timeout
 **
 ******************************************************************************/
en_result_t SpiFlash_WaitForWriteEnd(void)
{
    en_result_t enRet = Ok;
    uint8_t u8Status = 0u;
    uint32_t u32Timeout;
    stc_clk_freq_t stcClkFreq;

    CLK_GetClockFreq(&stcClkFreq);
    u32Timeout = stcClkFreq.sysclkFreq / 1000u;
    SPI_NSS_LOW();
    SpiFlash_WriteReadByte(FLASH_INSTR_READ_SR1);
    do
    {
        u8Status = SpiFlash_WriteReadByte(FLASH_DUMMY_BYTE_VALUE);
        u32Timeout--;
    } while ((u32Timeout != 0ul) &&
             ((u8Status & FLASH_BUSY_BIT_MASK) == FLASH_BUSY_BIT_MASK));

    if (FLASH_BUSY_BIT_MASK == u8Status)
    {
        enRet = ErrorTimeout;
    }
    while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSpiIdle))
    {
    }
    SPI_NSS_HIGH();

    return enRet;
}

/**
 *******************************************************************************
 ** \brief SPI flash page write program function
 **
 ** \param [in] u32Addr                     Valid flash address
 **
 ** \param [in] pData                       Pointer to send data buffer
 **
 ** \param [in] len                         Send data length
 **
 ** \retval Error                           Page write program failed
 ** \retval Ok                              Page write program success
 **
 ******************************************************************************/
en_result_t SpiFlash_WritePage(uint32_t u32Addr, const uint8_t pData[], uint16_t len)
{
    en_result_t enRet = Ok;
    uint16_t u16Index = 0u;

    if ((u32Addr > FLASH_MAX_ADDR) || (NULL == pData) || (len > FLASH_PAGE_SIZE))
    {
        enRet = Error;
    }
    else
    {
        SpiFlash_WriteEnable();
        /* Send data to flash */
        SPI_NSS_LOW();
        SpiFlash_WriteReadByte(FLASH_INSTR_PAGE_PROGRAM);
        SpiFlash_WriteReadByte((uint8_t)((u32Addr & 0xFF0000ul) >> 16u));
        SpiFlash_WriteReadByte((uint8_t)((u32Addr & 0xFF00u) >> 8u));
        SpiFlash_WriteReadByte((uint8_t)(u32Addr & 0xFFu));
        while (len--)
        {
            SpiFlash_WriteReadByte(pData[u16Index]);
            u16Index++;
        }
        while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSpiIdle))
        {
        }
        SPI_NSS_HIGH();
        /* Wait for flash idle */
        enRet = SpiFlash_WaitForWriteEnd();
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief SPI flash read data function
 **
 ** \param [in] u32Addr                     Valid flash address
 **
 ** \param [out] pData                      Pointer to receive data buffer
 **
 ** \param [in] len                         Read data length
 **
 ** \retval Error                           Read data program failed
 ** \retval Ok                              Read data program success
 **
 ******************************************************************************/
en_result_t SpiFlash_ReadData(uint32_t u32Addr, uint8_t pData[], uint16_t len)
{
    en_result_t enRet = Ok;
    uint16_t u16Index = 0u;

    if ((u32Addr > FLASH_MAX_ADDR) || (NULL == pData))
    {
        enRet = Error;
    }
    else
    {
        SpiFlash_WriteEnable();
        /* Send data to flash */
        SPI_NSS_LOW();
        SpiFlash_WriteReadByte(FLASH_INSTR_STANDARD_READ);
        SpiFlash_WriteReadByte((uint8_t)((u32Addr & 0xFF0000ul) >> 16u));
        SpiFlash_WriteReadByte((uint8_t)((u32Addr & 0xFF00u) >> 8u));
        SpiFlash_WriteReadByte((uint8_t)(u32Addr & 0xFFu));
        while (len--)
        {
            pData[u16Index] = SpiFlash_WriteReadByte(FLASH_DUMMY_BYTE_VALUE);
            u16Index++;
        }
        while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSpiIdle))
        {
        }
        SPI_NSS_HIGH();
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief SPI flash erase 4Kb sector function
 **
 ** \param [in] u32Addr                     Valid flash address
 **
 ** \retval Error                           Sector erase failed
 ** \retval Ok                              Sector erase success
 **
 ******************************************************************************/
en_result_t SpiFlash_Erase4KbSector(uint32_t u32Addr)
{
    en_result_t enRet = Ok;

    if (u32Addr >= FLASH_MAX_ADDR)
    {
        enRet =  Error;
    }
    else
    {
        SpiFlash_WriteEnable();
        /* Send instruction to flash */
        SPI_NSS_LOW();
        SpiFlash_WriteReadByte(FLASH_INSTR_ERASE_4KB_SECTOR);
        SpiFlash_WriteReadByte((uint8_t)((u32Addr & 0xFF0000ul) >> 16u));
        SpiFlash_WriteReadByte((uint8_t)((u32Addr & 0xFF00u) >> 8u));
        SpiFlash_WriteReadByte((uint8_t)(u32Addr & 0xFFu));
        while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSpiIdle))
        {
        }
        SPI_NSS_HIGH();
        /* Wait for flash idle */
        enRet = SpiFlash_WaitForWriteEnd();
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief  main function for SPI write and read flash function
 **
 ** \param [in]  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t flashAddr = 0u;
    uint16_t bufferLen = 0u;
    char txBuffer[] = "SPI read and write flash example: Welcome to use HDSC micro chip";
    char rxBuffer[128];
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    /* Flash NSS */
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(SPI_NSS_PORT, SPI_NSS_PIN, &stcPortInit);
    SPI_NSS_HIGH();
    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Configure SPI */
    Spi_Config();
    /* Get tx buffer length */
    bufferLen = (uint16_t)sizeof(txBuffer);

    while (1)
    {
        if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
        {
            BSP_LED_Off(LED_RED);
            BSP_LED_Off(LED_GREEN);
            memset(rxBuffer, 0l, sizeof(rxBuffer));
            /* Erase sector */
            SpiFlash_Erase4KbSector(flashAddr);
            /* Write data to flash */
            SpiFlash_WritePage(flashAddr, (uint8_t*)&txBuffer[0], bufferLen);
            /* Read data from flash */
            SpiFlash_ReadData(flashAddr, (uint8_t*)&rxBuffer[0], bufferLen);
            /* Compare txBuffer and rxBuffer */
            if (memcmp(txBuffer, rxBuffer, (uint32_t)bufferLen) != 0)
            {
                BSP_LED_On(LED_RED);
            }
            else
            {
                BSP_LED_On(LED_GREEN);
            }
            /* Flash address offset */
            flashAddr += FLASH_SECTOR_SIZE;
            if (flashAddr >= FLASH_MAX_ADDR)
            {
                flashAddr = 0u;
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
