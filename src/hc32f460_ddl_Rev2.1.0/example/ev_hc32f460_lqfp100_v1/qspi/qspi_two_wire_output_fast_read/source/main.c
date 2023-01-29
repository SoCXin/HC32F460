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
 ** \brief The example of QSPI two wire output fast read function
 **
 **   - 2018-11-05  CDT  First version for Device Driver Library of QSPI.
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
/* QSPCK Port/Pin definition */
#define QSPCK_PORT                      (PortC)
#define QSPCK_PIN                       (Pin06)

/* QSNSS Port/Pin definition */
#define QSNSS_PORT                      (PortC)
#define QSNSS_PIN                       (Pin07)

/* QSIO0 Port/Pin definition */
#define QSIO0_PORT                      (PortD)
#define QSIO0_PIN                       (Pin08)

/* QSIO1 Port/Pin definition */
#define QSIO1_PORT                      (PortD)
#define QSIO1_PIN                       (Pin09)

/* QSPI memory bus address definition */
#define QSPI_BUS_ADDRESS                (0x98000000ul)

/* FLASH parameters definition */
#define FLASH_PAGE_SIZE                 (0x100u)
#define FLASH_SECTOR_SIZE               (0x1000u)
#define FLASH_MAX_ADDR                  (0x800000ul)
#define FLASH_DUMMY_BYTE_VALUE          (0xffu)
#define FLASH_BUSY_BIT_MASK             (0x01u)

/* FLASH instruction definition */
#define FLASH_INSTR_WRITE_ENABLE        (0x06u)
#define FLASH_INSTR_PAGE_PROGRAM        (0x02u)
#define FLASH_INSTR_ERASE_4KB_SECTOR    (0x20u)
#define FLASH_INSTR_ERASE_CHIP          (0xC7u)
#define FLASH_INSTR_READ_SR1            (0x05u)
#define FLASH_INSTR_READ_SR2            (0x35u)
#define FLASH_INSTR_READ_SR3            (0x15u)

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
 ** \brief QSPI flash init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void QspiFlash_Init(void)
{
    stc_qspi_init_t stcQspiInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcQspiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_QSPI, Enable);

    /* Configuration QSPI pin */
    PORT_SetFunc(QSPCK_PORT, QSPCK_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSNSS_PORT, QSNSS_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO0_PORT, QSIO0_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO1_PORT, QSIO1_PIN, Func_Qspi, Disable);

    /* Configuration QSPI structure */
    stcQspiInit.enClkDiv = QspiHclkDiv2;
    stcQspiInit.enSpiMode = QspiSpiMode3;
    stcQspiInit.enBusCommMode = QspiBusModeRomAccess;
    stcQspiInit.enPrefetchMode = QspiPrefetchStopComplete;
    stcQspiInit.enPrefetchFuncEn = Enable;
    stcQspiInit.enQssnValidExtendTime = QspiQssnValidExtendNot;
    stcQspiInit.enQssnIntervalTime = QspiQssnIntervalQsck8;
    stcQspiInit.enQsckDutyCorr = QspiQsckDutyCorrNot;
    stcQspiInit.enVirtualPeriod = QspiVirtualPeriodQsck8;
    stcQspiInit.enWpPinLevel = QspiWpPinOutputHigh;
    stcQspiInit.enQssnSetupDelayTime = QspiQssnSetupDelay1Dot5Qsck;
    stcQspiInit.enQssnHoldDelayTime = QspiQssnHoldDelay1Dot5Qsck;
    stcQspiInit.enFourByteAddrReadEn = Disable;
    stcQspiInit.enAddrWidth = QspiAddressByteThree;
    stcQspiInit.stcCommProtocol.enReadMode = QspiReadModeTwoWiresOutput;
    stcQspiInit.stcCommProtocol.enTransInstrProtocol = QspiProtocolExtendSpi;
    stcQspiInit.stcCommProtocol.enTransAddrProtocol = QspiProtocolExtendSpi;
    stcQspiInit.stcCommProtocol.enReceProtocol = QspiProtocolExtendSpi;
    stcQspiInit.u8RomAccessInstr = QSPI_3BINSTR_TWO_WIRES_OUTPUT_READ;
    QSPI_Init(&stcQspiInit);
}

/**
 *******************************************************************************
 ** \brief QSPI flash write enable function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void QspiFlash_WriteEnable(void)
{
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_WRITE_ENABLE);
    QSPI_ExitDirectCommMode();
}

/**
 *******************************************************************************
 ** \brief QSPI flash wait for write operation end function
 **
 ** \param [in] None
 **
 ** \retval Ok                              Flash internal operation finish
 ** \retval ErrorTimeout                    Flash internal operation timeout
 **
 ******************************************************************************/
en_result_t QspiFlash_WaitForWriteEnd(void)
{
    en_result_t enRet = Ok;
    uint8_t u8Status = 0u;
    uint32_t u32Timeout;
    stc_clk_freq_t stcClkFreq;

    CLK_GetClockFreq(&stcClkFreq);
    u32Timeout = stcClkFreq.sysclkFreq / 1000u;
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_READ_SR1);
    do
    {
        u8Status = QSPI_ReadDirectCommValue();
        u32Timeout--;
    } while ((u32Timeout != 0u) &&
             ((u8Status & FLASH_BUSY_BIT_MASK) == FLASH_BUSY_BIT_MASK));

    if (FLASH_BUSY_BIT_MASK == u8Status)
    {
        enRet = ErrorTimeout;
    }
    QSPI_ExitDirectCommMode();

    return enRet;
}

/**
 *******************************************************************************
 ** \brief QSPI flash page write program function
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
en_result_t QspiFlash_WritePage(uint32_t u32Addr, const uint8_t pData[], uint16_t len)
{
    en_result_t enRet = Ok;
    uint16_t u16Index = 0u;

    if ((u32Addr > FLASH_MAX_ADDR) || (NULL == pData) || (len > FLASH_PAGE_SIZE))
    {
        enRet = Error;
    }
    else
    {
        QspiFlash_WriteEnable();
        /* Send data to flash */
        QSPI_EnterDirectCommMode();
        QSPI_WriteDirectCommValue(FLASH_INSTR_PAGE_PROGRAM);
        QSPI_WriteDirectCommValue((uint8_t)((u32Addr & 0xFF0000ul) >> 16));
        QSPI_WriteDirectCommValue((uint8_t)((u32Addr & 0xFF00u) >> 8));
        QSPI_WriteDirectCommValue((uint8_t)(u32Addr & 0xFFu));
        while (len--)
        {
           QSPI_WriteDirectCommValue(pData[u16Index]);
           u16Index++;
        }
        QSPI_ExitDirectCommMode();
        /* Wait for flash idle */
        enRet = QspiFlash_WaitForWriteEnd();
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief QSPI flash erase 4Kb sector function
 **
 ** \param [in] u32Addr                     Valid flash address
 **
 ** \retval Error                           Sector erase failed
 ** \retval Ok                              Sector erase success
 **
 ******************************************************************************/
en_result_t QspiFlash_Erase4KbSector(uint32_t u32Addr)
{
    en_result_t enRet = Ok;

    if (u32Addr >= FLASH_MAX_ADDR)
    {
        enRet = Error;
    }
    else
    {
        QspiFlash_WriteEnable();
        /* Send instruction to flash */
        QSPI_EnterDirectCommMode();
        QSPI_WriteDirectCommValue(FLASH_INSTR_ERASE_4KB_SECTOR);
        QSPI_WriteDirectCommValue((uint8_t)((u32Addr & 0xFF0000ul) >> 16));
        QSPI_WriteDirectCommValue((uint8_t)((u32Addr & 0xFF00u) >> 8));
        QSPI_WriteDirectCommValue((uint8_t)(u32Addr & 0xFFu));
        QSPI_ExitDirectCommMode();
        /* Wait for flash idle */
        enRet = QspiFlash_WaitForWriteEnd();
    }

    return enRet;
}

/**
 *******************************************************************************
 ** \brief QSPI flash erase chip function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void QspiFlash_EraseChip(void)
{
    QspiFlash_WriteEnable();
    /* Send instruction to flash */
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_ERASE_CHIP);
    QSPI_ExitDirectCommMode();
    /* Wait for flash idle */
    QspiFlash_WaitForWriteEnd();
}

/**
 *******************************************************************************
 ** \brief QSPI flash read status register function
 **
 ** \param [in] u8Reg                       Need to get status register
 ** \arg FLASH_INSTR_READ_SR1               Status register 1
 ** \arg FLASH_INSTR_READ_SR2               Status register 2
 ** \arg FLASH_INSTR_READ_SR3               Status register 3
 **
 ** \retval uint8_t                         Current register value
 **
 ******************************************************************************/
uint8_t QspiFlash_ReadStatusRegister(uint8_t u8Reg)
{
    uint8_t regSta = 0u;

    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(u8Reg);
    regSta = QSPI_ReadDirectCommValue();
    QSPI_ExitDirectCommMode();

    return regSta;
}

/**
 *******************************************************************************
 ** \brief  main function for QSPI two wire output fast read function
 **
 ** \param [in] None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t flashAddr = 0u;
    uint8_t *pFlashReadAddr;
    uint16_t bufferLen = 0u;
    char txBuffer[] = "QSPI read and write flash example: Welcome to use HDSC micro chip";
    stc_qspi_comm_protocol_t stcQspiCommProtocol;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcQspiCommProtocol);

    /* BSP initialization */
    BSP_CLK_Init();
    BSP_LED_Init();
    BSP_KEY_Init();
    /* Flash initialization */
    QspiFlash_Init();
    /* Get tx buffer length */
    bufferLen = (uint16_t)sizeof(txBuffer);

    while (1)
    {
        if (Set == BSP_KEY_GetStatus(BSP_KEY_2))
        {
            BSP_LED_Off(LED_RED);
            BSP_LED_Off(LED_GREEN);
            /* Switch to standard read mode */
            stcQspiCommProtocol.enReadMode = QspiReadModeStandard;
            QSPI_CommProtocolConfig(&stcQspiCommProtocol);
            /* Erase sector */
            QspiFlash_Erase4KbSector(flashAddr);
            /* Write data to flash */
            QspiFlash_WritePage(flashAddr, (uint8_t*)&txBuffer[0], bufferLen);
            /* Switch to two wire output fast read mode */
            stcQspiCommProtocol.enReadMode = QspiReadModeTwoWiresOutput;
            QSPI_CommProtocolConfig(&stcQspiCommProtocol);
            /* Pointer to flash address map */
            pFlashReadAddr = (uint8_t *)((uint32_t)QSPI_BUS_ADDRESS + flashAddr);
            /* Compare txBuffer and flash */
            if (memcmp(txBuffer, pFlashReadAddr, (uint32_t)bufferLen) != 0)
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
