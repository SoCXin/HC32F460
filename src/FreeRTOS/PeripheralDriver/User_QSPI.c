#include "hc32_ddl.h"
#include "User_QSPI.h"

/* QSPCK Port/Pin definition */
#define QSPCK_PORT                      PortC
#define QSPCK_PIN                       Pin06

/* QSNSS Port/Pin definition */
#define QSNSS_PORT                      PortC
#define QSNSS_PIN                       Pin07

/* QSIO0 Port/Pin definition */
#define QSIO0_PORT                      PortD
#define QSIO0_PIN                       Pin08

/* QSIO1 Port/Pin definition */
#define QSIO1_PORT                      PortD
#define QSIO1_PIN                       Pin09

/* QSIO2 Port/Pin definition */
#define QSIO2_PORT                      PortD
#define QSIO2_PIN                       Pin10

/* QSIO3 Port/Pin definition */
#define QSIO3_PORT                      PortD
#define QSIO3_PIN                       Pin11



void User_QSPI_Flash_Init(void)
{
    stc_qspi_init_t stcQspiInit;
    MEM_ZERO_STRUCT(stcQspiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_QSPI, Enable);

    /* Configuration QSPI pin */
    PORT_SetFunc(QSPCK_PORT, QSPCK_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSNSS_PORT, QSNSS_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO0_PORT, QSIO0_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO1_PORT, QSIO1_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO2_PORT, QSIO2_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO3_PORT, QSIO3_PIN, Func_Qspi, Disable);

    /* Configuration QSPI structure */
    stcQspiInit.enClkDiv = QspiHclkDiv4;//时钟分频
    stcQspiInit.enSpiMode = QspiSpiMode0;//SPI模式-模式3
    stcQspiInit.enBusCommMode = QspiBusModeRomAccess;//
    stcQspiInit.enPrefetchMode = QspiPrefetchStopComplete;//
    stcQspiInit.enPrefetchFuncEn = Disable;//预读取功能--无效
    stcQspiInit.enQssnValidExtendTime = QspiQssnValidExtendSck32;//CS延长32个QSCK周期
    stcQspiInit.enQssnIntervalTime = QspiQssnIntervalQsck8;//CS最小无效时间8个QSCK周期
    stcQspiInit.enQsckDutyCorr = QspiQsckDutyCorrHalfHclk;//上升沿滞后半个周期
    stcQspiInit.enVirtualPeriod = QspiVirtualPeriodQsck6;//虚拟周期6个QSCK
    stcQspiInit.enWpPinLevel = QspiWpPinOutputHigh;//写保护电平高电平
    stcQspiInit.enQssnSetupDelayTime = QspiQssnSetupDelay1Dot5Qsck;//提前1.5个周期输出CS
    stcQspiInit.enQssnHoldDelayTime = QspiQssnHoldDelay1Dot5Qsck;//滞后1.5个周期释放CS
    stcQspiInit.enFourByteAddrReadEn = Disable;//不使用4字节地址读指令
    stcQspiInit.enAddrWidth = QspiAddressByteThree;//地址宽度3字节
    stcQspiInit.stcCommProtocol.enReadMode = QspiReadModeFourWiresIO;//读取模式选择4线输入输出快速读
    stcQspiInit.stcCommProtocol.enTransInstrProtocol = QspiProtocolFourWiresSpi;//指令协议模式-扩展式
    stcQspiInit.stcCommProtocol.enTransAddrProtocol = QspiProtocolFourWiresSpi;//地址协议模式--扩展式
    stcQspiInit.stcCommProtocol.enReceProtocol = QspiProtocolFourWiresSpi;//数据接收协议模式--扩展式
    stcQspiInit.u8RomAccessInstr = QSPI_3BINSTR_FOUR_WIRES_IO_READ;//指令代码
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
    QSPI_WriteDirectCommValue(FLASH_INSTR_WRITE_ENABEL);
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
    uint8_t u8Status = 0;
    uint32_t u32Timeout;
    stc_clk_freq_t stcClkFreq;

    CLK_GetClockFreq(&stcClkFreq);
    u32Timeout = stcClkFreq.sysclkFreq / 1000;
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_READ_SR1);
    do
    {
        u8Status = QSPI_ReadDirectCommValue();
        u32Timeout--;
    } while ((u32Timeout != 0) &&
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
en_result_t QspiFlash_WritePage(uint32_t u32Addr, uint8_t *pData, uint16_t len)
{
    en_result_t enRet;

    if ((u32Addr > FALSH_MAX_ADDR) || (NULL == pData) || (len > FLASH_PAGE_SIZE))
    {
        return Error;
    }

    QspiFlash_WriteEnable();
    /* Send data to flash */
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_PAGE_PROGRAM);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF0000) >> 16);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF00) >> 8);
    QSPI_WriteDirectCommValue(u32Addr & 0xFF);
    while (len--)
    {
        QSPI_WriteDirectCommValue(*pData++);
    }
    QSPI_ExitDirectCommMode();
    /* Wait for flash idle */
    enRet = QspiFlash_WaitForWriteEnd();

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
    en_result_t enRet;

    if (u32Addr >= FALSH_MAX_ADDR)
    {
        return Error;
    }

    QspiFlash_WriteEnable();
    /* Send instruction to flash */
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_ERASE_4KB_SECTOR);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF0000) >> 16);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF00) >> 8);
    QSPI_WriteDirectCommValue(u32Addr & 0xFF);
    QSPI_ExitDirectCommMode();
    /* Wait for flash idle */
    enRet = QspiFlash_WaitForWriteEnd();

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
    uint8_t regSta = 0;

    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(u8Reg);
    regSta = QSPI_ReadDirectCommValue();
    QSPI_ExitDirectCommMode();

    return regSta;
}
uint8_t QspiFlash_WriteStatusRegister(uint8_t u8Reg,uint8_t value)
{
    uint8_t regSta = 0;

    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(u8Reg);
    QSPI_WriteDirectCommValue(value);
    QSPI_ExitDirectCommMode();

    return regSta;
}
void Test_QSPI(void)
{
    uint32_t flashAddr = 0x0;
    uint8_t *pFlashReadAddr, bufferLen = 0;//,reg_status;
    static uint8_t txBuffer[] = "QSPI read and write flash example: Welcome to use HDSC micro chip";
    static uint8_t rxBuffer[sizeof(txBuffer)];
    bufferLen = sizeof(txBuffer);
    stc_qspi_comm_protocol_t stcQspiCommProtocol;
//    stcQspiCommProtocol.enReadMode = QspiReadModeStandard;
//    QSPI_CommProtocolConfig(&stcQspiCommProtocol);
//    
//    reg_status = QspiFlash_ReadStatusRegister(FLASH_INSTR_READ_SR2);
//    if((reg_status & 0x02) !=0)
//    {
//      printf("Flash QSPI MODE Enabled\r\n");
//    }
//    else
//    {
//      QspiFlash_WriteStatusRegister(FLASH_INSTR_READ_SR2,0x02);
//    }
    stcQspiCommProtocol.enReadMode = QspiReadModeFourWiresIO;
    QSPI_CommProtocolConfig(&stcQspiCommProtocol);
    /* Erase sector */
    QspiFlash_Erase4KbSector(flashAddr);
    /* Write data to flash */
    QspiFlash_WritePage(flashAddr, &txBuffer[0], bufferLen);
    /* Switch to four wire i/o fast read mode */
    stcQspiCommProtocol.enReadMode = QspiReadModeFourWiresIO;
    QSPI_CommProtocolConfig(&stcQspiCommProtocol);
    /* Pointer to flash address map */
    pFlashReadAddr = (uint8_t *)((uint32_t)QSPI_BUS_ADDRESS + flashAddr);
    /* Compare txBuffer and flash */
    memcpy(rxBuffer, pFlashReadAddr,bufferLen);
    if (memcmp(txBuffer, rxBuffer, bufferLen) == 0)
    {
        printf("QSPI test Pass\r\n");
    }
    else
    {
        printf("QSPI test failed!\r\n");
    }
    
}
/*end of file*/
