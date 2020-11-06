#ifndef USER_QSPI_H
#define USER_QSPI_H

/* FLASH parameters definition */
#define FLASH_PAGE_SIZE                 (0x100u)
#define FLASH_SRCTOR_SIZE               (0x1000u)
#define FALSH_MAX_ADDR                  (0x800000u)
#define FLASH_DUMMY_BYTE_VALUE          (0xffu)
#define FLASH_BUSY_BIT_MASK             (0x01u)

/* FLASH instruction definition */
#define FLASH_INSTR_WRITE_ENABEL        (0x06u)
#define FLASH_INSTR_PAGE_PROGRAM        (0x02u)
#define FLASH_INSTR_ERASE_4KB_SECTOR    (0x20u)
#define FLASH_INSTR_ERASE_CHIP          (0xC7u)
#define FLASH_INSTR_READ_SR1            (0x05u)
#define FLASH_INSTR_READ_SR2            (0x35u)
#define FLASH_INSTR_READ_SR3            (0x15u)

/* QSPI memory bus address definition */
#define QSPI_BUS_ADDRESS                (0x98000000U)

void User_QSPI_Flash_Init(void);
void QspiFlash_WriteEnable(void);
en_result_t QspiFlash_WaitForWriteEnd(void);
en_result_t QspiFlash_WritePage(uint32_t u32Addr, uint8_t *pData, uint16_t len);
en_result_t QspiFlash_Erase4KbSector(uint32_t u32Addr);
void QspiFlash_EraseChip(void);
uint8_t QspiFlash_ReadStatusRegister(uint8_t u8Reg);
void Test_QSPI(void);

#endif





