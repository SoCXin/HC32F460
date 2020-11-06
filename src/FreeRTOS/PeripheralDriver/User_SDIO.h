#ifndef USER_SDIO_H
#define USER_SDIO_H
#include "hc32_ddl.h"
#include "sd_card.h"

en_result_t SD_CARD_Init(void);
void SD_CARD_TEST(void);
en_result_t FS_SD_ReadBlocks(const uint8_t* buff, uint16_t BlockAddr, uint8_t BlockCnt);
en_result_t FS_SD_WriteBlocks(const uint8_t* buff, uint16_t BlockAddr, uint8_t BlockCnt);

#endif

