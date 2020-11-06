#ifndef BSP_EFM_H
#define BSP_EFM_H
#include "hc32_ddl.h"
#define SECTORSIZE  8192 
#define SECTORSIZE_W	2048
typedef union EEFLASH{
    uint32_t U32_Data[SECTORSIZE_W];
    char U8_Data[SECTORSIZE];
}EEFLASH_data_t;

en_result_t FlashWritePage(uint32_t u32Addr, EEFLASH_data_t *eeflashdata,uint32_t len);
en_result_t FlashReadPage(uint32_t u32Addr, EEFLASH_data_t *eeflashdata,uint32_t len);
en_result_t FlashWritePageRB(uint32_t u32Addr, EEFLASH_data_t *eeflashdata,uint32_t len);
void TestEFM(void);
#endif
