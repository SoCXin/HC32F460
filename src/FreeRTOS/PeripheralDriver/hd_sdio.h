#ifndef _HD_SDIO_H_
#define _HD_SDIO_H_
#include "hc32_ddl.h"
#include "stdbool.h"

//初始化存储芯片 SD NAND 硬件资源，文件系统初始化在此接口内部完成，其它地方不再对文件系统进行初始化，注意关机的需要umount文件系统
void hd_sdio_hw_init(void);

//通过开关机制，降低SDIO不使用时的空闲功耗
void enable_sdio_hw(bool enable);

//读写SD block
en_result_t SD_ReadDisk(uint8_t* buf, uint32_t blk_addr, uint16_t blk_len);
en_result_t SD_WriteDisk(uint8_t* buf, uint32_t blk_addr, uint16_t blk_len);
#endif
