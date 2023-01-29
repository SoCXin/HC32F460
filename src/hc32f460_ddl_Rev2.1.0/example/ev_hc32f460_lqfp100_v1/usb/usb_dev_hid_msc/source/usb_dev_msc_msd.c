/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_dev_msc_msd.c
 **
 ** A detailed description is available at
 ** @link
        This file includes the user MSC application layer.
    @endlink
 **
 **   - 2019-05-15  1.0  CDT First version for USB MSC device demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "usb_dev_msc_msd.h"
#include "usb_dev_msc_mem.h"
#include "usb_app_conf.h"
#include "w25qxx.h"

int8_t msc_init(uint8_t lun);
int8_t msc_getcapacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size);
int8_t msc_ifready(uint8_t lun);
int8_t msc_ifwrprotected(uint8_t lun);
int8_t msc_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t msc_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t msc_getmaxlun(void);

USB_DEV_MSC_cbk_TypeDef flash_fops =
{
    &msc_init,
    &msc_getcapacity,
    &msc_getmaxlun,
    &msc_ifready,
    &msc_read,
    &msc_write,    
    &msc_ifwrprotected,    
    (int8_t *)msc_inquirydata
};

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
/* Variable for Storage operation status */
volatile uint8_t USB_STATUS_REG = 0u;

/* USB Mass storage querty data (36 bytes for each lun) */
const int8_t msc_inquirydata[] =
{
    /* LUN 0 */
    0x00,
    0x80,
    0x02,
    0x02,
    (USB_DEV_INQUIRY_LENGTH - 4u),
    0x00,
    0x00,
    0x00,
    /* Vendor Identification */
    'H', 'D', 'S', 'C', ' ', 'M', 'C', 'U', ' ',    //9 bytes
    /* Product Identification */
    'S', 'P', 'I', ' ', 'F', 'l', 'a', 's', 'h',    //15 bytes
    ' ', 'D', 'i', 's', 'k', ' ',
    /* Product Revision Level */
    '1', '.', '0', ' ',                             //4 bytes
    /* LUN 1 */
    0x00,
    0x80,
    0x02,
    0x02,
    (USB_DEV_INQUIRY_LENGTH - 4u),
    0x00,
    0x00,
    0x00,
    /* Vendor Identification */
    'H', 'D', 'S', 'C', ' ', 'M', 'C', 'U', ' ',    //9 bytes
    /* Product Identification */
    'S', 'D', ' ', 'F', 'l', 'a', 's', 'h', ' ',    //15 bytes
    'D', 'i', 's', 'k', ' ', ' ',
    /* Product Revision Level */
    '1', '.', '0', ' ',                             //4 bytes
};
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/* Pointer to flash_fops */
USB_DEV_MSC_cbk_TypeDef *msc_fops = &flash_fops;

/**
 *******************************************************************************
 ** \brief  initialize storage
 ** \param  lun: logic number
 ** \retval status
 ******************************************************************************/
int8_t msc_init(uint8_t lun)
{
    W25QXX_Init();
    return Ok;
}

/**
 *******************************************************************************
 ** \brief  Get Storage capacity
 ** \param  lun: logic number
 ** \param  block_num: sector number
 ** \param  block_size: sector size
 ** \retval status
 ******************************************************************************/
int8_t msc_getcapacity(uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    *block_size = 512u;
    *block_num  = 1024u * 1024u * 8u / 512u;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief  Check if storage is ready
 ** \param  lun: logic number
 ** \retval status
 ******************************************************************************/
int8_t  msc_ifready(uint8_t lun)
{
    USB_STATUS_REG |= (uint8_t)0X10;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief  Check if storage is write protected
 ** \param  lun: logic number
 ** \retval status
 ******************************************************************************/
int8_t msc_ifwrprotected(uint8_t lun)
{
    return Ok;
}

/**
 *******************************************************************************
 ** \brief  read data from storage devices
 ** \param  lun: logic number
 ** \param  buf: data buffer be read
 ** \param  blk_addr: sector address
 ** \param  blk_len: sector count
 ** \retval status
 ******************************************************************************/
int8_t msc_read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    int8_t res = (int8_t)0;
    USB_STATUS_REG |= (uint8_t)0X02;
    W25QXX_Read(buf, blk_addr * 512u, blk_len * 512u);
    return res;
}

/**
 *******************************************************************************
 ** \brief  Write data to storage devices
 ** \param  lun: logic number
 ** \param  buf: data buffer be written
 ** \param  blk_addr: sector address
 ** \param  blk_len: sector count
 ** \retval status
 ******************************************************************************/
int8_t msc_write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
    int8_t res = (int8_t)0;
    USB_STATUS_REG |= (uint8_t)0X01;
    W25QXX_Write(buf, blk_addr * 512u, blk_len * 512u);
    return res;
}

/**
 *******************************************************************************
 ** \brief  Get supported logic number
 ** \param  none
 ** \retval 1
 ******************************************************************************/
int8_t msc_getmaxlun(void)
{
    return 1;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
