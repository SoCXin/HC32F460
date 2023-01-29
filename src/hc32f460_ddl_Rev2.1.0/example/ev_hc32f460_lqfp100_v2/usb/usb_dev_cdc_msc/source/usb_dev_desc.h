/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_dev_desc.h
 **
 ** A detailed description is available at
 ** @link header file for the usbd_desc.c @endlink
 **
 **   - 2021-04-26  1.0  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_DEV_DESC_H__
#define __USB_DEV_DESC_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_dev_def.h"
#include "stdint.h"
/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CFG_DESCRIPTOR_TYPE                 0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_SIZ_DEVICE_DESC                     18
#define USB_SIZ_STRING_LANGID                   4

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern  usb_dev_desc_func user_desc;

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/

#endif

