/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_bsp.h
 **
 ** A detailed description is available at
 ** @link Specific api's relative to the used hardware platform @endlink
 **
 **   - 2018-12-26  1.0  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_BSP__H__
#define __USB_BSP__H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_core_driver.h"
#include "usb_app_conf.h"
#include "hc32_ddl.h"

#define KEY_PORT    (PortD)
#define KEY_UP      (Pin03)
#define KEY_LEFT    (Pin04)
#define KEY_DOWN    (Pin05)
#define KEY_RIGHT   (Pin06)

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
void BSP_Init(void);
void hd_usb_bsp_init (usb_core_instance *pdev);
void hd_usb_udelay(const uint32_t usec);
void hd_usb_mdelay(const uint32_t msec);
void hd_usb_bsp_nvicconfig (void);

#endif //__USB_BSP__H__

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
