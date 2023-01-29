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
 **   - 2018-12-26  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_BSP__H__
#define __USB_BSP__H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_core_driver.h"
#include "usb_app_conf.h"

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern usb_core_instance usb_app_instance;
/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
void hd_usb_bsp_init(usb_core_instance *pdev);
void hd_usb_udelay(const uint32_t usec);
void hd_usb_mdelay(const uint32_t msec);
void usb_bsp_nvic(void);
#ifdef USE_HOST_MODE
void hd_usb_bsp_cfgvbus(usb_core_instance *pdev);
void usb_bsp_drivevbus(usb_core_instance *pdev,uint8_t state);
#endif

#endif //__USB_BSP__H__

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
