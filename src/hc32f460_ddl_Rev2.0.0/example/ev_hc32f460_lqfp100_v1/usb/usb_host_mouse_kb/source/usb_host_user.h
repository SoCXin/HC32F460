/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usbh_usr.h
 **
 ** A detailed description is available at
 ** @link Header file for usbh_usr.c @endlink
 **
 **   - 2018-12-26  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_HOST_USER_H__
#define __USB_HOST_USER_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_host_core.h"
#include "usb_app_conf.h"
#include "usb_host_user_print.h"
#include <stdio.h>

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern  usb_host_user_callback_func USR_cb;
extern  uint8_t USBH_USR_ApplicationState ;
/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
void USBH_USR_ApplicationSelected(void);
void host_user_init(void);
void host_user_denint(void);
void host_user_devattached(void);
void host_user_devreset(void);
void host_user_devdisconn (void);
void host_user_overcurrent (void);
void host_user_devspddetected(uint8_t DeviceSpeed);
void host_user_devdescavailable(void *DeviceDesc);
void host_user_devaddrdistributed(void);
void host_user_cfgdescavailable(usb_host_cfgdesc_typedef * cfgDesc,
                                          usb_host_itfdesc_typedef *itfDesc,
                                          USB_HOST_EPDesc_TypeDef *epDesc);
void host_user_mfcstring(void *ManufacturerString);
void host_user_productstring(void *ProductString);
void host_user_serialnum(void *SerialNumString);
void host_user_enumcompl(void);
HOST_USER_STATUS host_user_userinput(void);
void host_user_devunsupported(void);
void host_user_unrecoverederror(void);


extern void  user_keyboard_init(void);
extern void  user_keyboard_dataprocess(uint8_t pbuf);
#endif /*__USB_HOST_USER_H__*/

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
