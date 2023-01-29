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
 **   - 2021-04-16  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_HOST_USER_H__
#define __USB_HOST_USER_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_host_core.h"
#include "usb_app_conf.h"
#include <stdio.h>
#include "usb_host_msc_class.h"

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* State Machine for the USBH_USR_ApplicationState */
#define USH_USR_FS_INIT       0u
#define USH_USR_FS_READLIST   1u
#define USH_USR_FS_WRITEFILE  2u
#define USH_USR_FS_IDLE       3u
/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern  usb_host_user_callback_func USR_cb;
extern uint8_t USB_HOST_USER_AppState ;
/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
extern void host_user_init(void);
extern void host_user_denint(void);
extern void host_user_devattached(void);
extern void host_user_devreset(void);
extern void host_user_devdisconn (void);
extern void host_user_overcurrent (void);
extern void host_user_devspddetected(uint8_t DeviceSpeed);
extern void host_user_devdescavailable(void *DeviceDesc);
extern void host_user_devaddrdistributed(void);
extern void host_user_cfgdescavailable(usb_host_cfgdesc_typedef * cfgDesc,
                                       usb_host_itfdesc_typedef *itfDesc,
                                       USB_HOST_EPDesc_TypeDef *epDesc);
extern void host_user_mfcstring(void *ManufacturerString);
extern void host_user_productstring(void *ProductString);
extern void host_user_serialnum(void *SerialNumString);
extern void host_user_enumcompl(void);
extern HOST_USER_STATUS host_user_userinput(void);
extern void host_user_devunsupported(void);
extern void host_user_unrecoverederror(void);
extern int host_user_msc_app(void);

#endif /*__USB_HOST_USER_H__*/

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


