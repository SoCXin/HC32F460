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
 **   - 2021-03-29  Linsq First version for USB demo.
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

#define MSC_HID_COMPOSITE     1
/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern usb_host_user_callback_func USR_cb;
extern uint8_t                     USBH_USR_ApplicationState ;
extern USBH_HOST                   usb_app_host;
extern void user_keyboard_init(void);
extern void user_keyboard_dataprocess(uint8_t data);

#ifdef MSC_HID_COMPOSITE
extern uint8_t msc_flag;
#endif //MSC_HID_COMPOSITE
/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/


#endif /*__USB_HOST_USER_H__*/

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


