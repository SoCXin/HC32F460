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
extern  uint8_t USBH_USR_ApplicationState ;
/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
void USBH_USR_ApplicationSelected(void);
void USBH_USR_Init(void);
void USBH_USR_DeInit(void);
void USBH_USR_DeviceAttached(void);
void USBH_USR_ResetDevice(void);
void USBH_USR_DeviceDisconnected (void);
void USBH_USR_OverCurrentDetected (void);
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed);
void USBH_USR_Device_DescAvailable(void *DeviceDesc);
void USBH_USR_DeviceAddressAssigned(void);
void USBH_USR_Configuration_DescAvailable(usb_host_cfgdesc_typedef * cfgDesc,
                                          usb_host_itfdesc_typedef *itfDesc,
                                          USB_HOST_EPDesc_TypeDef *epDesc);
void USBH_USR_Manufacturer_String(void *ManufacturerString);
void USBH_USR_Product_String(void *ProductString);
void USBH_USR_SerialNum_String(void *SerialNumString);
void USBH_USR_EnumerationDone(void);
HOST_USER_STATUS USBH_USR_UserInput(void);
void USBH_USR_DeviceNotSupported(void);
void USBH_USR_UnrecoveredError(void);
int USBH_USR_MSC_Application(void);

#endif /*__USB_HOST_USER_H__*/

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


