/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_host_user_print.h
 **
 ** A detailed description is available at
 ** @link Header file for usb_host_user_print.c @endlink
 **
 **   - 2019-12-13  CDT First version for USB hid mouse & keyboard demo.
 **
 ******************************************************************************/
#ifndef __USB_HOST_USER_PRINT_H__
#define __USB_HOST_USER_PRINT_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_host_hid_class.h"

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
extern void Mouse_PositionUpdate(int8_t x ,int8_t y);
extern void Mouse_ButtonRelease(uint8_t button_idx);
extern void Mouse_ButtonPress (uint8_t button_idx);

#endif /* __USB_HOST_USER_PRINT_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
