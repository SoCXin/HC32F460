/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief USB composite device(HID+CDC) example.
 **
 **   - 2021-04-14  Linsq First version for USB composite device demo.
 **
 ******************************************************************************/

#include "hc32_ddl.h"
#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_bsp.h"
#include "usb_dev_hid_cdc_wrapper.h"

usb_core_instance  usb_dev;

int32_t main (void)
{
    hd_usb_dev_init(&usb_dev, &user_desc, &class_composite_cbk, &user_cb);
    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

