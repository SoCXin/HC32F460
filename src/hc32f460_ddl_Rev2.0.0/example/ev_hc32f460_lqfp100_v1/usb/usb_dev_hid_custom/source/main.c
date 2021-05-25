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
 ** \brief USB custom hid example.
 **
 **   - 2021-04-16  CDT   First version for USB custom hid device demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/

#include "hc32_ddl.h"
#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_bsp.h"
#include "usb_dev_custom_hid_class.h"

usb_core_instance  usb_dev;
uint8_t flag = 0;

int32_t main (void)
{
    hd_usb_dev_init(&usb_dev, &USR_desc, &class_customhid_cbk, &user_cb);
    while (1)
    {
        if(flag == 1)
        {
            if(usb_dev.dev.device_cur_status == USB_DEV_CONFIGURED)
            {
                flag = 0;
                hd_usb_deveptx(&usb_dev, HID_IN_EP, NULL, 0);
            }
        }
    }
}

void SysTick_IrqHandler(void)
{
    flag = 1;
}
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

