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
 ** A detailed description is available at
 ** @link
        This file provides all the Application firmware functions.
    @endlink
 **
 **   - 2019-11-19  1.0  CDT First version.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_dev_msc_class.h"
#include "usb_dev_cdc_msc_wrapper.h"
#include "usb_dev_user.h"
#include "usb_Dev_desc.h"
#include "usb_app_conf.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
usb_core_instance usb_dev;

int main(void)
{
    hd_usb_dev_init(&usb_dev, &user_desc, &usb_dev_composite_cbk, &user_cb);
    while (1)
    {

    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
