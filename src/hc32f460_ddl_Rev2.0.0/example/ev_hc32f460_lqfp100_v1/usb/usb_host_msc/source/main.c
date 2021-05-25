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
 **   - 2021-03-29  Linsq First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "usb_host_user.h"
#include "usb_host_msc_class.h"



extern void hd_usb_prtsusp(usb_core_instance *pdev);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
#ifdef USB_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_INTERNAL_DMA_ENABLED */
__USB_ALIGN_BEGIN usb_core_instance      usb_app_instance __USB_ALIGN_END;

#ifdef USB_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_INTERNAL_DMA_ENABLED */
__USB_ALIGN_BEGIN USBH_HOST                usb_app_host __USB_ALIGN_END;

/**
 *******************************************************************************
 ** \brief  main function for mouse function
 **
 ** \param [in]  None
 **
 ** \return int32_t Return value, if needed
 **
 ******************************************************************************/
int main (void)
{
    usb_host_init(&usb_app_instance, &usb_app_host, &USBH_MSC_cb, &USR_cb);
    while (1)
    {
        usb_host_mainprocess(&usb_app_instance, &usb_app_host);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
