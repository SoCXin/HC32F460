/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_app_conf.h
 **
 ** A detailed description is available at
 ** @link General low level driver configuration @endlink
 **
 **   - 2021-04-13  1.0  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_CONF__H__
#define __USB_CONF__H__

/*******************************************************************************
 * Include files
 ******************************************************************************/

/************** USB DEVICE ENDPOINT CONFIGURATION *****************************/
#define MSC_IN_EP                         0x81u
#define MSC_OUT_EP                        0x01u

#define USBD_ITF_MAX_NUM     1
/****************** USB FS CONFIGURATION **********************************/
#define RX_FIFO_FS_SIZE       128u
#define TX0_FIFO_FS_SIZE      64u
#define TX1_FIFO_FS_SIZE      64u
#define TX2_FIFO_FS_SIZE      64u
#define TX3_FIFO_FS_SIZE      0u
#define TX4_FIFO_FS_SIZE      0u
#define TX5_FIFO_FS_SIZE      0u

#define USE_DEVICE_MODE

//
#define MSC_MEDIA_PACKET      (12ul * 1024ul)
#define MSC_MAX_PACKET        (64u)


/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__CC_ARM)         /* ARM Compiler */
  #define __packed    __packed
#elif defined (__ICCARM__)     /* IAR Compiler */
//  #define __packed    __packed  /* MISRAC2004 19.4*/
#elif defined   ( __GNUC__ )   /* GNU Compiler */
//  #define __packed    __attribute__ ((__packed__))
#elif defined   (__TASKING__)  /* TASKING Compiler */
  #define __packed    __unaligned
#endif /* __CC_ARM */

#endif //__USB_CONF__H__

