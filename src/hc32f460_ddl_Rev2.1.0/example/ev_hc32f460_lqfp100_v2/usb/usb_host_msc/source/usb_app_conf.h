/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file
 **
 ** A detailed description is available at
 ** @link General low level driver configuration @endlink
 **
 **   - 2021-04-16  CDT First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USB_CORE_CONF_H__
#define __USB_CORE_CONF_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/

/* USB Core and PHY interface configuration.
   Tip: To avoid modifying these defines each time you need to change the USB
        configuration, you can declare the needed define in your toolchain
        compiler preprocessor.
   */
/****************** USB OTG FS PHY CONFIGURATION *******************************
*  The USB OTG FS Core supports one on-chip Full Speed PHY.
*
*  The USE_EMBEDDED_PHY symbol is defined in the project compiler preprocessor
*  when FS core is used.
*******************************************************************************/
#ifndef USE_USB_FS
 #define USE_USB_FS
#endif /* USE_USB_FS */

#ifdef USE_USB_FS
 #define USB_FS_CORE
#endif

/****************** USB OTG HS PHY CONFIGURATION *******************************
*  The USB OTG HS Core supports two PHY interfaces:
*   (i)  An ULPI interface for the external High Speed PHY: the USB HS Core will
*        operate in High speed mode
*   (ii) An on-chip Full Speed PHY: the USB HS Core will operate in Full speed mode
*
*  You can select the PHY to be used using one of these two defines:
*   (i)  USE_ULPI_PHY: if the USB OTG HS Core is to be used in High speed mode
*   (ii) USE_EMBEDDED_PHY: if the USB OTG HS Core is to be used in Full speed mode
*
*  Notes:
*   - The USE_ULPI_PHY symbol is defined in the project compiler preprocessor as
*     default PHY when HS core is used.
*******************************************************************************/
#ifndef USE_ULPI_PHY
 //#define USE_ULPI_PHY
#endif /* USE_ULPI_PHY */

#ifndef USE_EMBEDDED_PHY
 #define USE_EMBEDDED_PHY
#endif /* USE_EMBEDDED_PHY */


/*******************************************************************************
*                      FIFO Size Configuration in Device mode
*
*  (i) Receive data FIFO size = RAM for setup packets +
*                   OUT endpoint control information +
*                   data OUT packets + miscellaneous
*      Space = ONE 32-bits words
*     --> RAM for setup packets = 10 spaces
*        (n is the nbr of CTRL EPs the device core supports)
*     --> OUT EP CTRL info      = 1 space
*        (one space for status information written to the FIFO along with each
*        received packet)
*     --> data OUT packets      = (Largest Packet Size / 4) + 1 spaces
*        (MINIMUM to receive packets)
*     --> OR data OUT packets  = at least 2*(Largest Packet Size / 4) + 1 spaces
*        (if high-bandwidth EP is enabled or multiple isochronous EPs)
*     --> miscellaneous = 1 space per OUT EP
*        (one space for transfer complete status information also pushed to the
*        FIFO with each endpoint's last packet)
*
*  (ii)MINIMUM RAM space required for each IN EP Tx FIFO = MAX packet size for
*       that particular IN EP. More space allocated in the IN EP Tx FIFO results
*       in a better performance on the USB and can hide latencies on the AHB.
*
*  (iii) TXn min size = 16 words. (n  : Transmit FIFO index)
*   (iv) When a TxFIFO is not used, the Configuration should be as follows:
*       case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txm can use the space allocated for Txn.
*       case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
*       --> Txn should be configured with the minimum space of 16 words
*  (v) The FIFO is used optimally when used TxFIFOs are allocated in the top
*       of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
*   (vi) In HS case12 FIFO locations should be reserved for internal DMA registers
*        so total FIFO size should be 1012 Only instead of 1024
*******************************************************************************/

/****************** USB OTG FS CONFIGURATION **********************************/
#ifdef USB_FS_CORE
 #define RX_FIFO_FS_SIZE                          128u
 #define TX0_FIFO_FS_SIZE                          64u
 #define TX1_FIFO_FS_SIZE                         128u
 #define TX2_FIFO_FS_SIZE                           0u
 #define TX3_FIFO_FS_SIZE                           0u
 #define TX4_FIFO_FS_SIZE                           0u
 #define TX5_FIFO_FS_SIZE                           0u
 #define USB_INTERNAL_DMA_ENABLED
#endif

/****************** USB OTG MODE CONFIGURATION ********************************/
#define USE_HOST_MODE
#define USE_DEVICE_MODE

#ifndef USE_DEVICE_MODE
 #ifndef USE_HOST_MODE
    #error  "USE_DEVICE_MODE or USE_HOST_MODE should be defined"
 #endif
#endif

#define RX_FIFO_HS_SIZE                          512u
#define TXH_NP_HS_FIFOSIZ                        128u
#define TXH_P_HS_FIFOSIZ                         256u

#define RX_FIFO_FS_SIZE                           128u
#define TXH_NP_FS_FIFOSIZ                         32u
#define TXH_P_FS_FIFOSIZ                          64u

/****************** C Compilers dependant keywords ****************************/

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__CC_ARM)         /* ARM Compiler */
  #define __packed    __packed
#elif defined (__ICCARM__)     /* IAR Compiler */
//  #define __packed    __packed
#elif defined   ( __GNUC__ )   /* GNU Compiler */
//  #define __packed    __attribute__ ((__packed__))
#elif defined   (__TASKING__)  /* TASKING Compiler */
  #define __packed    __unaligned
#endif /* __CC_ARM */


//#define USB_MSC_FAT_VALID 1


#endif //__USB_CORE_CONF_H__

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
