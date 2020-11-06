/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file usbh_stdreq.h
 **
 ** A detailed description is available at
 ** @link Header file for usbh_stdreq.c @endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USBH_STDREQ_H
#define __USBH_STDREQ_H

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_conf.h"
#include "usb_hcd.h"
#include "usbh_core.h"
#include "usbh_def.h"

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief USBH_STDREQ_Exported_Defines
 **
 ******************************************************************************/
/*Standard Feature Selector for clear feature command*/
#define FEATURE_SELECTOR_ENDPOINT         0X00
#define FEATURE_SELECTOR_DEVICE           0X01


#define INTERFACE_DESC_TYPE               0x04
#define ENDPOINT_DESC_TYPE                0x05
#define INTERFACE_DESC_SIZE               0x09


#define USBH_HID_CLASS                    0x03

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern uint8_t USBH_CfgDesc[512];

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
USBH_Status USBH_GetDescriptor(USB_OTG_CORE_HANDLE *pdev,
                               USBH_HOST           *phost,
                               uint8_t  req_type,
                               uint16_t value_idx,
                               uint8_t* buff,
                               uint16_t length );

USBH_Status USBH_Get_DevDesc(USB_OTG_CORE_HANDLE *pdev,
                             USBH_HOST *phost,
                             uint8_t length);

USBH_Status USBH_Get_StringDesc(USB_OTG_CORE_HANDLE *pdev,
                                USBH_HOST           *phost,
                                uint8_t string_index,
                                uint8_t *buff,
                                uint16_t length);

USBH_Status USBH_SetCfg(USB_OTG_CORE_HANDLE *pdev,
                        USBH_HOST *phost,
                        uint16_t configuration_value);

USBH_Status USBH_Get_CfgDesc(USB_OTG_CORE_HANDLE *pdev,
                             USBH_HOST           *phost,
                             uint16_t length);

USBH_Status USBH_SetAddress(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST           *phost,
                            uint8_t DeviceAddress);

USBH_Status USBH_ClrFeature(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST           *phost,
                            uint8_t ep_num, uint8_t hc_num);

USBH_Status USBH_SetInterface(USB_OTG_CORE_HANDLE *pdev,
                        USBH_HOST *phost,
                        uint8_t ep_num, uint8_t altSetting);

USBH_Status USBH_Issue_ClrFeature(USB_OTG_CORE_HANDLE *pdev,
                                  USBH_HOST           *phost,
                                  uint8_t ep_num);

USBH_DescHeader_t      *USBH_GetNextDesc (uint8_t   *pbuf,
                                                  uint16_t  *ptr);


#endif /* __USBH_STDREQ_H */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



