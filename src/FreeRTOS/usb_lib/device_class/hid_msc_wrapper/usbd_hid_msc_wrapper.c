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
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
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
/** \file usbd_hid_cdc_wrapper.c
 **
 ** A detailed description is available at
 ** @link
        This file provides the MSC_HID composite functions.
  @endlink

 **   - 2019-11-19  1.0  zhangxl First version for USB MSC_HID composite demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usbd_hid_custom_core.h"
#include "usbd_msc_core.h"
#include "usbd_hid_msc_wrapper.h"
#include "usbd_desc.h"
#include "usbd_req.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
/*********************************************
   MSC Device library callbacks
*********************************************/
extern uint8_t  USBD_MSC_Init (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_MSC_DeInit (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_MSC_Setup (void  *pdev, USB_SETUP_REQ *req);
extern uint8_t  USBD_MSC_DataIn (void  *pdev, uint8_t epnum);
extern uint8_t  USBD_MSC_DataOut (void  *pdev,  uint8_t epnum);
extern uint8_t  *USBD_MSC_GetCfgDesc (uint8_t speed, uint16_t *length);
extern uint8_t  USBD_MSC_CfgDesc[USB_MSC_CONFIG_DESC_SIZ];

/*********************************************
   HID Device library callbacks
*********************************************/
extern uint8_t  USBD_CUSTOM_HID_Init (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_CUSTOM_HID_DeInit (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_CUSTOM_HID_Setup (void  *pdev, USB_SETUP_REQ *req);
extern uint8_t  *USBD_CUSTOM_HID_GetCfgDesc (uint8_t speed, uint16_t *length);
extern uint8_t  USBD_CUSTOM_HID_DataIn (void  *pdev, uint8_t epnum);
extern uint8_t  USBD_CUSTOM_HID_DataOut (void  *pdev, uint8_t epnum);

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static uint8_t USBD_MSC_HID_Init (void *pdev, uint8_t cfgidx);

static uint8_t USBD_MSC_HID_DeInit (void *pdev, uint8_t cfgidx);

static uint8_t USBD_MSC_HID_Setup (void *pdev, USB_SETUP_REQ *req);

static uint8_t *USBD_MSC_HID_GetCfgDesc (uint8_t speed, uint16_t *length);

static uint8_t USBD_MSC_HID_DataIn (void *pdev, uint8_t epnum);
static uint8_t USBD_MSC_HID_DataOut(void *pdev , uint8_t epnum);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

USBD_Class_cb_TypeDef  USBD_MSC_HID_cb =
{
    &USBD_MSC_HID_Init,
    &USBD_MSC_HID_DeInit,
    &USBD_MSC_HID_Setup,
    NULL, /*EP0_TxSent*/
    NULL, /*EP0_RxReady*/
    &USBD_MSC_HID_DataIn, /*DataIn*/
    &USBD_MSC_HID_DataOut, /*DataOut*/
    NULL, /*SOF */
    NULL,
    NULL,
    &USBD_MSC_HID_GetCfgDesc,
#ifdef USB_OTG_HS_CORE
    &USBD_MSC_HID_GetCfgDesc, /* use same config as per FS */
#endif
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB HID device Configuration Descriptor */
__USB_ALIGN_BEGIN static uint8_t USBD_MSC_HID_CfgDesc[USB_MSC_HID_CONFIG_DESC_SIZ] __USB_ALIGN_END =
{
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    USB_MSC_HID_CONFIG_DESC_SIZ,
    /* wTotalLength: Bytes returned */
    0x00,
    0x02,         /*bNumInterfaces: 2 interface*/
    0x01,         /*bConfigurationValue: Configuration value*/
    0x00,         /*iConfiguration: Index of string descriptor describing
    the configuration*/
    0xE0,         /*bmAttributes: bus powered and Support Remote Wake-up */
    0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

    /************** Descriptor of HID interface ****************/
    /* 09 */
    0x09,         /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    HID_INTERFACE,         /*bInterfaceNumber: Number of Interface*/
    0x00,         /*bAlternateSetting: Alternate setting*/
    0x02,         /*bNumEndpoints*/
    0x03,         /*bInterfaceClass: HID*/
    0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0,            /*iInterface: Index of string descriptor*/
    /******************** Descriptor of HID ********************/
    /* 18 */
    0x09,         /*bLength: HID Descriptor size*/
    CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x11,         /*bcdHID: HID Class Spec release number*/
    0x01,
    0x00,         /*bCountryCode: Hardware target country*/
    0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,         /*bDescriptorType*/
    CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of HID endpoint ********************/
    /* 27 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    HID_IN_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    HID_IN_PACKET, /*wMaxPacketSize: 4 Byte max */
    0x00,
    0x0A,          /*bInterval: Polling Interval (10 ms)*/
    /* 34 */
    0x07,          /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

    HID_OUT_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,          /*bmAttributes: Interrupt endpoint*/
    HID_OUT_PACKET, /*wMaxPacketSize: 4 Byte max */
    0x00,
    0x0A,          /*bInterval: Polling Interval (10 ms)*/
    /* 41 */

    /********************  Mass Storage interface ********************/
    0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: */
    MSC_INTERFACE,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints*/
    0x08,   /* bInterfaceClass: MSC Class */
    0x06,   /* bInterfaceSubClass : SCSI transparent*/
    0x50,   /* nInterfaceProtocol */
    0x05,          /* iInterface: */
    /********************  Mass Storage Endpoints ********************/
    0x07,   /*Endpoint descriptor length = 7*/
    0x05,   /*Endpoint descriptor type */
    MSC_IN_EP,   /*Endpoint address (IN, address 1) */
    0x02,   /*Bulk endpoint type */
    LOBYTE(MSC_MAX_PACKET),
    HIBYTE(MSC_MAX_PACKET),
    0x00,   /*Polling interval in milliseconds */

    0x07,   /*Endpoint descriptor length = 7 */
    0x05,   /*Endpoint descriptor type */
    MSC_OUT_EP,   /*Endpoint address (OUT, address 1) */
    0x02,   /*Bulk endpoint type */
    LOBYTE(MSC_MAX_PACKET),
    HIBYTE(MSC_MAX_PACKET),
    0x00     /*Polling interval in milliseconds*/
} ;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
  * @brief  USBD_MSC_HID_Init
  *         Initialize the MSC-HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MSC_HID_Init (void  *pdev, uint8_t cfgidx)
{
    /* HID initialization */
    USBD_CUSTOM_HID_Init (pdev,cfgidx);

    /* MSC initialization */
    USBD_MSC_Init (pdev,cfgidx);

    return USBD_OK;
}

/**
  * @brief  USBD_MSC_HID_DeInit
  *         DeInitialize the MSC_HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MSC_HID_DeInit (void  *pdev, uint8_t cfgidx)
{
    /* HID De-initialization */
    USBD_CUSTOM_HID_DeInit (pdev,cfgidx);

    /* MSC De-initialization */
    USBD_MSC_DeInit (pdev,cfgidx);

    return USBD_OK;
}

/**
  * @brief  USBD_MSC_HID_Setup
  *         Handle the MSC_HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_MSC_HID_Setup (void  *pdev, USB_SETUP_REQ *req)
{
    uint8_t u8Res = USBD_OK;
    switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
    {
        case USB_REQ_RECIPIENT_INTERFACE:
            if (req->wIndex == HID_INTERFACE)
            {
              u8Res = USBD_CUSTOM_HID_Setup (pdev, req);
            }
            else
            {
              u8Res = USBD_MSC_Setup(pdev, req);
            }
            break;

        case USB_REQ_RECIPIENT_ENDPOINT:
            if (req->wIndex == HID_IN_EP)
            {
              u8Res = USBD_CUSTOM_HID_Setup (pdev, req);
            }
            else
            {
              u8Res = USBD_MSC_Setup(pdev, req);
            }
            break;
        default:
            break;
    }
    return u8Res;
}

/**
  * @brief  USBD_MSC_HID_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_MSC_HID_GetCfgDesc (uint8_t speed, uint16_t *length)
{
    *length = (uint16_t)sizeof (USBD_MSC_HID_CfgDesc);
    return USBD_MSC_HID_CfgDesc;
}

/**
  * @brief  USBD_MSC_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MSC_HID_DataIn (void  *pdev, uint8_t epnum)
{
    uint8_t u8Ret = 0u;
    /*DataIN can be for MSC or HID */
    if (epnum == (uint8_t)(MSC_IN_EP&((uint8_t)~0x80u)))
    {
        u8Ret = USBD_MSC_DataIn(pdev, epnum);
    }
    else
    {
        u8Ret = USBD_CUSTOM_HID_DataIn(pdev, epnum);
    }
    return u8Ret;
}

/**
  * @brief  USBD_MSC_HID_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MSC_HID_DataOut(void *pdev , uint8_t epnum)
{
    uint8_t u8Ret = 0u;
    /*DataOut can be for MSC*/
    if (epnum == (uint8_t)(MSC_OUT_EP&((uint8_t)~0x80u)))
    {
        u8Ret = USBD_MSC_DataOut(pdev, epnum);
    }
    else
    {
        /*DataOut can be for HID */
        u8Ret = USBD_CUSTOM_HID_DataOut(pdev, epnum);
    }
    return u8Ret;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
