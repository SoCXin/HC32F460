/*****************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
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
/** \file usbh_core.h
 **
 ** A detailed description is available at
 ** @link header file for the usbh_core.c @endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USBH_CORE_H
#define __USBH_CORE_H

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_hcd.h"
#include "usbh_def.h"
#include "usbh_conf.h"

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief USBH_CORE_Exported_Types
 **
 ******************************************************************************/
typedef enum {
    USBH_OK   = 0,
    USBH_BUSY,
    USBH_FAIL,
    USBH_NOT_SUPPORTED,
    USBH_UNRECOVERED_ERROR,
    USBH_ERROR_SPEED_UNKNOWN,
    USBH_APPLY_DEINIT
}USBH_Status;

/* Following states are used for gState */
typedef enum {
    HOST_IDLE =0,
    HOST_DEV_ATTACHED,
    HOST_DEV_DISCONNECTED,
    HOST_DETECT_DEVICE_SPEED,
    HOST_ENUMERATION,
    HOST_CLASS_REQUEST,
    HOST_CLASS,
    HOST_CTRL_XFER,
    HOST_USR_INPUT,
    HOST_SUSPENDED,
    HOST_ERROR_STATE
}HOST_State;

/* Following states are used for EnumerationState */
typedef enum {
    ENUM_IDLE = 0,
    ENUM_GET_FULL_DEV_DESC,
    ENUM_SET_ADDR,
    ENUM_GET_CFG_DESC,
    ENUM_GET_FULL_CFG_DESC,
    ENUM_GET_MFC_STRING_DESC,
    ENUM_GET_PRODUCT_STRING_DESC,
    ENUM_GET_SERIALNUM_STRING_DESC,
    ENUM_SET_CONFIGURATION,
    ENUM_DEV_CONFIGURED
} ENUM_State;

/* Following states are used for CtrlXferStateMachine */
typedef enum {
    CTRL_IDLE =0,
    CTRL_SETUP,
    CTRL_SETUP_WAIT,
    CTRL_DATA_IN,
    CTRL_DATA_IN_WAIT,
    CTRL_DATA_OUT,
    CTRL_DATA_OUT_WAIT,
    CTRL_STATUS_IN,
    CTRL_STATUS_IN_WAIT,
    CTRL_STATUS_OUT,
    CTRL_STATUS_OUT_WAIT,
    CTRL_ERROR,
    CTRL_STALLED,
    CTRL_COMPLETE
}CTRL_State;

typedef enum {
    USBH_USR_NO_RESP   = 0,
    USBH_USR_RESP_OK = 1,
}USBH_USR_Status;

/* Following states are used for RequestState */
typedef enum {
    CMD_IDLE =0,
    CMD_SEND,
    CMD_WAIT
} CMD_State;

typedef struct _Ctrl
{
    uint8_t               hc_num_in;
    uint8_t               hc_num_out;
    uint8_t               ep0size;
    uint8_t               *buff;
    uint16_t              length;
    uint8_t               errorcount;
    uint16_t              timer;
    CTRL_STATUS           status;
    USB_Setup_TypeDef     setup;
    CTRL_State            state;
} USBH_Ctrl_TypeDef;

typedef struct _DeviceProp
{
    uint8_t                           address;
    uint8_t                           speed;
    USBH_DevDesc_TypeDef              Dev_Desc;
    USBH_CfgDesc_TypeDef              Cfg_Desc;
    USBH_InterfaceDesc_TypeDef        Itf_Desc[USBH_MAX_NUM_INTERFACES];
    USBH_EpDesc_TypeDef               Ep_Desc[USBH_MAX_NUM_INTERFACES][USBH_MAX_NUM_ENDPOINTS];
    USBH_HIDDesc_TypeDef              HID_Desc;
}USBH_Device_TypeDef;

typedef struct _USBH_Class_cb
{
    USBH_Status  (*Init)    (USB_OTG_CORE_HANDLE *pdev , void *phost);
    void         (*DeInit)  (USB_OTG_CORE_HANDLE *pdev , void *phost);
    USBH_Status  (*Requests)  (USB_OTG_CORE_HANDLE *pdev , void *phost);
    USBH_Status  (*Machine)   (USB_OTG_CORE_HANDLE *pdev , void *phost);
} USBH_Class_cb_TypeDef;

typedef struct _USBH_USR_PROP
{
    void (*Init)(void);       /* HostLibInitialized */
    void (*DeInit)(void);       /* HostLibInitialized */
    void (*DeviceAttached)(void);           /* DeviceAttached */
    void (*ResetDevice)(void);
    void (*DeviceDisconnected)(void);
    void (*OverCurrentDetected)(void);
    void (*DeviceSpeedDetected)(uint8_t DeviceSpeed);          /* DeviceSpeed */
    void (*DeviceDescAvailable)(void *);    /* DeviceDescriptor is available */
    void (*DeviceAddressAssigned)(void);  /* Address is assigned to USB Device */
    void (*ConfigurationDescAvailable)(USBH_CfgDesc_TypeDef *,
                                     USBH_InterfaceDesc_TypeDef *,
                                     USBH_EpDesc_TypeDef *);
    /* Configuration Descriptor available */
    void (*ManufacturerString)(void *);     /* ManufacturerString*/
    void (*ProductString)(void *);          /* ProductString*/
    void (*SerialNumString)(void *);        /* SerialNubString*/
    void (*EnumerationDone)(void);           /* Enumeration finished */
    USBH_USR_Status (*UserInput)(void);
    int  (*UserApplication) (void);
    void (*DeviceNotSupported)(void); /* Device is not supported*/
    void (*UnrecoveredError)(void);
}USBH_Usr_cb_TypeDef;

typedef struct _Host_TypeDef
{
    HOST_State            gState;       /*  Host State Machine Value */
    HOST_State            gStateBkp;    /* backup of previous State machine value */
    ENUM_State            EnumState;    /* Enumeration state Machine */
    CMD_State             RequestState;
    USBH_Ctrl_TypeDef     Control;

    USBH_Device_TypeDef   device_prop;

    USBH_Class_cb_TypeDef               *class_cb;
    USBH_Usr_cb_TypeDef  	              *usr_cb;
} USBH_HOST, *pUSBH_HOST;
/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief USBH_CORE_Exported_Defines
 **
 ******************************************************************************/
#define MSC_CLASS                         0x08
#define HID_CLASS                         0x03
#define MSC_PROTOCOL                      0x50
#define CBI_PROTOCOL                      0x01

#define USBH_MAX_ERROR_COUNT                            2
#define USBH_DEVICE_ADDRESS_DEFAULT                     0
#define USBH_DEVICE_ADDRESS                             1

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
void USBH_Init(USB_OTG_CORE_HANDLE *pdev,
               USB_OTG_CORE_ID_TypeDef coreID,
               USBH_HOST *phost,
               USBH_Class_cb_TypeDef *class_cb,
               USBH_Usr_cb_TypeDef *usr_cb);
USBH_Status USBH_DeInit(USB_OTG_CORE_HANDLE *pdev,
                        USBH_HOST *phost);
void USBH_Process(USB_OTG_CORE_HANDLE *pdev ,
                  USBH_HOST *phost);
void USBH_ErrorHandle(USBH_HOST *phost,
                      USBH_Status errType);

#endif /* __USBH_CORE_H */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/




