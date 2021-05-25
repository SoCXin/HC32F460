/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_host_user.c
 **
 ** A detailed description is available at
 ** @link
        This file includes the user application layer.
    @endlink
 **
 **   - 2019-12-10  CDT First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_host_user.h"
#include "usb_host_hid_mouseapp.h"
#include "usb_host_hid_keyboardapp.h"
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */

usb_host_user_callback_func USR_cb =
{
    host_user_init,
    host_user_denint,
    host_user_devattached,
    host_user_devreset,
    host_user_devdisconn,
    host_user_overcurrent,
    host_user_devspddetected,
    host_user_devdescavailable,
    host_user_devaddrdistributed,
    host_user_cfgdescavailable,
    host_user_mfcstring,
    host_user_productstring,
    host_user_serialnum,
    host_user_enumcompl,
    host_user_userinput,
    NULL,
    host_user_devunsupported,
    host_user_unrecoverederror
};

/*--------------- Messages ---------------*/
const char* MSG_HOST_INIT          = "> Host Library Initialized\n";
const char* MSG_DEV_ATTACHED       = "> Device Attached\n";
const char* MSG_DEV_DISCONNECTED   = "> Device Disconnected\n";
const char* MSG_DEV_ENUMERATED     = "> Enumeration completed\n";
const char* MSG_DEV_HIGHSPEED      = "> High speed device detected\n";
const char* MSG_DEV_FULLSPEED      = "> Full speed device detected\n";
const char* MSG_DEV_LOWSPEED       = "> Low speed device detected\n";
const char* MSG_DEV_ERROR          = "> Device fault \n";

const char* MSG_MSC_CLASS          = "> Mass storage device connected\n";
const char* MSG_HID_CLASS          = "> HID device connected\n";

const char* USB_HID_MouseStatus    = "> HID Demo Device : Mouse\n";
const char* USB_HID_KeybrdStatus   = "> HID Demo Device : Keyboard\n";
const char* MSG_UNREC_ERROR        = "> UNRECOVERED ERROR STATE\n";

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Displays the message on debugging terminal for host lib initialization
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_init(void)
{
    static uint8_t startup = 0;
    if(startup == 0 )
    {
        startup = 1;
        DDL_Printf("> USB Host library started.\n");
    }
}

/**
 *******************************************************************************
 ** \brief  Displays the message on debugging terminal as device attached
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devattached(void)
{
    DDL_Printf((void*)MSG_DEV_ATTACHED);
}

/**
 *******************************************************************************
 ** \brief
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_unrecoverederror (void)
{
    DDL_Printf((void*)MSG_UNREC_ERROR);
}

/**
 *******************************************************************************
 ** \brief  device disconnect event
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devdisconn (void)
{
    DDL_Printf((void *)MSG_DEV_DISCONNECTED);
}

/**
 *******************************************************************************
 ** \brief  device reset
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devreset(void)
{
  /* Users can do their application actions here for the USB-Reset */
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for device speed
 ** \param  DeviceSpeed: device speed
 ** \retval none
 ******************************************************************************/
void host_user_devspddetected(uint8_t DeviceSpeed)
{
    if(DeviceSpeed == PRTSPD_FULL_SPEED)
    {
        DDL_Printf((void *)MSG_DEV_FULLSPEED);
    }
    else if(DeviceSpeed == PRTSPD_LOW_SPEED)
    {
        DDL_Printf((void *)MSG_DEV_LOWSPEED);
    }
    else
    {
        DDL_Printf((void *)MSG_DEV_ERROR);
    }
}

/**
 *******************************************************************************
 ** \brief  Displays the message on terminal for device descriptor
 ** \param  DeviceDesc : device descriptor
 ** \retval none
 ******************************************************************************/
void host_user_devdescavailable(void *DeviceDesc)
{
    uint8_t temp[50];
    usb_host_devdesc_typedef *hs;
    hs = DeviceDesc;
    DDL_Printf((char *)temp , "VID : %04lXh\n" , (uint32_t)(*hs).idVendor);
    DDL_Printf((void *)temp);
    DDL_Printf((char *)temp , "PID : %04lXh\n" , (uint32_t)(*hs).idProduct);
    DDL_Printf((void *)temp);
}

/**
 *******************************************************************************
 ** \brief  USB device is successfully distributed a Address
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devaddrdistributed(void)
{

}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for configuration descriptor
 ** \param  cfgDesc: configuration descriptor
 ** \param  itfDesc: interface descriptor
 ** \param  epDesc:  EP descriptor
 ** \retval none
 ******************************************************************************/
void host_user_cfgdescavailable(usb_host_cfgdesc_typedef * cfgDesc,
                                          usb_host_itfdesc_typedef *itfDesc,
                                          USB_HOST_EPDesc_TypeDef *epDesc)
{
    usb_host_itfdesc_typedef *id;

    id = itfDesc;

    if((*id).bInterfaceClass  == 0x08)
    {
        DDL_Printf("%s", MSG_MSC_CLASS);
    }
    else if((*id).bInterfaceClass  == 0x03)
    {
        DDL_Printf("%s", MSG_HID_CLASS);
    }
}

/**
* @brief  host_user_mfcstring
*         Displays the message on LCD for Manufacturer String
* @param  ManufacturerString : Manufacturer String of Device
* @retval None
*/
/**
 *******************************************************************************
 ** \brief  Displays the Manufacturer String
 ** \param  ManufacturerString: MFC string
 ** \retval none
 ******************************************************************************/
void host_user_mfcstring(void *ManufacturerString)
{
    char temp[100];
    DDL_Printf(temp, "Manufacturer : %s\n", (char *)ManufacturerString);
    DDL_Printf((void *)temp);
}

/**
 *******************************************************************************
 ** \brief  display product string
 ** \param  ProductString: product string
 ** \retval none
 ******************************************************************************/
void host_user_productstring(void *ProductString)
{
    char temp[100];
    DDL_Printf((char *)temp, "Product : %s\n", (char *)ProductString);
    DDL_Printf((void *)temp);
}

/**
 *******************************************************************************
 ** \brief  display the serial number string
 ** \param  SerialNumString: serial number string pointer
 ** \retval none
 ******************************************************************************/
void host_user_serialnum(void *SerialNumString)
{
    uint8_t temp[100];
    DDL_Printf((char *)temp, "Serial Number : %s\n", (char *)SerialNumString);
    DDL_Printf((void *)temp);
}

/**
 *******************************************************************************
 ** \brief  display enumeration complete
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_enumcompl(void)
{
    /* Enumeration complete */
    DDL_Printf((void *)MSG_DEV_ENUMERATED);
}

/**
 *******************************************************************************
 ** \brief  display that host does not support the device connected
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devunsupported(void)
{
    DDL_Printf("> Device not supported.\n");
}

/**
 *******************************************************************************
 ** \brief  user input
 ** \param  none
 ** \retval none
 ******************************************************************************/
HOST_USER_STATUS host_user_userinput(void)
{

    HOST_USER_STATUS usbh_usr_status;

    usbh_usr_status = USER_NONE_RESP;

    /*Key is in polling mode to detect user action */
    if(BSP_KEY_GetStatus(BSP_KEY_2) == Reset)
    {
        usbh_usr_status = USER_HAVE_RESP;
    }
    return usbh_usr_status;
}

/**
 *******************************************************************************
 ** \brief  display that have detected device over current
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_overcurrent (void)
{
    DDL_Printf("Overcurrent detected.\n");
}

/**
 *******************************************************************************
 ** \brief  initialize mouse window
 ** \param  none
 ** \retval none
 ******************************************************************************/
void USR_MOUSE_Init	(void)
{
    Mouse_ButtonRelease(0);
    Mouse_ButtonRelease(1);
    Mouse_ButtonRelease(2);
}

/**
 *******************************************************************************
 ** \brief  process the mouse data
 ** \param  data: mouse data would be decode
 ** \retval none
 ******************************************************************************/
void USR_MOUSE_ProcessData(HID_MOUSE_Data_TypeDef *data)
{

    uint8_t idx = 1u;
    static uint8_t b_state[3u] = { 0u, 0u , 0u};

    if ((data->x != 0u) && (data->y != 0u))
    {
        Mouse_PositionUpdate(data->x , data->y);
    }

    for ( idx = 0u ; idx < 3u ; idx ++)
    {

        if(data->button & (0x01u << idx))
        {
            if(b_state[idx] == 0u)
            {
                Mouse_ButtonPress(idx);
                b_state[idx] = 1u;
            }
        }
        else
        {
            if(b_state[idx] == 1u)
            {
                Mouse_ButtonRelease(idx);
                b_state[idx] = 0u;
            }
        }
    }
}

/**
 *******************************************************************************
 ** \brief  initialize the key board
 ** \param  none
 ** \retval none
 ******************************************************************************/
void  user_keyboard_init(void)
{
    DDL_Printf((void*)USB_HID_KeybrdStatus);
    DDL_Printf("> Use Keyboard to tape characters: \n\n");
    DDL_Printf("\n\n\n\n\n\n");
}

/**
 *******************************************************************************
 ** \brief  display key value
 ** \param  data: key value
 ** \retval none
 ******************************************************************************/
void  user_keyboard_dataprocess(uint8_t data)
{
    DDL_Printf("%c", (char)data);
}

/**
 *******************************************************************************
 ** \brief  deint user state and other related state if needed
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_denint(void)
{
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
