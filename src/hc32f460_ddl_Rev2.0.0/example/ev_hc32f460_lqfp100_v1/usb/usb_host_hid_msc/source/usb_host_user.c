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
 **   - 2021-03-29  Linsq First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "usb_host_user.h"
#ifdef USB_MSC_FAT_VALID
#include "ff.h"
#endif
#include "usb_host_msc_class.h"
#include "usb_host_msc_scsi.h"
#include "usb_host_msc_bot.h"
#include "hc32_ddl.h"
#include "usb_host_hid_class.h"
#include "usb_host_hid_keyboardapp.h"
#include "usb_host_hid_mouseapp.h"
#include "usb_host_user_print.h"
/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* USBH_USR_Private_Macros */
extern usb_core_instance          usb_app_instance;

#ifdef MSC_HID_COMPOSITE
uint8_t msc_flag = 0;
#endif //MSC_HID_COMPOSITE

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;

#ifdef USB_MSC_FAT_VALID
FATFS fatfs;
FIL file;
#endif

uint8_t line_idx = 0u;

void host_user_init(void);
void host_user_denint(void);
void host_user_devattached(void);
void host_user_devreset(void);
void host_user_devdisconn (void);
void host_user_overcurrent (void);
void host_user_devspddetected(uint8_t devspeed);
void host_user_devdescavailable(void *DeviceDesc);
void host_user_devaddrdistributed(void);
void host_user_cfgdescavailable(usb_host_cfgdesc_typedef * cfgDesc,
                                usb_host_itfdesc_typedef *itfDesc,
                                USB_HOST_EPDesc_TypeDef *epDesc);
void host_user_mfcstring(void *ManufacturerString);
void host_user_productstring(void *ProductString);
void host_user_serialnum(void *SerialNumString);
void host_user_enumcompl(void);
HOST_USER_STATUS host_user_userinput(void);
void host_user_devunsupported(void);
void host_user_unrecoverederror(void);
int host_user_msc_app(void);


usb_host_user_callback_func USR_cb =
{
    &host_user_init,
    &host_user_denint,
    &host_user_devattached,
    &host_user_devreset,
    &host_user_devdisconn,
    &host_user_overcurrent,
    &host_user_devspddetected,
    &host_user_devdescavailable,
    &host_user_devaddrdistributed,
    &host_user_cfgdescavailable,
    &host_user_mfcstring,
    &host_user_productstring,
    &host_user_serialnum,
    &host_user_enumcompl,
    &host_user_userinput,
    &host_user_msc_app,
    &host_user_devunsupported,
    &host_user_unrecoverederror

};

/* USBH_USR_Private_Constants */
const char* MSG_ROOT_CONT        = "> Exploring disk flash ...\n";
const char* MSG_WR_PROTECT       = "> The disk is write protected\n";

const char* USB_HID_KeybrdStatus   = "> HID Demo Device : Keyboard\n";
#ifdef USB_MSC_FAT_VALID
static uint8_t Explore_Disk (char* path , uint8_t recu_level);
static void     Toggle_Leds(void);
#endif

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  sends the message through the uart and displays on the PC
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_init(void)
{
    static uint8_t startup = 0u;

    if(startup == 0u )
    {
        startup = 1u;
        DDL_Printf(">>--USB FS MSC Host\n");
        DDL_Printf(">>--USB host library started.\n");
        DDL_Printf(">>--USB host library v2.1.0\n" );
    }
}

/**
 *******************************************************************************
 ** \brief  sends message through uart and display on the PC when device connects.
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devattached(void)
{
    DDL_Printf(">>--A USB device has connected\r\n");
}

/**
 *******************************************************************************
 ** \brief  host_user_unrecoverederror
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_unrecoverederror (void)
{
    DDL_Printf(">>--UNRECOVERED ERROR STATE\r\n");
}

/**
 *******************************************************************************
 ** \brief  diaplays that the device has disconnected.
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devdisconn (void)
{
    DDL_Printf(">>--The device has disconnected\r\n");
}

/**
 *******************************************************************************
 ** \brief  displays when the device resets.
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devreset(void)
{
    DDL_Printf(">>--The USB device resets\r\n");
}

/**
 *******************************************************************************
 ** \brief  displays the speed message of the USB core of the device.
 ** \param  DeviceSpeed : speed of the USB core.
 ** \retval None
 ******************************************************************************/
void host_user_devspddetected(uint8_t devspeed)
{
    if(devspeed == PRTSPD_FULL_SPEED)
    {
        DDL_Printf(">>--The USB device is a FS device\r\n");
    }
    else if(devspeed == PRTSPD_LOW_SPEED)
    {
        DDL_Printf(">>--The USB device is a LS device\r\n");
    }
    else
    {
        DDL_Printf(">>--Device fault \r\n");
    }
}

/**
 *******************************************************************************
 ** \brief  host_user_devdescavailable
 ** \param  DeviceDesc : device descriptor
 ** \retval none
 ******************************************************************************/
void host_user_devdescavailable(void *DeviceDesc)
{
    usb_host_devdesc_typedef *hs;
    hs = DeviceDesc;

    DDL_Printf(">>--VID : %04lXh\r\n" , (uint32_t)(*hs).idVendor);
    DDL_Printf(">>--PID : %04lXh\r\n" , (uint32_t)(*hs).idProduct);
}

/**
 *******************************************************************************
 ** \brief  host_user_devaddrdistributed
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devaddrdistributed(void)
{
    DDL_Printf(">>--The address of the attaching USB device has been distributed\r\n");
}

/**
 *******************************************************************************
 ** \brief  host_user_cfgdescavailable
 ** \param  cfgDesc : Configuration desctriptor
 ** \param  itfDesc : Interface desctriptor
 ** \param  epDesc : Endpoint desctriptor
 ** \retval none
 ******************************************************************************/
void host_user_cfgdescavailable(usb_host_cfgdesc_typedef * cfgDesc,
                                          usb_host_itfdesc_typedef *itfDesc,
                                          USB_HOST_EPDesc_TypeDef *epDesc)
{
    usb_host_itfdesc_typedef *id;

    id = itfDesc;

    if((*id).bInterfaceClass  == 0x08u)
    {
        DDL_Printf(">>--The attached device is a MSC device\r\n");
        usb_app_host.class_callbk = &USBH_MSC_cb;
#ifdef MSC_HID_COMPOSITE
        msc_flag = 1;
#endif //MSC_HID_COMPOSITE
    }
    else if((*id).bInterfaceClass  == 0x03u)
    {
        DDL_Printf(">>--The attached device is a HID device\r\n");
        usb_app_host.class_callbk = &USBH_HID_cb;
    }
    else
    {
        //
    }
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for Manufacturer String
 ** \param  ManufacturerString
 ** \retval none
 ******************************************************************************/
void host_user_mfcstring(void *ManufacturerString)
{
    DDL_Printf(">>--The manufacturer is: %s\n", (char *)ManufacturerString);
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for product String
 ** \param  ProductString
 ** \retval none
 ******************************************************************************/
void host_user_productstring(void *ProductString)
{
    DDL_Printf(">>--The product is: %s\n", (char *)ProductString);
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for SerialNum_String
 ** \param  SerialNumString
 ** \retval none
 ******************************************************************************/
void host_user_serialnum(void *SerialNumString)
{
    DDL_Printf( ">>--The serial number is: %s\n", (char *)SerialNumString);
}

/**
 *******************************************************************************
 ** \brief  User response request is displayed to ask application jump to class
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_enumcompl(void)
{

    /* Enumeration complete */
    DDL_Printf(">>--Enumeration has completed \r\n");
    if(usb_app_host.class_callbk == &USBH_MSC_cb)
    {
        DDL_Printf(">>--To see the root content of the disk : \r\n" );
        DDL_Printf(">>--Press USER KEY...\r\n");
    }
}

/**
 *******************************************************************************
 ** \brief  Device is not supported
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_devunsupported(void)
{
    DDL_Printf (">>--The host doesn't support such a device\r\n");
}

/**
 *******************************************************************************
 ** \brief  User Action for application state entry
 ** \param  none
 ** \retval status defined by HOST_USER_STATUS
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
 ** \brief  Over Current Detected on VBUS
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_overcurrent (void)
{
    DDL_Printf (">>--Overcurrent detected.\r\n");
}

/**
 *******************************************************************************
 ** \brief  Demo application for mass storage
 ** \param  none
 ** \retval zero
 ******************************************************************************/
int host_user_msc_app(void)
{
#ifdef USB_MSC_FAT_VALID
    FRESULT res;
    uint8_t writeTextBuff[] = "HDSC Connectivity line Host Demo application using FAT_FS   ";
    uint16_t bytesWritten, bytesToWrite;
    uint32_t u32DevConnectTmp = 0ul;

    switch(USBH_USR_ApplicationState)
    {
        case USH_USR_FS_INIT:
            if ( f_mount(&fatfs, "", 0u) != FR_OK )//register the work area of the volume
            {
                DDL_Printf("> Cannot initialize File System.\n");
                return(-1);
            }
            DDL_Printf("> File System initialized.\n");
            DDL_Printf("> Disk capacity : %ld Bytes\n", USB_HOST_MSC_Param.MSC_Capacity * \
              USB_HOST_MSC_Param.MSC_PageLength);
            DDL_Printf("> Disk capacity : %ld * %d = %ld Bytes\n",
                USB_HOST_MSC_Param.MSC_Capacity,
                USB_HOST_MSC_Param.MSC_PageLength,
                USB_HOST_MSC_Param.MSC_Capacity * \
            USB_HOST_MSC_Param.MSC_PageLength);
            if(USB_HOST_MSC_Param.MSC_WriteProtect == DISK_WRITE_PROTECTED)
            {
                DDL_Printf((void *)MSG_WR_PROTECT);
            }

            USBH_USR_ApplicationState = USH_USR_FS_READLIST;
            break;

        case USH_USR_FS_READLIST:
            DDL_Printf((void *)MSG_ROOT_CONT);
            Explore_Disk("0:/", 1u);
            line_idx = 0u;
            USBH_USR_ApplicationState = USH_USR_FS_WRITEFILE;
            break;

        case USH_USR_FS_WRITEFILE:
            DDL_Printf("Press USER KEY to write file\n");
            hd_usb_mdelay(100ul);

            /*Key in polling*/
            u32DevConnectTmp = host_driver_ifdevconnected(&usb_app_instance);
            while((BSP_KEY_GetStatus(BSP_KEY_2) == Reset) && u32DevConnectTmp)
            {
                Toggle_Leds();
            }
            /* Writes a text file, HC32.TXT in the disk*/
            DDL_Printf("> Writing File to disk flash ...\n");
            if(USB_HOST_MSC_Param.MSC_WriteProtect == DISK_WRITE_PROTECTED)
            {
                DDL_Printf ( "> Disk flash is write protected \n");
                USBH_USR_ApplicationState = USH_USR_FS_IDLE;
                break;
            }

            /* Register work area for logical drives */
            f_mount(&fatfs, "", 0u);

            if(f_open(&file, "0:HC32.TXT",FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
            {
                /* Write buffer to file */
                bytesToWrite = (uint16_t)sizeof(writeTextBuff);
                res= f_write (&file, writeTextBuff, (UINT)bytesToWrite, (void *)&bytesWritten);

                if((bytesWritten == 0u) || (res != FR_OK)) /*EOF or Error*/
                {
                    DDL_Printf("> HC32.TXT CANNOT be writen.\n");
                }
                else
                {
                    DDL_Printf("> 'HC32.TXT' file created\n");
                }

                /*close file and filesystem*/
                f_close(&file);
                f_mount(NULL, "", 0u);
            }
            else
            {
                DDL_Printf ("> HC32.TXT created in the disk\n");
            }

            USBH_USR_ApplicationState = USH_USR_FS_IDLE;

            break;

        case USH_USR_FS_IDLE:
            break;

        default: break;
    }
#endif
    return((int)0);
}
#ifdef USB_MSC_FAT_VALID
/**
 *******************************************************************************
 ** \brief  Displays disk content
 ** \param  path: pointer to root path
 ** \param  recu_level
 ** \retval uint8_t
 ******************************************************************************/
static uint8_t Explore_Disk (char* path , uint8_t recu_level)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    char *fn;
    char tmp[14];
    uint32_t u32DevConnectTmp = 0ul;

    res = f_opendir(&dir, path);
    if (res == FR_OK)
    {
        while(host_driver_ifdevconnected(&usb_app_instance))
        {
            res = f_readdir(&dir, &fno);
            if ((res != FR_OK )|| (fno.fname[0] == 0))
            {
                break;
            }
            if (fno.fname[0] == '.')
            {
                continue;
            }

            fn = fno.fname;
            strcpy(tmp, fn);

            line_idx++;
            if(line_idx > 9u)
            {
                line_idx = 0u;
                DDL_Printf(">>--Press USER KEY to continue...\r\n");

                /*Key in polling*/
                u32DevConnectTmp = host_driver_ifdevconnected(&usb_app_instance);
                while((BSP_KEY_GetStatus(BSP_KEY_2) == Reset) && u32DevConnectTmp)
                {
                    Toggle_Leds();
                }
            }

            if(recu_level == 1u)
            {
                DDL_Printf("   |__");
            }
            //else if(recu_level == 2u)
            else
            {
                DDL_Printf("   |   |__");
            }
            if((fno.fattrib & AM_DIR) == AM_DIR)
            {
                strcat(tmp, "\n");
                DDL_Printf((void *)tmp);
            }
            else
            {
                strcat(tmp, "\n");
                DDL_Printf((void *)tmp);
            }

            if(((fno.fattrib & AM_DIR) == AM_DIR)&&(recu_level == 1u))
            {
                Explore_Disk(fn, 2u);
            }
        }
    }
    return res;
}

/**
 *******************************************************************************
 ** \brief  Toggle leds to shows user input state
 ** \param  none
 ** \retval none
 ******************************************************************************/
static void Toggle_Leds(void)
{
    static uint32_t i;
    if (i++ == 0x10000u)
    {
        BSP_LED_Toggle(LED_RED);
        BSP_LED_Toggle(LED_GREEN);
        i = 0u;
    }
}
#endif
/**
 *******************************************************************************
 ** \brief  Deint User state and associated variables
 ** \param  none
 ** \retval none
 ******************************************************************************/
void host_user_denint(void)
{
    USBH_USR_ApplicationState = USH_USR_FS_INIT;
}

/**
 *******************************************************************************
 ** \brief  initialize the key board
 ** \param  none
 ** \retval none
 ******************************************************************************/
void user_keyboard_init(void)
{
    DDL_Printf((void*)USB_HID_KeybrdStatus);
    DDL_Printf(">>--Use Keyboard to tape characters: \n\n");
    DDL_Printf("\n\n\n\n\n\n");
}

/**
 *******************************************************************************
 ** \brief  display key value
 ** \param  data: key value
 ** \retval none
 ******************************************************************************/
void user_keyboard_dataprocess(uint8_t data)
{
    DDL_Printf("%c", (char)data);
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
                Mouse_ButtonPress (idx);
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
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
