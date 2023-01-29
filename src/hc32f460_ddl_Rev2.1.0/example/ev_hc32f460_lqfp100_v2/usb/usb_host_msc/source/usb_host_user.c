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
 **   - 2021-04-16  CDT First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "usb_host_user.h"
#ifdef USB_MSC_FAT_VALID
#include "ff.h"       /* FATFS */
#endif
#include "usb_host_msc_class.h"
#include "usb_host_msc_scsi.h"
#include "usb_host_msc_bot.h"
#include "hc32_ddl.h"


/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* USBH_USR_Private_Macros */
extern usb_core_instance          usb_app_instance;

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t USB_HOST_USER_AppState = USH_USR_FS_INIT;
uint8_t filenameString[15]  = {0u};
#ifdef USB_MSC_FAT_VALID
FATFS fatfs;
FIL file;
#endif
uint8_t line_idx = 0u;

/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */

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
const char* MSG_HOST_INIT        = "> Host Library Initialized\n";
const char* MSG_DEV_ATTACHED     = "> Device Attached \n";
const char* MSG_DEV_DISCONNECTED = "> Device Disconnected\n";
const char* MSG_DEV_ENUMERATED   = "> Enumeration completed \n";
const char* MSG_DEV_HIGHSPEED    = "> High speed device detected\n";
const char* MSG_DEV_FULLSPEED    = "> Full speed device detected\n";
const char* MSG_DEV_LOWSPEED     = "> Low speed device detected\n";
const char* MSG_DEV_ERROR        = "> Device fault \n";

const char* MSG_MSC_CLASS        = "> Mass storage device connected\n";
const char* MSG_HID_CLASS        = "> HID device connected\n";
const char* MSG_DISK_SIZE        = "> Size of the disk in MBytes: \n";
const char* MSG_LUN              = "> LUN Available in the device:\n";
const char* MSG_ROOT_CONT        = "> Exploring disk flash ...\n";
const char* MSG_WR_PROTECT       = "> The disk is write protected\n";
const char* MSG_UNREC_ERROR      = "> UNRECOVERED ERROR STATE\n";

#ifdef USB_MSC_FAT_VALID
static uint8_t Explore_Disk (char* path , uint8_t recu_level);
static void     Toggle_Leds(void);
#endif

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for host lib initialization
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_init(void)
{
    static uint8_t startup = 0u;

    if(startup == 0u )
    {
        startup = 1u;

        DDL_Printf(" USB OTG FS MSC Host\n");
        DDL_Printf("> USB Host library started.\n");
        DDL_Printf("     USB Host Library v2.1.0\n" );
    }
}

/**
 *******************************************************************************
 ** \brief  Displays the message on terminal via DDL_Printf
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_devattached(void)
{
    DDL_Printf((void *)MSG_DEV_ATTACHED);
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_UnrecoveredError
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_unrecoverederror(void)
{

  /* Set default screen color*/
    DDL_Printf((void *)MSG_UNREC_ERROR);
}

/**
 *******************************************************************************
 ** \brief  Device disconnect event
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_devdisconn(void)
{
    /* Set default screen color*/
    DDL_Printf((void *)MSG_DEV_DISCONNECTED);
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_ResetUSBDevice
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_devreset(void)
{
    /* callback for USB-Reset */
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_DeviceSpeedDetected
 ** \param  DeviceSpeed : USB speed
 ** \retval None
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
 ** \brief  USBH_USR_Device_DescAvailable
 ** \param  DeviceDesc : device descriptor
 ** \retval None
 ******************************************************************************/
void host_user_devdescavailable(void *DeviceDesc)
{
    usb_host_devdesc_typedef *hs;
    hs = DeviceDesc;

    DDL_Printf("VID : %04lXh\n" , (uint32_t)(*hs).idVendor);
    DDL_Printf("PID : %04lXh\n" , (uint32_t)(*hs).idProduct);
}

/**
 *******************************************************************************
 ** \brief  USBH_USR_DeviceAddressAssigned
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_devaddrdistributed(void)
{

}

/**
 *******************************************************************************
 ** \brief  USBH_USR_Configuration_DescAvailable
 ** \param  cfgDesc : Configuration desctriptor
 ** \param  itfDesc : Interface desctriptor
 ** \param  epDesc : Endpoint desctriptor
 ** \retval None
 ******************************************************************************/
void host_user_cfgdescavailable(usb_host_cfgdesc_typedef * cfgDesc,
                                usb_host_itfdesc_typedef *itfDesc,
                                USB_HOST_EPDesc_TypeDef *epDesc)
{
    usb_host_itfdesc_typedef *id;

    id = itfDesc;

    if((*id).bInterfaceClass  == 0x08u)
    {
        DDL_Printf((void *)MSG_MSC_CLASS);
    }
    else if((*id).bInterfaceClass  == 0x03u)
    {
        DDL_Printf((void *)MSG_HID_CLASS);
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
 ** \retval None
 ******************************************************************************/
void host_user_mfcstring(void *ManufacturerString)
{
    DDL_Printf("Manufacturer : %s\n", (char *)ManufacturerString);
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for product String
 ** \param  ProductString
 ** \retval None
 ******************************************************************************/
void host_user_productstring(void *ProductString)
{
    DDL_Printf("Product : %s\n", (char *)ProductString);
}

/**
 *******************************************************************************
 ** \brief  Displays the message on LCD for SerialNum_String
 ** \param  SerialNumString
 ** \retval None
 ******************************************************************************/
void host_user_serialnum(void *SerialNumString)
{
    DDL_Printf( "Serial Number : %s\n", (char *)SerialNumString);
}

/**
 *******************************************************************************
 ** \brief  User response request is displayed to ask application jump to class
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_enumcompl(void)
{

    /* Enumeration complete */
    DDL_Printf((void *)MSG_DEV_ENUMERATED);
    DDL_Printf("To see the root content of the disk : \n" );
    DDL_Printf("Press USER KEY...\n");

}

/**
 *******************************************************************************
 ** \brief  Device is not supported
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_devunsupported(void)
{
    DDL_Printf ("> Device not supported.\n");
}

/**
 *******************************************************************************
 ** \brief  User Action for application state entry
 ** \param  None
 ** \retval HOST_USER_STATUS : User response for key button
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
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_overcurrent(void)
{
    DDL_Printf ("Overcurrent detected.\n");
}

/**
 *******************************************************************************
 ** \brief  Demo application for mass storage
 ** \param  None
 ** \retval None
 ******************************************************************************/
int host_user_msc_app(void)
{
#ifdef USB_MSC_FAT_VALID
    FRESULT res;
    uint8_t writeTextBuff[] = "HDSC Connectivity line Host Demo application using FAT_FS   ";
    uint16_t bytesWritten, bytesToWrite;
    uint32_t u32DevConnectTmp = 0ul;

    switch(USB_HOST_USER_AppState)
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

            USB_HOST_USER_AppState = USH_USR_FS_READLIST;
            break;

        case USH_USR_FS_READLIST:
            DDL_Printf((void *)MSG_ROOT_CONT);
            Explore_Disk("0:/", 1u);
            line_idx = 0u;
            USB_HOST_USER_AppState = USH_USR_FS_WRITEFILE;
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
                USB_HOST_USER_AppState = USH_USR_FS_IDLE;
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

            USB_HOST_USER_AppState = USH_USR_FS_IDLE;
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
                DDL_Printf("Press USER KEY to continue...\n");

                /*Key B3 in polling*/
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
 ** \param  None
 ** \retval None
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
 ** \param  None
 ** \retval None
 ******************************************************************************/
void host_user_denint(void)
{
    USB_HOST_USER_AppState = USH_USR_FS_INIT;
}


/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
