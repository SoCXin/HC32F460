/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_dev_user.c
 **
 ** A detailed description is available at
 ** @link
        This file includes the user application layer.
    @endlink
 **
 **   - 2021-04-14  1.0  CDT First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "usb_dev_user.h"
#include "usb_dev_ctrleptrans.h"
#include "usb_app_conf.h"
#include "usb_bsp.h"


void usb_dev_user_init(void);
void usb_dev_user_rst (void);
void usb_dev_user_devcfg (void);
void usb_dev_user_devsusp(void);
void usb_dev_user_devresume(void);
void usb_dev_user_conn(void);
void usb_dev_user_disconn(void); 
/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
usb_dev_user_func user_cb =
{
    &usb_dev_user_init,
    &usb_dev_user_rst,
    &usb_dev_user_devcfg,
    &usb_dev_user_devsusp,
    &usb_dev_user_devresume,
    &usb_dev_user_conn,
    &usb_dev_user_disconn,
};

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

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
 ** \brief  Key initialization
 ** \param  None
 ** \retval None
 ******************************************************************************/
static void Key_Config(void)
{
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_In;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    /* LED0 Port/Pin initialization */
    PORT_Init(KEY_PORT, KEY_UP, &stcPortInit);
    PORT_Init(KEY_PORT, KEY_DOWN, &stcPortInit);
    PORT_Init(KEY_PORT, KEY_LEFT, &stcPortInit);
    PORT_Init(KEY_PORT, KEY_RIGHT, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_init
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_init(void)
{
    /* Configure the IOE on which the JoyStick is connected */
    Key_Config();

    /* Setup SysTick Timer for 20 msec interrupts This interrupt is used to probe the joystick */
    if (SysTick_Config(SystemCoreClock / 50u))
    {
        /* Capture error */
        while (1)
        {
            ;
        }
    }
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_rst
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_rst(void)
{
    DDL_Printf(">>USB Device Library.\n" );
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_devcfg
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_devcfg (void)
{
    DDL_Printf(">>The Interface for MSC Device starts.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_conn
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_conn (void)
{
    DDL_Printf(">>The USB MSC Device connects.\n");
}

/**
 *******************************************************************************
 ** \brief  USBD_USR_DeviceDisonnected
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_disconn (void)
{
    DDL_Printf(">>The USB MSC Device disconnected.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_devsusp
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_devsusp(void)
{
    DDL_Printf(">>The USB MSC Device is in suspending Status.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_devresume
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_devresume(void)
{
    DDL_Printf(">>The USB MSC Device resumes.\n");
}

/*******************************************************************************
 * EOF 
 ******************************************************************************/
