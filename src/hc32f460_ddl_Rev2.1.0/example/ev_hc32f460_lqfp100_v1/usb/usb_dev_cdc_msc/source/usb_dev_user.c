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
 ** \brief  usb_dev_user_init
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_init(void)
{

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
    DDL_Printf(">>The Interface for COMPOSITE Device of MSC and CDC starts.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_conn
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_conn (void)
{
    DDL_Printf(">>The USB COMPOSITE Device of MSC and CDC connects.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_disconn
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_disconn (void)
{
    DDL_Printf(">>The USB COMPOSITE Device of MSC and CDC disconnected.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_devsusp
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_devsusp(void)
{
    DDL_Printf(">>The USB COMPOSITE Device of MSC and CDC is in suspending Status.\n");
}

/**
 *******************************************************************************
 ** \brief  usb_dev_user_devresume
 ** \param  none
 ** \retval none
 ******************************************************************************/
void usb_dev_user_devresume(void)
{
    DDL_Printf(">>The USB COMPOSITE Device of MSC and CDC resumes.\n");
}

/*******************************************************************************
 * EOF 
 ******************************************************************************/
