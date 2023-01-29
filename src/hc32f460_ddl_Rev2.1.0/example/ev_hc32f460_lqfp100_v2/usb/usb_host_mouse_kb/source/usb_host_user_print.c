/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file usb_host_user_print.c
 **
 ** A detailed description is available at
 ** @link
        This file includes the user application layer.
    @endlink
 **
 **   - 2021-04-16  CDT First version for USB hid mouse & keyboard demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_host_user_print.h"
#include "usb_app_conf.h"
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
 ** \brief  The function is to handle mouse scroll to upadte the mouse position
 **         through uart port.
 ** \param   x : USB Mouse X co-ordinate
 ** \param   y : USB Mouse Y co-ordinate
 ** \retval none
 ******************************************************************************/
void Mouse_PositionUpdate(int8_t x, int8_t y)
{
    DDL_Printf("X: %d, Y: %d\r\n",x,y);
}

/**
 *******************************************************************************
 ** \brief  The function is to handle event when the mouse button is pressed
 ** \param  button_idx : mouse button pressed
 ** \retval none
 ******************************************************************************/
void Mouse_ButtonPress(uint8_t button_idx)
{
    switch (button_idx)
    {
        /* Left Button Pressed */
        case 0 :
            DDL_Printf("L Pressed!\r\n");
            break;

        /* Right Button Pressed */
        case 1 :
            DDL_Printf("R Pressed!\r\n");
            break;

        /* Middle button Pressed */
        case 2 :
            DDL_Printf("M Pressed!\r\n");
            break;
    }
}

/**
 *******************************************************************************
 ** \brief  The function is to handle event when the mouse button is released
 ** \param  button_idx : mouse button released
 ** \retval none
 ******************************************************************************/
void Mouse_ButtonRelease(uint8_t button_idx)
{
    switch (button_idx)
    {
        /* Left Button Released */
        case 0 :
            DDL_Printf("L Released!\r\n");
            break;

        /* Right Button Released */
        case 1 :
            DDL_Printf("R Released!\r\n");
            break;

        /* Middle Button Released */
        case 2 :
            DDL_Printf("M Released!\r\n");
            break;
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
