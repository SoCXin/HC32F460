/*******************************************************************************
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by HDSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief USB mouse example.
 **
 **   - 2018-12-25  1.0  CDT First version for USB mouse demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "usb_dev_user.h"
#include "usb_dev_desc.h"
#include "usb_bsp.h"
#include "usb_dev_mouse_class.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  The button typedef.
 **
 ******************************************************************************/
typedef enum
{
    BUTTON_NULL = 1u,
    BUTTON_RIGHT,
    BUTTON_LEFT,
    BUTTON_UP,
    BUTTON_DOWN,
}Button_TypeDef;

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define CURSOR_STEP     10u

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
usb_core_instance  usb_dev;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  Key status read function
 ** \param  none
 ** \retval button id
 ******************************************************************************/
Button_TypeDef Key_ReadIOPin_continuous(void)
{
    Button_TypeDef enKey = BUTTON_NULL;
    if(BSP_KEY_GetStatus(BSP_KEY_2))
    {
        enKey = BUTTON_UP;
    }
    else if(BSP_KEY_GetStatus(BSP_KEY_8))
    {
        enKey = BUTTON_DOWN;
    }
    else if(BSP_KEY_GetStatus(BSP_KEY_4))
    {
        enKey = BUTTON_LEFT;
    }
    else if(BSP_KEY_GetStatus(BSP_KEY_6))
    {
        enKey = BUTTON_RIGHT;
    }
    else
    {
        enKey = BUTTON_NULL;
    }

    return enKey;
}

/**
 *******************************************************************************
 ** \brief  get the position of the mouse
 ** \param  none
 ** \retval Pointer to report
 ******************************************************************************/
static uint8_t* get_mouse_pos(void)
{
    int8_t  x = (int8_t)0, y = (int8_t)0;
    static uint8_t HID_Buffer [4];

    switch (Key_ReadIOPin_continuous())
    {
        case BUTTON_UP:
            y -= (int8_t)CURSOR_STEP;
            break;
        case BUTTON_DOWN:
            y += (int8_t)CURSOR_STEP;
            break;
        case BUTTON_LEFT:
            x -= (int8_t)CURSOR_STEP;
            break;
        case BUTTON_RIGHT:
            x += (int8_t)CURSOR_STEP;
            break;
        default:
            break;
    }
    HID_Buffer[0] = (uint8_t)0;
    HID_Buffer[1] = (uint8_t)x;
    HID_Buffer[2] = (uint8_t)y;
    HID_Buffer[3] = (uint8_t)0;

    return HID_Buffer;
}
/**
 *******************************************************************************
 ** \brief  SysTick IRQ function that get mouse position and report it
 ** \param  none
 ** \retval none
 ******************************************************************************/
void SysTick_IrqHandler(void)
{
    uint8_t *buf;

    buf = get_mouse_pos();
    if((buf[1] != 0u) ||(buf[2] != 0u))
    {
        usb_dev_mouse_txreport(&usb_dev, buf, 4u);
    }
}
/**
 *******************************************************************************
 ** \brief  main function for mouse function
 ** \param  none
 ** \retval int32_t Return value, if needed
 ******************************************************************************/
int32_t main (void)
{
    hd_usb_dev_init(&usb_dev, &user_desc, &usb_dev_mouse_cbk, &user_cb);
    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

