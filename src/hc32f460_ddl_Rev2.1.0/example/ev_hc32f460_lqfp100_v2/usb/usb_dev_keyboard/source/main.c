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
#include "usb_dev_keyboard_class.h"

usb_core_instance  usb_dev;

/**
 *******************************************************************************
 ** \brief  get the status of keys
 ** \param  none
 ** \retval button id
 ******************************************************************************/
uint8_t key_state(void)
{
    if(BSP_KEY_GetStatus(BSP_KEY_2))
    {
        return 1;
    }
    else if(BSP_KEY_GetStatus(BSP_KEY_4))
    {
        return 2;
    }
    else if(BSP_KEY_GetStatus(BSP_KEY_6))
    {
        return 3;
    }
    else if(BSP_KEY_GetStatus(BSP_KEY_8))
    {
        return 4;
    }
    else
    {
        return 0;
    }
}

/**
 *******************************************************************************
 ** \brief  send the packet of key values
 ** \param  key: key status
 ** \retval none
 ******************************************************************************/
void SendKeyValue(uint8_t key)
{
    uint8_t tmp_buf[8]={0xfe,1,0,0,0,0,0,0};
    switch(key)
    {
        case 1:
            tmp_buf[2] = 0x04;     //'a'
            break;
        case 2:
            tmp_buf[2] = 0x05;     //'b'
            break;
        case 3:
            tmp_buf[2] = 0x06;     //'c'
            break;
        case 4:
            tmp_buf[2] = 0x07;     //'d'
            break;
        default:
            break;
    }
    if(key != 0 )
    {
        hd_usb_deveptx(&usb_dev, HID_IN_EP, tmp_buf, 8);
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
    __IO uint8_t key_stat_tmp;
    uint8_t tmp_buf[8]={0xfe,1,0,0,0,0,0,0};
    uint8_t press_status;
    press_status = 0;
    hd_usb_dev_init(&usb_dev, &user_desc, &usb_dev_keyboard_cbk, &user_cb);

    while(usb_dev.dev.device_cur_status != USB_DEV_CONFIGURED)
    {
        ;
    }
    while (1)
    {
        key_stat_tmp = key_state();
        if(key_stat_tmp != 0)
        {
            press_status = 1;
            SendKeyValue(key_stat_tmp);
            Ddl_Delay1ms(20);
        }

        key_stat_tmp = key_state();
        if((key_stat_tmp == 0)&&(press_status == 1))
        {
            press_status = 0;
            hd_usb_deveptx(&usb_dev, HID_IN_EP, tmp_buf, 8);
            Ddl_Delay1ms(20);
        }

    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

