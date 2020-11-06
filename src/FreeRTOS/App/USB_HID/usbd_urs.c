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
/** \file usbd_usr.c
 **
 ** A detailed description is available at
 ** @link
		This file includes the user application layer.
	@endlink
 **
 **   - 2018-05-21  1.0  gouwei First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"
#include "usbd_usr.h"
#include "usbd_ioreq.h"
#include "usb_conf.h"
#include "usb_bsp.h"


/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/


/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
USBD_Usr_cb_TypeDef USR_cb =
{
    USBD_USR_Init,
    USBD_USR_DeviceReset,
    USBD_USR_DeviceConfigured,
    USBD_USR_DeviceSuspended,
    USBD_USR_DeviceResumed,

    USBD_USR_DeviceConnected,
    USBD_USR_DeviceDisconnected,
};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
* \brief  Key_Config
*         Key initialization
* \param  None
* \retval None
*/
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
* \brief  USBD_USR_Init
*         Displays the message on LCD for host lib initialization
* \param  None
* \retval None
*/
void USBD_USR_Init(void)
{
    /* Configure the IOE on which the JoyStick is connected */
    Key_Config();

    /* Setup SysTick Timer for 20 msec interrupts This interrupt is used to probe the joystick */
//    if (SysTick_Config(SystemCoreClock / 50))
//    {
//        /* Capture error */
//        while (1);
//    }
}

/**
* \brief  USBD_USR_DeviceReset
*         Displays the message on LCD on device Reset Event
* \param  speed : device speed
* \retval None
*/
void USBD_USR_DeviceReset(uint8_t speed )
{
    switch (speed)
    {
        case USB_OTG_SPEED_HIGH:
             printf("     USB Device Library v1.1.0 [HS]\n" );
             break;

        case USB_OTG_SPEED_FULL:
             printf("     USB Device Library v1.1.0 [FS]\n" );
             break;
        default:
             printf("     USB Device Library v1.1.0 [??]\n" );
             break;
    }
}


/**
* \brief  USBD_USR_DeviceConfigured
*         Displays the message on LCD on device configuration Event
* \param  None
* \retval Staus
*/
void USBD_USR_DeviceConfigured (void)
{
    printf("> HID Interface started.\n");
}


/**
* \brief  USBD_USR_DeviceConnected
*         Displays the message on LCD on device connection Event
* \param  None
* \retval Staus
*/
void USBD_USR_DeviceConnected (void)
{
    printf("> USB Device Connected.\n");
}


/**
* \brief  USBD_USR_DeviceDisonnected
*         Displays the message on LCD on device disconnection Event
* \param  None
* \retval Staus
*/
void USBD_USR_DeviceDisconnected (void)
{
    printf("> USB Device Disconnected.\n");
}

/**
* \brief  USBD_USR_DeviceSuspended
*         Displays the message on LCD on device suspend Event
* \param  None
* \retval None
*/
void USBD_USR_DeviceSuspended(void)
{
    printf("> USB Device in Suspend Mode.\n");
    /* Users can do their application actions here for the USB-Reset */
}


/**
* \brief  USBD_USR_DeviceResumed
*         Displays the message on LCD on device resume Event
* \param  None
* \retval None
*/
void USBD_USR_DeviceResumed(void)
{
    printf("> USB Device in Idle Mode.\n");
    /* Users can do their application actions here for the USB-Reset */
}
