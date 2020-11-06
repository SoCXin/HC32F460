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
/** \file usbh_hcs.c
 **
 ** A detailed description is available at
 ** @link
		This file implements functions for opening and closing host channels
	@endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usbh_hcs.h"

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
static uint16_t USBH_GetFreeChannel (USB_OTG_CORE_HANDLE *pdev);
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  USBH_Open_Channel
 **         Open a  pipe
 ** \param  pdev : Selected device
 ** \param  hc_num: Host channel Number
 ** \param  dev_address: USB Device address allocated to attached device
 ** \param  speed : USB device speed (Full/Low)
 ** \param  ep_type: end point type (Bulk/int/ctl)
 ** \param  mps: max pkt size
 ** \retval Status
 ******************************************************************************/
uint8_t USBH_Open_Channel  (USB_OTG_CORE_HANDLE *pdev,
                            uint8_t hc_num,
                            uint8_t dev_address,
                            uint8_t speed,
                            uint8_t ep_type,
                            uint16_t mps)
{
    pdev->host.hc[hc_num].ep_num = pdev->host.channel[hc_num]& 0x7F;
    pdev->host.hc[hc_num].ep_is_in = (pdev->host.channel[hc_num] & 0x80 ) == 0x80;
    pdev->host.hc[hc_num].dev_addr = dev_address;
    pdev->host.hc[hc_num].ep_type = ep_type;
    pdev->host.hc[hc_num].max_packet = mps;
    pdev->host.hc[hc_num].speed = speed;
    pdev->host.hc[hc_num].toggle_in = 0;
    pdev->host.hc[hc_num].toggle_out = 0;
    if(speed == HPRT0_PRTSPD_HIGH_SPEED)
    {
        pdev->host.hc[hc_num].do_ping = 1;
    }

    USB_OTG_HC_Init(pdev, hc_num) ;
    return HC_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_Modify_Channel
 **         Modify a  pipe
 ** \param  pdev : Selected device
 ** \param  hc_num: Host channel Number
 ** \param  dev_address: USB Device address allocated to attached device
 ** \param  speed : USB device speed (Full/Low)
 ** \param  ep_type: end point type (Bulk/int/ctl)
 ** \param  mps: max pkt size
 ** \retval Status
 ******************************************************************************/
uint8_t USBH_Modify_Channel (USB_OTG_CORE_HANDLE *pdev,
                            uint8_t hc_num,
                            uint8_t dev_address,
                            uint8_t speed,
                            uint8_t ep_type,
                            uint16_t mps)
{
    if(dev_address != 0)
    {
        pdev->host.hc[hc_num].dev_addr = dev_address;
    }

    if((pdev->host.hc[hc_num].max_packet != mps) && (mps != 0))
    {
        pdev->host.hc[hc_num].max_packet = mps;
    }

    if((pdev->host.hc[hc_num].speed != speed ) && (speed != 0 ))
    {
        pdev->host.hc[hc_num].speed = speed;
    }

    USB_OTG_HC_Init(pdev, hc_num);
    return HC_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_Alloc_Channel
 **         Allocate a new channel for the pipe
 ** \param  ep_addr: End point for which the channel to be allocated
 ** \retval hc_num: Host channel number
 ******************************************************************************/
uint8_t USBH_Alloc_Channel  (USB_OTG_CORE_HANDLE *pdev, uint8_t ep_addr)
{
    uint16_t hc_num;

    hc_num =  USBH_GetFreeChannel(pdev);

    if (hc_num != HC_ERROR)
    {
        pdev->host.channel[hc_num] = HC_USED | ep_addr;
    }
    return hc_num;
}

/**
 *******************************************************************************
 ** \brief  USBH_Free_Pipe
 **         Free the USB host channel
 ** \param  idx: Channel number to be freed
 ** \retval Status
 ******************************************************************************/
uint8_t USBH_Free_Channel  (USB_OTG_CORE_HANDLE *pdev, uint8_t idx)
{
    if(idx < HC_MAX)
    {
        pdev->host.channel[idx] &= HC_USED_MASK;
    }
    return USBH_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_DeAllocate_AllChannel
 **         Free all USB host channel
 ** \param  pdev : core instance
 ** \retval Status
 ******************************************************************************/
uint8_t USBH_DeAllocate_AllChannel  (USB_OTG_CORE_HANDLE *pdev)
{
    uint8_t idx;

    for (idx = 2; idx < HC_MAX ; idx ++)
    {
        pdev->host.channel[idx] = 0;
    }
    return USBH_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_GetFreeChannel
 **         Get a free channel number for allocation to a device endpoint
 ** \param  None
 ** \retval idx: Free Channel number
 ******************************************************************************/
static uint16_t USBH_GetFreeChannel (USB_OTG_CORE_HANDLE *pdev)
{
    uint8_t idx = 0;

    for (idx = 0 + 0 ; idx < HC_MAX ; idx++)
    {
        if ((pdev->host.channel[idx] & HC_USED) == 0)
        {
            return idx;
        }
    }
    return HC_ERROR;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
