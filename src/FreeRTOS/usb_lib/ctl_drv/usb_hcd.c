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
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
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
/** \file usbd_desc.c
 **
 ** A detailed description is available at
 ** @link
        Host Interface Layer.
    @endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_core.h"
#include "usb_hcd.h"
#include "usb_conf.h"
#include "usb_bsp.h"

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
 ** \brief  HCD_Init
 **         Initialize the HOST portion of the driver.
 ** \param  pdev: Selected device
 ** \param  base_address: OTG base address
 ** \retval Status
 ******************************************************************************/
uint32_t HCD_Init(USB_OTG_CORE_HANDLE *pdev, USB_OTG_CORE_ID_TypeDef coreID)
{
    uint8_t i = 0u;
    pdev->host.ConnSts = 0u;

    for (i= 0u; i< USB_OTG_MAX_TX_FIFOS; i++)
    {
        pdev->host.ErrCnt[i]  = 0u;
        pdev->host.XferCnt[i]   = 0u;
        pdev->host.HC_Status[i]   = HC_IDLE;
    }
    pdev->host.hc[0].max_packet  = 8u;

    USB_OTG_SelectCore(pdev, coreID);
#ifndef DUAL_ROLE_MODE_ENABLED
    USB_OTG_DisableGlobalInt(pdev);
    USB_OTG_CoreInit(pdev);

    /* Force Host Mode*/
#ifndef USE_OTG_MODE
    USB_OTG_SetCurrentMode(pdev , HOST_MODE);
#endif
    USB_OTG_CoreInitHost(pdev);
    USB_OTG_EnableGlobalInt(pdev);
#endif
    return 0ul;
}


/**
 *******************************************************************************
 ** \brief  HCD_GetCurrentSpeed
 **         Get Current device Speed.
 ** \param  pdev : Selected device
 ** \retval Status
 ******************************************************************************/
uint32_t HCD_GetCurrentSpeed (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_HPRT0_TypeDef  HPRT0;
    HPRT0.b = *(__IO stc_bUSB_OTG_HPRT0_t*)&USB_OTG_READ_REG32(pdev->regs.HPRT0);   /* C-STAT */

    return HPRT0.b.prtspd;
}

/**
 *******************************************************************************
 ** \brief  HCD_ResetPort
 **         Issues the reset command to device
 ** \param  pdev : Selected device
 ** \retval Status
 ******************************************************************************/
uint32_t HCD_ResetPort(USB_OTG_CORE_HANDLE *pdev)
{
    /*
    Before starting to drive a USB reset, the application waits for the OTG
    interrupt triggered by the debounce done bit (DBCDNE bit in OTG_FS_GOTGINT),
    which indicates that the bus is stable again after the electrical debounce
    caused by the attachment of a pull-up resistor on DP (FS) or DM (LS).
    */

    USB_OTG_ResetPort(pdev);
    return 0ul;
}

/**
 *******************************************************************************
 ** \brief  HCD_IsDeviceConnected
 **         Check if the device is connected.
 ** \param  pdev : Selected device
 ** \retval Device connection status. 1 -> connected and 0 -> disconnected
 **
 ******************************************************************************/
uint32_t HCD_IsDeviceConnected(USB_OTG_CORE_HANDLE *pdev)
{
    return (pdev->host.ConnSts);
}

/**
 *******************************************************************************
 ** \brief  HCD_GetCurrentFrame
 **         This function returns the frame number for sof packet
 ** \param  pdev : Selected device
 ** \retval Frame number
 **
 ******************************************************************************/
uint32_t HCD_GetCurrentFrame (USB_OTG_CORE_HANDLE *pdev)
{
    return (USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM) & 0xFFFFul) ;
}

/**
 *******************************************************************************
 ** \brief  HCD_GetURB_State
 **         This function returns the last URBstate
 ** \param  pdev: Selected device
 ** \retval URB_STATE
 **
 ******************************************************************************/
URB_STATE HCD_GetURB_State (USB_OTG_CORE_HANDLE *pdev , uint8_t ch_num)
{
    return pdev->host.URB_State[ch_num] ;
}

/**
 *******************************************************************************
 ** \brief  HCD_GetXferCnt
 **         This function returns the last URBstate
 ** \param  pdev: Selected device
 ** \retval No. of data bytes transferred
 **
 ******************************************************************************/
uint32_t HCD_GetXferCnt (USB_OTG_CORE_HANDLE *pdev, uint8_t ch_num)
{
    return pdev->host.XferCnt[ch_num] ;
}



/**
 *******************************************************************************
 ** \brief  HCD_GetHCState
 **         This function returns the HC Status
 ** \param  pdev: Selected device
 ** \retval HC_STATUS
 **
 ******************************************************************************/
HC_STATUS HCD_GetHCState (USB_OTG_CORE_HANDLE *pdev ,  uint8_t ch_num)
{
    return pdev->host.HC_Status[ch_num] ;
}

/**
 *******************************************************************************
 ** \brief  HCD_HC_Init
 **         This function prepare a HC and start a transfer
 ** \param  pdev: Selected device
 ** \param  hc_num: Channel number
 ** \retval status
 ******************************************************************************/
uint32_t HCD_HC_Init (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
    return USB_OTG_HC_Init(pdev, hc_num);
}

/**
 *******************************************************************************
 ** \brief  HCD_SubmitRequest
 **         This function prepare a HC and start a transfer
 ** \param  pdev: Selected device
 ** \param  hc_num: Channel number
 ** \retval status
 ******************************************************************************/
uint32_t HCD_SubmitRequest (USB_OTG_CORE_HANDLE *pdev , uint8_t hc_num)
{
    pdev->host.URB_State[hc_num] = URB_IDLE;
    pdev->host.hc[hc_num].xfer_count = 0u ;
    return USB_OTG_HC_StartXfer(pdev, hc_num);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
