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
/** \file usbh_ioreq.c
 **
 ** A detailed description is available at
 ** @link
		This file handles the issuing of the USB transactions
	@endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usbh_ioreq.h"

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
static USBH_Status USBH_SubmitSetupRequest(USBH_HOST *phost,
                                           uint8_t* buff,
                                           uint16_t length);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 *******************************************************************************
 ** \brief  USBH_CtlReq
 **         USBH_CtlReq sends a control request and provide the status after
 **            completion of the request
 ** \param  pdev: Selected device
 ** \param  req: Setup Request Structure
 ** \param  buff: data buffer address to store the response
 ** \param  length: length of the response
 ** \retval Status
 ******************************************************************************/
USBH_Status USBH_CtlReq     (USB_OTG_CORE_HANDLE *pdev,
                             USBH_HOST           *phost,
                             uint8_t             *buff,
                             uint16_t            length)
{
    USBH_Status status;
    status = USBH_BUSY;

    switch (phost->RequestState)
    {
        case CMD_SEND:
            /* Start a SETUP transfer */
            USBH_SubmitSetupRequest(phost, buff, length);
            phost->RequestState = CMD_WAIT;
            status = USBH_BUSY;
            break;
        case CMD_WAIT:
            if (phost->Control.state == CTRL_COMPLETE )
            {
                /* Commands successfully sent and Response Received  */
                phost->RequestState = CMD_SEND;
                phost->Control.state =CTRL_IDLE;
                status = USBH_OK;
            }
            else if  (phost->Control.state == CTRL_ERROR)
            {
                /* Failure Mode */
                phost->RequestState = CMD_SEND;
                status = USBH_FAIL;
            }
            else if  (phost->Control.state == CTRL_STALLED )
            {
                /* Commands successfully sent and Response Received  */
                phost->RequestState = CMD_SEND;
                status = USBH_NOT_SUPPORTED;
            }
            break;
        default:
            break;
    }
    return status;
}

/**
 *******************************************************************************
 ** \brief  USBH_CtlSendSetup
 **         Sends the Setup Packet to the Device
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer from which the Data will be send to Device
 ** \param  hc_num: Host channel Number
 ** \retval Status
 ******************************************************************************/
USBH_Status USBH_CtlSendSetup ( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].data_pid = HC_PID_SETUP;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = USBH_SETUP_PKT_SIZE;

    return (USBH_Status)HCD_SubmitRequest (pdev , hc_num);
}


/**
 *******************************************************************************
 ** \brief  USBH_CtlSendData
 **         Sends a data Packet to the Device
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer from which the Data will be sent to Device
 ** \param  length: Length of the data to be sent
 ** \param  hc_num: Host channel Number
 ** \retval Status
 ******************************************************************************/
USBH_Status USBH_CtlSendData ( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint16_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

    if ( length == 0 )
    { /* For Status OUT stage, Length==0, Status Out PID = 1 */
        pdev->host.hc[hc_num].toggle_out = 1;
    }

    /* Set the Data Toggle bit as per the Flag */
    if ( pdev->host.hc[hc_num].toggle_out == 0)
    { /* Put the PID 0 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    { /* Put the PID 1 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1 ;
    }

    HCD_SubmitRequest (pdev , hc_num);

    return USBH_OK;
}


/**
 *******************************************************************************
 ** \brief  USBH_CtlReceiveData
 **         Receives the Device Response to the Setup Packet
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer in which the response needs to be copied
 ** \param  length: Length of the data to be received
 ** \param  hc_num: Host channel Number
 ** \retval Status.
 ******************************************************************************/
USBH_Status USBH_CtlReceiveData(USB_OTG_CORE_HANDLE *pdev,
                                uint8_t* buff,
                                uint16_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

    HCD_SubmitRequest (pdev , hc_num);

    return USBH_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_BulkSendData
 **         Sends the Bulk Packet to the device
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer from which the Data will be sent to Device
 ** \param  length: Length of the data to be sent
 ** \param  hc_num: Host channel Number
 ** \retval Status
 ******************************************************************************/
USBH_Status USBH_BulkSendData ( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint16_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

    /* Set the Data Toggle bit as per the Flag */
    if ( pdev->host.hc[hc_num].toggle_out == 0)
    { /* Put the PID 0 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    { /* Put the PID 1 */
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1 ;
    }

    HCD_SubmitRequest (pdev , hc_num);
    return USBH_OK;
}


/**
 *******************************************************************************
 ** \brief  USBH_BulkReceiveData
 **         Receives IN bulk packet from device
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer in which the received data packet to be copied
 ** \param  length: Length of the data to be received
 ** \param  hc_num: Host channel Number
 ** \retval Status.
 ******************************************************************************/
USBH_Status USBH_BulkReceiveData( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint16_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;


    if( pdev->host.hc[hc_num].toggle_in == 0)
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

    HCD_SubmitRequest (pdev , hc_num);
    return USBH_OK;
}


/**
 *******************************************************************************
 ** \brief  USBH_InterruptReceiveData
 **         Receives the Device Response to the Interrupt IN token
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer in which the response needs to be copied
 ** \param  length: Length of the data to be received
 ** \param  hc_num: Host channel Number
 ** \retval Status.
 ******************************************************************************/
USBH_Status USBH_InterruptReceiveData( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint16_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

    if(pdev->host.hc[hc_num].toggle_in == 0)
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

    /* toggle DATA PID */
    pdev->host.hc[hc_num].toggle_in ^= 1;

    HCD_SubmitRequest (pdev , hc_num);
    return USBH_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_InterruptSendData
 **         Sends the data on Interrupt OUT Endpoint
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer from where the data needs to be copied
 ** \param  length: Length of the data to be sent
 ** \param  hc_num: Host channel Number
 ** \retval Status.
 ******************************************************************************/
USBH_Status USBH_InterruptSendData( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint16_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;

    if(pdev->host.hc[hc_num].toggle_in == 0)
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;
    }
    else
    {
        pdev->host.hc[hc_num].data_pid = HC_PID_DATA1;
    }

    pdev->host.hc[hc_num].toggle_in ^= 1;
    HCD_SubmitRequest (pdev , hc_num);
    return USBH_OK;
}


/**
 *******************************************************************************
 ** \brief  USBH_SubmitSetupRequest
 **         Start a setup transfer by changing the state-machine and
 **         initializing  the required variables needed for the Control Transfer
 ** \param  pdev: Selected device
 ** \param  setup: Setup Request Structure
 ** \param  buff: Buffer used for setup request
 ** \param  length: Length of the data
 ** \retval Status.
 ******************************************************************************/
static USBH_Status USBH_SubmitSetupRequest(USBH_HOST *phost,
                                           uint8_t* buff,
                                           uint16_t length)
{
    /* Save Global State */
    phost->gStateBkp =   phost->gState;

    /* Prepare the Transactions */
    phost->gState = HOST_CTRL_XFER;
    phost->Control.buff = buff;
    phost->Control.length = length;
    phost->Control.state = CTRL_SETUP;

    return USBH_OK;
}


/**
 *******************************************************************************
 ** \brief  USBH_IsocReceiveData
 **         Receives the Device Response to the Isochronous IN token
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer in which the response needs to be copied
 ** \param  length: Length of the data to be received
 ** \param  hc_num: Host channel Number
 ** \retval Status.
 ******************************************************************************/
USBH_Status USBH_IsocReceiveData( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint32_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 1;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;

    HCD_SubmitRequest (pdev , hc_num);

    return USBH_OK;
}

/**
 *******************************************************************************
 ** \brief  USBH_IsocSendData
 **         Sends the data on Isochronous OUT Endpoint
 ** \param  pdev: Selected device
 ** \param  buff: Buffer pointer from where the data needs to be copied
 ** \param  length: Length of the data to be sent
 ** \param  hc_num: Host channel Number
 ** \retval Status.
 ******************************************************************************/
USBH_Status USBH_IsocSendData( USB_OTG_CORE_HANDLE *pdev,
                                uint8_t *buff,
                                uint32_t length,
                                uint8_t hc_num)
{
    pdev->host.hc[hc_num].ep_is_in = 0;
    pdev->host.hc[hc_num].xfer_buff = buff;
    pdev->host.hc[hc_num].xfer_len = length;
    pdev->host.hc[hc_num].data_pid = HC_PID_DATA0;

    HCD_SubmitRequest (pdev , hc_num);

    return USBH_OK;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
