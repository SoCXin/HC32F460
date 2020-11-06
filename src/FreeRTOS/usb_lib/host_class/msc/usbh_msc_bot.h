/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
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
/** \file usbh_msc_bot.h
 **
 ** A detailed description is available at
 ** @link 
		Header file for usbh_msc_bot.c 
	@endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/
#ifndef __USBH_MSC_BOT_H__
#define __USBH_MSC_BOT_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usbh_stdreq.h"


/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_CLASS
  * @{
  */

/** @addtogroup USBH_MSC_CLASS
  * @{
  */
  
/** @defgroup USBH_MSC_BOT
  * \brief This file is the Header file for usbh_msc_core.c
  * @{
  */ 


/** @defgroup USBH_MSC_BOT_Exported_Types
  * @{
  */ 

typedef union _USBH_CBW_Block
{
  struct __CBW
  {
    uint32_t CBWSignature;
    uint32_t CBWTag;
    uint32_t CBWTransferLength;
    uint8_t CBWFlags;
    uint8_t CBWLUN; 
    uint8_t CBWLength;
    uint8_t CBWCB[16];
}field;
  uint8_t CBWArray[31];
}HostCBWPkt_TypeDef;

typedef enum
{
  USBH_MSC_BOT_INIT_STATE = 0,                
  USBH_MSC_BOT_RESET,                
  USBH_MSC_GET_MAX_LUN,              
  USBH_MSC_TEST_UNIT_READY,          
  USBH_MSC_READ_CAPACITY10,
  USBH_MSC_MODE_SENSE6,
  USBH_MSC_REQUEST_SENSE,            
  USBH_MSC_BOT_USB_TRANSFERS,        
  USBH_MSC_DEFAULT_APPLI_STATE,  
  USBH_MSC_CTRL_ERROR_STATE,
  USBH_MSC_UNRECOVERED_STATE
}
MSCState;


typedef struct _BOTXfer
{
uint8_t MSCState;
uint8_t MSCStateBkp;
uint8_t MSCStateCurrent;
uint8_t CmdStateMachine;
uint8_t BOTState;
uint8_t BOTStateBkp;
uint8_t* pRxTxBuff;
uint16_t DataLength;
uint8_t BOTXferErrorCount;
uint8_t BOTXferStatus;
} USBH_BOTXfer_TypeDef;


typedef union _USBH_CSW_Block
{
  struct __CSW
  {
    uint32_t CSWSignature;
    uint32_t CSWTag;
    uint32_t CSWDataResidue;
    uint8_t  CSWStatus;
  }field;
  uint8_t CSWArray[13];
}HostCSWPkt_TypeDef;

/**
  * @}
  */ 



/** @defgroup USBH_MSC_BOT_Exported_Defines
  * @{
  */ 
#define USBH_MSC_SEND_CBW                 1
#define USBH_MSC_SENT_CBW                 2
#define USBH_MSC_BOT_DATAIN_STATE         3
#define USBH_MSC_BOT_DATAOUT_STATE        4
#define USBH_MSC_RECEIVE_CSW_STATE        5
#define USBH_MSC_DECODE_CSW               6
#define USBH_MSC_BOT_ERROR_IN             7
#define USBH_MSC_BOT_ERROR_OUT            8


#define USBH_MSC_BOT_CBW_SIGNATURE        0x43425355
#define USBH_MSC_BOT_CBW_TAG              0x20304050             
#define USBH_MSC_BOT_CSW_SIGNATURE        0x53425355           
#define USBH_MSC_CSW_DATA_LENGTH          0x000D
#define USBH_MSC_BOT_CBW_PACKET_LENGTH    31
#define USBH_MSC_CSW_LENGTH               13  
#define USBH_MSC_CSW_MAX_LENGTH           63     

/* CSW Status Definitions */
#define USBH_MSC_CSW_CMD_PASSED           0x00
#define USBH_MSC_CSW_CMD_FAILED           0x01
#define USBH_MSC_CSW_PHASE_ERROR          0x02

#define USBH_MSC_SEND_CSW_DISABLE         0
#define USBH_MSC_SEND_CSW_ENABLE          1

#define USBH_MSC_DIR_IN                   0
#define USBH_MSC_DIR_OUT                  1
#define USBH_MSC_BOTH_DIR                 2

//#define USBH_MSC_PAGE_LENGTH                 0x40
#define USBH_MSC_PAGE_LENGTH              512


#define CBW_CB_LENGTH                     16
#define CBW_LENGTH                        10
#define CBW_LENGTH_TEST_UNIT_READY         6

#define USB_REQ_BOT_RESET                0xFF
#define USB_REQ_GET_MAX_LUN              0xFE

#define MAX_BULK_STALL_COUNT_LIMIT       0x04   /* If STALL is seen on Bulk 
                                         Endpoint continously, this means 
                                         that device and Host has phase error
                                         Hence a Reset is needed */

/**
  * @}
  */ 

/** @defgroup USBH_MSC_BOT_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_MSC_BOT_Exported_Variables
  * @{
  */ 
extern USBH_BOTXfer_TypeDef USBH_MSC_BOTXferParam;
extern HostCBWPkt_TypeDef USBH_MSC_CBWData;
extern HostCSWPkt_TypeDef USBH_MSC_CSWData;
/**
  * @}
  */ 

/** @defgroup USBH_MSC_BOT_Exported_FunctionsPrototype
  * @{
  */ 
void USBH_MSC_HandleBOTXfer(USB_OTG_CORE_HANDLE *pdev,
                            USBH_HOST *phost);
uint8_t USBH_MSC_DecodeCSW(USB_OTG_CORE_HANDLE *pdev,
                           USBH_HOST *phost);
void USBH_MSC_Init(USB_OTG_CORE_HANDLE *pdev);
USBH_Status USBH_MSC_BOT_Abort(USB_OTG_CORE_HANDLE *pdev, 
                               USBH_HOST *phost,
                               uint8_t direction);
/**
  * @}
  */ 

#endif  //__USBH_MSC_BOT_H__

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


