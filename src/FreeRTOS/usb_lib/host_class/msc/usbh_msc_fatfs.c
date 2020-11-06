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
/** \file usbh_msc_fatfs.c
 **
 ** A detailed description is available at
 ** @link
		This file provides the fatfs functions.
	@endlink
 **
 **   - 2018-12-26  1.0  wangmin First version for USB demo.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_conf.h"
#include "diskio.h"
#include "usbh_msc_core.h"

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
static volatile DSTATUS Stat = STA_NOINIT;	/* Disk status */
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

extern USB_OTG_CORE_HANDLE          USB_OTG_Core;
extern USBH_HOST                     USB_Host;

/**
 *******************************************************************************
 ** \brief  Initialize Disk Drive
 ** \param  drv Physical drive number (0)
 ** \retval DSTATUS
 ******************************************************************************/
DSTATUS disk_initialize (
                         BYTE drv		/* Physical drive number (0) */
                           )
{
    if(HCD_IsDeviceConnected(&USB_OTG_Core))
    {
    Stat &= ~STA_NOINIT;
    }
    return Stat;
}

/**
 *******************************************************************************
 ** \brief  Get Disk Status
 ** \param  drv Physical drive number (0)
 ** \retval DSTATUS
 ******************************************************************************/
DSTATUS disk_status (
                     BYTE drv		/* Physical drive number (0) */
                       )
{
    if (drv) return STA_NOINIT;		/* Supports only single drive */
    return Stat;
}

/**
 *******************************************************************************
 ** \brief  Read Sector(s)
 ** \param  drv Physical drive number (0)
 ** \param  buff Pointer to the data buffer to store read data
 ** \param  sector Start sector number (LBA)
 ** \param  count Sector count (1..255)
 ** \retval DSTATUS
 ******************************************************************************/
DRESULT disk_read (
                   BYTE drv,			/* Physical drive number (0) */
                   BYTE *buff,			/* Pointer to the data buffer to store read data */
                   DWORD sector,		/* Start sector number (LBA) */
                   BYTE count			/* Sector count (1..255) */
                     )
{
  BYTE status = USBH_MSC_OK;

  if (drv || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;


  if(HCD_IsDeviceConnected(&USB_OTG_Core))
  {

    do
    {
      status = USBH_MSC_Read10(&USB_OTG_Core, buff,sector,512 * count);
      USBH_MSC_HandleBOTXfer(&USB_OTG_Core ,&USB_Host);

      if(!HCD_IsDeviceConnected(&USB_OTG_Core))
      {
        return RES_ERROR;
      }
    }
    while(status == USBH_MSC_BUSY );
  }

  if(status == USBH_MSC_OK)
    return RES_OK;
  return RES_ERROR;

}


#if _READONLY == 0
/**
 *******************************************************************************
 ** \brief  Write Sector(s)
 ** \param  drv Physical drive number (0)
 ** \param  buff Pointer to the data to be written
 ** \param  sector Start sector number (LBA)
 ** \param  count Sector count (1..255)
 ** \retval DSTATUS
 ******************************************************************************/
DRESULT disk_write (
                    BYTE drv,			/* Physical drive number (0) */
                    const BYTE *buff,	/* Pointer to the data to be written */
                    DWORD sector,		/* Start sector number (LBA) */
                    BYTE count			/* Sector count (1..255) */
                      )
{
    BYTE status = USBH_MSC_OK;
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;
    if(HCD_IsDeviceConnected(&USB_OTG_Core))
    {
        do
        {
            status = USBH_MSC_Write10(&USB_OTG_Core,(BYTE*)buff,sector,512 * count);
            USBH_MSC_HandleBOTXfer(&USB_OTG_Core, &USB_Host);

            if(!HCD_IsDeviceConnected(&USB_OTG_Core))
            {
                return RES_ERROR;
            }
        }
        while(status == USBH_MSC_BUSY );
    }

    if(status == USBH_MSC_OK)
        return RES_OK;
    return RES_ERROR;
}
#endif /* _READONLY == 0 */

#if _USE_IOCTL != 0
/**
 *******************************************************************************
 ** \brief  Miscellaneous Functions
 ** \param  drv Physical drive number (0)
 ** \param  ctrl Control code
 ** \param  buff Buffer to send/receive control data
 **
 ** \retval DSTATUS
 ******************************************************************************/
DRESULT disk_ioctl (
                    BYTE drv,		/* Physical drive number (0) */
                    BYTE ctrl,		/* Control code */
                    void *buff		/* Buffer to send/receive control data */
                      )
{
    DRESULT res = RES_OK;

    if (drv) return RES_PARERR;

    res = RES_ERROR;

    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (ctrl)
    {
        case CTRL_SYNC :		/* Make sure that no pending write process */
            res = RES_OK;
            break;
        case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
            *(DWORD*)buff = (DWORD) USBH_MSC_Param.MSCapacity;
            res = RES_OK;
            break;
        case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
            *(WORD*)buff = 512;
            res = RES_OK;
            break;
        case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
            *(DWORD*)buff = 512;
            break;
        default:
            res = RES_PARERR;
    }
    return res;
}
#endif /* _USE_IOCTL != 0 */
