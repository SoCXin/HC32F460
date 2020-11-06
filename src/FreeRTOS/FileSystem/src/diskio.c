/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "msg_dbg.h"
#include "sd_card.h"
#define DEBUGE printf
#define DEBUGN	printf
extern stc_sd_handle_t m_stcSdhandle;

extern void enable_sdio_hw(bool enable);

/* Definitions of physical drive number for each drive */
#define SD_NAND		0	/* Example: Map Ramdisk to physical drive 0 */

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
        BYTE pdrv		/* Physical drive nmuber to identify the drive */
        )
{
    if(pdrv == SD_NAND)
    {
//        DEBUGN("fatfs call \r\n");
        return RES_OK;
    }
    else
    {
        DEBUGE("!!!disk_status ERR\r\n");
        return RES_PARERR;
    }
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
        BYTE pdrv				/* Physical drive nmuber to identify the drive */
        )
{
    if(pdrv == SD_NAND)
    {
        DEBUGN("fatfs init hw \r\n");
        return RES_OK;
    }
    else
    {
        DEBUGE("!!!disk_initialize ERR\r\n");
        return RES_PARERR;
    }
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
        BYTE pdrv,		/* Physical drive nmuber to identify the drive */
        BYTE *buff,		/* Data buffer to store read data */
        DWORD sector,	/* Start sector in LBA */
        BYTE count		/* Number of sectors to read */
        )
{
  printf("disk Read\r\n");
	//DEBUGN("enter \r\n");
    DRESULT res;
//    DEBUGN("disk_read---sector:%d,count:%d\r\n",sector,count);
    if(pdrv == SD_NAND)
    { 
		//DEBUGN("sector: %d, count: %d \r\n", sector, count);
        enable_sdio_hw(true);

        en_result_t ret_sdio = SDCARD_ReadBlocks(&m_stcSdhandle, sector, count, (uint8_t *)buff, 20000);
        if(Ok != ret_sdio)
        {
//            DEBUGE("error to read sdio nand sector , sector = %d, count = %d, ret_sdio = %d\r\n", sector, count, ret_sdio);
            res = RES_ERROR;
        }
        else{
//            DEBUGN("read sdio nand data sector OK data start\r\n");
            res = RES_OK;
        }

		enable_sdio_hw(false);
		//DEBUGN("exit \r\n");
        return res;
    }
    else
    {
		DEBUGE("!!!disk_read ERR\r\n");
		//DEBUGN("exit \r\n");
        return RES_PARERR;
    }
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
        BYTE pdrv,			/* Physical drive nmuber to identify the drive */
        const BYTE *buff,	/* Data to be written */
        DWORD sector,		/* Start sector in LBA */
        BYTE count			/* Number of sectors to write */
        )
{
    DRESULT res;

    if(pdrv == SD_NAND)
    {
        enable_sdio_hw(true);

        en_result_t ret_sdio = SDCARD_WriteBlocks(&m_stcSdhandle, sector, count, (uint8_t *)buff, 20000);
        if(Ok != ret_sdio)
        {
//            DEBUGE("error to write sdio nand sector , sector = %d, count = %d, ret_sdio = %d\r\n", sector, count, ret_sdio);
            res = RES_ERROR;
        }
        else{
//            DEBUGN("read sdio nand data sector OK data start\r\n");
            res = RES_OK;
        }

        enable_sdio_hw(false);
        return res;
    }
    else
    {
        DEBUGE("!!!disk_write ERR\r\n");
        return RES_PARERR;
    }
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
        BYTE pdrv,		/* Physical drive nmuber (0..) */
        BYTE cmd,		/* Control code */
        void *buff		/* Buffer to send/receive control data */
        )
{
    if (pdrv == SD_NAND)
    {
        switch (cmd)
        {
        case CTRL_SYNC:
            printf("CTRL_SYNC\r\n");
            return RES_OK;

        case GET_SECTOR_COUNT:
            printf("GET_SECTOR_COUNT\r\n");
            *(DWORD * )buff = m_stcSdhandle.stcSdCardInfo.u32BlockNbr;
            return RES_OK;

        case GET_SECTOR_SIZE :
            printf("GET_SECTOR_SIZE\r\n");
            *(WORD * )buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE :
            printf("GET_BLOCK_SIZE\r\n");
            *(DWORD * )buff = 1;
            return RES_OK;

        default:
            return RES_PARERR;
        }
    }
    else
    {
//        DEBUGE("!!!disk_ioctl ERR\r\n");
        return RES_PARERR;
    }
}
DWORD get_fattime(void)
{
    DWORD time;

    return 0;
}
