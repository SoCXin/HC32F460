/*-----------------------------------------------------------------------
/  Low level disk interface modlue include file  R0.07   (C)ChaN, 2009
/-----------------------------------------------------------------------*/

#ifndef _DISKIO

#define _READONLY	0u	/* 1: Read-only mode */
#define _USE_IOCTL	1u

#include "integer.h"

/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0u,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;


/*---------------------------------------*/
/* Prototypes for disk control functions */

BOOL assign_drives (int argc, char *argv[]);
DSTATUS disk_initialize (BYTE drv);
DSTATUS disk_status (BYTE drv);
DRESULT disk_read (BYTE drv, BYTE *buff, DWORD sector, BYTE count);
#if	_READONLY == 0
DRESULT disk_write (BYTE drv, const BYTE *buff, DWORD sector, BYTE count);
#endif
DRESULT disk_ioctl (BYTE drv, BYTE ctrl, void *buff);



/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT		0x01u	/* Drive not initialized */
#define STA_NODISK		0x02u	/* No medium in the drive */
#define STA_PROTECT		0x04u	/* Write protected */


/* Command code for disk_ioctrl() */

/* Generic command */
#define CTRL_SYNC		0u	/* Mandatory for write functions */
#define GET_SECTOR_COUNT	1u	/* Mandatory for only f_mkfs() */
#define GET_SECTOR_SIZE		2u
#define GET_BLOCK_SIZE		3u	/* Mandatory for only f_mkfs() */
#define CTRL_POWER		4u
#define CTRL_LOCK		5u
#define CTRL_EJECT		6u
/* MMC/SDC command */
#define MMC_GET_TYPE		10u
#define MMC_GET_CSD		11u
#define MMC_GET_CID		12u
#define MMC_GET_OCR		13u
#define MMC_GET_SDSTAT		14u
/* ATA/CF command */
#define ATA_GET_REV		20u
#define ATA_GET_MODEL		21u
#define ATA_GET_SN		22u

#define _DISKIO
#endif
