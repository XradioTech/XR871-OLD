/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "fs/fatfs/diskio.h"		/* FatFs lower layer API */

#include "driver/sdmmc_diskio.h"
#include <stdio.h>	//for debug

/* Definitions of physical drive number for each drive */
#define DEV_RAM		1	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		0	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

#define SUPPORT_DEV_RAM 0
#define SUPPORT_DEV_USB 0

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
#if (SUPPORT_DEV_RAM)
	case DEV_RAM :
		result = RAM_disk_status();

		// translate the reslut code here

		return stat;
#endif

	case DEV_MMC :
		result = SDMMC_status();//MMC_disk_status();

		stat = result;// translate the reslut code here

		return stat;

#if (SUPPORT_DEV_USB)
	case DEV_USB :
		result = USB_disk_status();

		// translate the reslut code here

		return stat;
#endif
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
#if (SUPPORT_DEV_RAM)
	case DEV_RAM :
		result = RAM_disk_initialize();

		// translate the reslut code here

		return stat;
#endif

	case DEV_MMC :
		result = SDMMC_initialize();

		stat = result;// translate the reslut code here

		return stat;

#if (SUPPORT_DEV_USB)
	case DEV_USB :
		result = USB_disk_initialize();

		// translate the reslut code here

		return stat;
#endif
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
#if (SUPPORT_DEV_RAM)
	case DEV_RAM :
		// translate the arguments here

		result = RAM_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
#endif

	case DEV_MMC :
		// translate the arguments here

		result = SDMMC_read(buff, sector, count);

		res = result;// translate the reslut code here

		return res;

#if (SUPPORT_DEV_USB)
	case DEV_USB :
		// translate the arguments here

		result = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
#endif
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
#if (SUPPORT_DEV_RAM)
	case DEV_RAM :
		// translate the arguments here

		result = RAM_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
#endif

	case DEV_MMC :
		// translate the arguments here

		result = SDMMC_write(buff, sector, count);

		res = result;// translate the reslut code here

		return res;

#if (SUPPORT_DEV_USB)
	case DEV_USB :
		// translate the arguments here

		result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
#endif
	}

	return RES_PARERR;
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
	DRESULT res;
	int result;

	switch (pdrv) {
#if (SUPPORT_DEV_RAM)
	case DEV_RAM :

		// Process of the command for the RAM drive

		return res;
#endif

	case DEV_MMC :

		result = SDMMC_ioctl(cmd, buff);// Process of the command for the MMC/SD card

		res = result;

		return res;

#if (SUPPORT_DEV_USB)
	case DEV_USB :

		// Process of the command the USB drive

		return res;
#endif
	}

	return RES_PARERR;
}

