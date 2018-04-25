/**
  ******************************************************************************
  * @file
  * @author
  * @version
  * @date
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdmmc_diskio.h"
#include "driver/chip/sdmmc/hal_sdhost.h"
#include "driver/chip/sdmmc/sdmmc.h"


//#include "ff_gen_drv.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define _USE_WRITE	0
#define _USE_IOCTL	0

/* Block Size in Bytes */
#define BLOCK_SIZE                512

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

/* Private functions ---------------------------------------------------------*/

#if 0
#define SDMMC_DEBUG(fmt, arg...) printf(fmt, ##arg)
#define SDMMC_ENTRY() printf("[%s entry]\n", __func__)
#else
#define SDMMC_DEBUG(fmt, arg...)
#define SDMMC_ENTRY()
#endif


/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SDMMC_initialize()
{
	struct mmc_card *card;

	SDMMC_ENTRY();

	Stat = STA_NOINIT;
	card = mmc_card_open(0);
	if (card != NULL) {
		if (mmc_card_present(card)) {
			Stat &= ~STA_NOINIT;
		}
	}
	mmc_card_close(0);
	return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SDMMC_status()
{
	SDMMC_ENTRY();
  	return 0;
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SDMMC_read(BYTE *buff, const DWORD sector, UINT count)
{
  SDMMC_ENTRY();
  DRESULT res = RES_ERROR;
  struct mmc_card *card;

  card = mmc_card_open(0);
  if (card != NULL) {
  	if (mmc_block_read(card, buff, sector, count) != 0) {
		SDMMC_DEBUG("sdmmc driver read failed\n");
 	} else
		res = RES_OK;
  }
  mmc_card_close(0);

  return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
//#if _USE_WRITE == 1
DRESULT SDMMC_write(const BYTE *buff, DWORD sector, UINT count)
{
	int32_t err;
	struct mmc_card *card;

	SDMMC_ENTRY();
	DRESULT res = RES_ERROR;

	card = mmc_card_open(0);
	if (card != NULL) {
		err = mmc_block_write(card, (uint8_t *)buff, sector, count);
		if (err != 0) {
			SDMMC_DEBUG("sdmmc driver write failed\n");
		} else
			res = RES_OK;
	}
	mmc_card_close(0);

	return res;
}
//#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
//#if _USE_IOCTL == 1
DRESULT SDMMC_ioctl(BYTE cmd, void *buff)
{
	SDMMC_ENTRY();
  DRESULT res = RES_ERROR;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
#if ((_USE_MKFS == 0) || (_FS_READONLY == 1))
    res = RES_OK;
#else
    res = RES_PARERR; /* not support now */
#endif
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    *(WORD*)buff = BLOCK_SIZE;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    *(DWORD*)buff = BLOCK_SIZE;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
//#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

