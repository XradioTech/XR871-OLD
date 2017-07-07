#ifndef _SDMMC_DISKIO_H_
#define _SDMMC_DISKIO_H_

#include "fs/fatfs/integer.h"
#include "fs/fatfs/ffconf.h"
#include "fs/fatfs/ff.h"
#include "fs/fatfs/diskio.h"

DSTATUS SDMMC_initialize();
DSTATUS SDMMC_status();
DRESULT SDMMC_read(BYTE *buff, DWORD sector, UINT count);
DRESULT SDMMC_write(const BYTE *buff, DWORD sector, UINT count);
DRESULT SDMMC_ioctl(BYTE cmd, void *buff);

#endif /* _SDMMC_DISKIO_H_ */

