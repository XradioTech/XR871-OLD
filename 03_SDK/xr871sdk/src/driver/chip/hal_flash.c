/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdio.h>
#include "sys/list.h"
#include "sys/param.h"
#include "sys/defs.h"

#include "driver/chip/hal_spi.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_flash.h"
#include "hal_base.h"

#define FD_DEBUG(msg, arg...) XR_DEBUG((DBG_OFF | XR_LEVEL_ALL), NOEXPAND, "[Flash Driver debug] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define FD_ERROR(msg, arg...) XR_ERROR((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash Driver error] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)

#define FLASH_DMA_TRANSFER_MIN_SIZE (64)

static FlashBoardCfg *getFlashBoardCfg(int minor);


/*
	SpiFlashDriver
*/
typedef struct SpiFlashDriver
{
	FlashDrvierBase base;
	SPI_Port port;
	SPI_CS cs;
	SPI_Config config;
} SpiFlashDriver;

void HAL_XIP_Delay(unsigned int us);


HAL_Status spiFlashOpen(FlashDrvierBase *base)
{
	/*TODO: it should be suspend schedule and deinit xip when spi is using spi0 with xiping*/

	return HAL_OK;
}

HAL_Status spiFlashClose(FlashDrvierBase *base)
{
	/*TODO: it should be resume schedule and init xip when spi is using spi0 with xiping*/

	return HAL_OK;
}

#define FD_SPI_WRITE(impl, ins, line_max)	\
	do {	\
		ret = insToSpi(impl, ins, line_max, HAL_SPI_Transmit);	\
		if (ret != HAL_OK) { \
			FD_ERROR("spi instruction param error"); \
			goto failed; \
		} \
	} while (0)

#define FD_SPI_READ(impl, ins, line_max)	\
	do { \
		ret = insToSpi(impl, ins, line_max, HAL_SPI_Receive);	\
		if (ret != HAL_OK) { \
			FD_ERROR("spi instruction param error"); \
			goto failed; \
		} \
	} while (0)

static HAL_Status insToSpi(SpiFlashDriver *impl, InstructionField *ins, uint32_t line_max, HAL_Status (*fun)(SPI_Port, uint8_t *, uint32_t))
{
	uint8_t *p;

	if (ins)
	{
		if (ins->line > line_max)
			return HAL_INVALID;
		if (!ins->pdata && ins->len > 4)
			return HAL_ERROR;

		p = ins->pdata ? ins->pdata : (uint8_t *)&ins->data;
		return fun(impl->port, p, ins->len);
	}
	else
		return HAL_OK;
}

static HAL_Status spiFlashWrite(FlashDrvierBase *base, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	SpiFlashDriver *impl = __containerof(base, SpiFlashDriver, base);
	InstructionField naddr = {0};
	HAL_Status ret;

	if (data && data->len >= (base->sizeToDma - 1))
		impl->config.opMode = SPI_OPERATION_MODE_DMA;
	else
		impl->config.opMode = SPI_OPERATION_MODE_POLL;

	ret = HAL_SPI_Open(impl->port, impl->cs, &impl->config, 5000);
	if (ret != HAL_OK) {
		FD_ERROR("spi open failed");
		return ret;
	}

	HAL_SPI_CS(impl->port, 1);
	FD_SPI_WRITE(impl, cmd, 1);
	if (addr)
	{
		naddr.len = addr->len;
		naddr.line = addr->line;
		if (!addr->pdata)
			addr->pdata = (uint8_t *)&addr->data;
		naddr.data = ((addr->pdata)[2]) | ((addr->pdata)[1] << 8) | ((addr->pdata)[0] << 16);
		FD_DEBUG("naddr.data: 0x%x", naddr.data);
		FD_SPI_WRITE(impl, &naddr, 1);
	}
	FD_SPI_WRITE(impl, dummy, 1);
	FD_SPI_WRITE(impl, data, 1);
	HAL_SPI_CS(impl->port, 0);

failed:
	HAL_SPI_Close(impl->port);
	return ret;
}

static HAL_Status spiFlashRead(FlashDrvierBase *base, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	SpiFlashDriver *impl = __containerof(base, SpiFlashDriver, base);
	InstructionField naddr = {0};
	HAL_Status ret;

	if (data && data->len >= (base->sizeToDma - 1))
		impl->config.opMode = SPI_OPERATION_MODE_DMA;
	else
		impl->config.opMode = SPI_OPERATION_MODE_POLL;

	ret = HAL_SPI_Open(impl->port, impl->cs, &impl->config, 5000);
	if (ret != HAL_OK) {
		FD_ERROR("spi open failed");
		return ret;
	}

	if (data && data->line == 2)
		HAL_SPI_Config(impl->port, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_DUAL_RX);
	else
		HAL_SPI_Config(impl->port, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);

	HAL_SPI_CS(impl->port, 1);
	FD_SPI_WRITE(impl, cmd, 1);
	if (addr)
	{
		naddr.len = addr->len;
		naddr.line = addr->line;
		if (!addr->pdata)
			addr->pdata = (uint8_t *)&addr->data;
		naddr.data = ((addr->pdata)[2]) | ((addr->pdata)[1] << 8) | ((addr->pdata)[0] << 16);
		FD_DEBUG("naddr.data: 0x%x", naddr.data);
		FD_SPI_WRITE(impl, &naddr, 1);
	}
	FD_SPI_WRITE(impl, dummy, 1);
	FD_SPI_READ(impl, data, 2);
	HAL_SPI_CS(impl->port, 0);

failed:
	HAL_SPI_Close(impl->port);
	return ret;
}

static HAL_Status spiFlashSetFreq(FlashDrvierBase *base, uint32_t freq)
{
	/* TODO: tbc... */
	return HAL_INVALID;
}

void spiFlashMsleep(FlashDrvierBase *base, uint32_t ms)
{
	HAL_MSleep(ms);
}

void spiFlashMsleepReuseFlashc(FlashDrvierBase *base, uint32_t ms)
{
	HAL_XIP_Delay(ms * 1000);
}

static void spiFlashDestroy(FlashDrvierBase *base)
{
	SpiFlashDriver *impl = __containerof(base, SpiFlashDriver, base);
	HAL_Free(impl);
}

FlashDrvierBase *spiDriverCreate(int dev, FlashBoardCfg *bcfg)
{
	/*TODO: check read mode*/

	SpiFlashDriver *impl = HAL_Malloc(sizeof(SpiFlashDriver));
	HAL_Memset(impl, 0, sizeof(*impl));

	impl->port = bcfg->spi.port;
	impl->cs = bcfg->spi.cs;
	impl->config.sclk = bcfg->spi.clk;
	impl->config.firstBit = SPI_TCTRL_FBS_MSB;
	impl->config.mode = SPI_CTRL_MODE_MASTER;
	impl->config.opMode = SPI_OPERATION_MODE_DMA;	/*spi0 must be poll on xip;*/
	impl->config.sclkMode = SPI_SCLK_Mode0;

	FD_DEBUG("bcfg->type: %d; impl->port: %d; impl->cs: %d; impl->config.sclk: %d;",
		bcfg->type, impl->port, impl->cs, impl->config.sclk);

	impl->base.dev = dev;
	impl->base.open = spiFlashOpen;
	impl->base.close = spiFlashClose;
	impl->base.read = spiFlashRead;
	impl->base.write = spiFlashWrite;
	impl->base.setFreq = spiFlashSetFreq;
	impl->base.destroy = spiFlashDestroy;
	impl->base.sizeToDma = FLASH_DMA_TRANSFER_MIN_SIZE;
	if (bcfg->spi.port == SPI1)
		impl->base.msleep = spiFlashMsleep;
	else
		impl->base.msleep = spiFlashMsleepReuseFlashc;

	return &impl->base;
}

/*
	FlashcFlashDriver
*/
typedef struct FlashcFlashDriver
{
	FlashDrvierBase base;

} FlashcFlashDriver;

HAL_Status flashcFlashOpen(FlashDrvierBase *base)
{
	FD_DEBUG("open");
	return HAL_Flashc_Open();
}

HAL_Status flashcFlashClose(FlashDrvierBase *base)
{
	HAL_Status ret = HAL_Flashc_Close();
	FD_DEBUG("close");
	return ret;
}

void insToFcIns(InstructionField *ins, FC_InstructionField *fcins)
{
	if (ins == NULL)
		return;

	if (ins->pdata)
		fcins->pdata = ins->pdata;
	else
		fcins->pdata = (uint8_t *)&ins->data;
	fcins->len = ins->len;
	fcins->line = (FC_CycleBits)((ins->line == 4) ? FC_CYCLEBITS_4 : ins->line);
}

#define INS_CMD  (0)
#define INS_ADDR (1)
#define INS_DUM  (2)
#define INS_DATA (3)

static HAL_Status flashcFlashWrite(FlashDrvierBase *base, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	int dma = 0;
	FC_InstructionField tmp[4];
	HAL_Memset(tmp, 0, sizeof(tmp));

	insToFcIns(cmd, &tmp[INS_CMD]);
	insToFcIns(addr, &tmp[INS_ADDR]);
	insToFcIns(dummy, &tmp[INS_DUM]);
	insToFcIns(data, &tmp[INS_DATA]);

	if (tmp[INS_DATA].len >= (base->sizeToDma - 1) &&  tmp[INS_DATA].len < 128 * 1024)
		dma = 1;

	return HAL_Flashc_Write(&tmp[INS_CMD], &tmp[INS_ADDR], &tmp[INS_DUM], &tmp[INS_DATA], dma);
}

static HAL_Status flashcFlashRead(FlashDrvierBase *base, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	int dma = 0;
	FC_InstructionField tmp[4];
	HAL_Memset(tmp, 0, sizeof(tmp));

	insToFcIns(cmd, &tmp[INS_CMD]);
	insToFcIns(addr, &tmp[INS_ADDR]);
	insToFcIns(dummy, &tmp[INS_DUM]);
	insToFcIns(data, &tmp[INS_DATA]);

	if (tmp[INS_DATA].len >= (base->sizeToDma - 1) &&  tmp[INS_DATA].len < 128 * 1024)
		dma = 1;

	return HAL_Flashc_Read(&tmp[INS_CMD], &tmp[INS_ADDR], &tmp[INS_DUM], &tmp[INS_DATA], dma);
}

static HAL_Status flashcFlashSetFreq(FlashDrvierBase *base, uint32_t freq)
{
	/* TODO: tbc... */
	return HAL_INVALID;
}

void flashcFlashMsleep(FlashDrvierBase *base, uint32_t ms)
{
	HAL_XIP_Delay(ms * 1000);
}

static void flashcFlashDestroy(FlashDrvierBase *base)
{
	FlashcFlashDriver *impl = __containerof(base, FlashcFlashDriver, base);
	HAL_Flashc_Deinit();
	HAL_Free(impl);
}

static FlashDrvierBase *flashcDriverCreate(int dev, FlashBoardCfg *bcfg)
{
	FlashcFlashDriver *impl = HAL_Malloc(sizeof(FlashcFlashDriver));
	if (impl == NULL)
		FD_ERROR("malloc failed");
	HAL_Memset(impl, 0, sizeof(*impl));

	Flashc_Config cfg;
	cfg.freq = bcfg->flashc.clk;
	HAL_Flashc_Init(&cfg);

	impl->base.dev = dev;
	impl->base.open = flashcFlashOpen;
	impl->base.close = flashcFlashClose;
	impl->base.read = flashcFlashRead;
	impl->base.write = flashcFlashWrite;
	impl->base.setFreq = flashcFlashSetFreq;
	impl->base.msleep = flashcFlashMsleep;
	impl->base.destroy = flashcFlashDestroy;
	impl->base.sizeToDma = FLASH_DMA_TRANSFER_MIN_SIZE;

	return &impl->base;
}


/*
	FlashDriverCtor
*/
static FlashDrvierBase *flashDriverCreate(int minor)
{
	FlashDrvierBase *base = NULL;
	FlashBoardCfg *cfg;
	int dev = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, minor);

	HAL_BoardIoctl(HAL_BIR_GET_CFG, dev, (uint32_t)&cfg);
	if (cfg == NULL)
		FD_ERROR("flash config error");

	if (cfg->type == FLASH_CONTROLLER)
		base = flashcDriverCreate(dev, cfg);
	else
		base = spiDriverCreate(dev, cfg);

	if (base == NULL)
		FD_ERROR("flash driver can't be create");

	/*manage driver gpio. tbc...*/

	return base;
}

static int flashDriverDestory(FlashDrvierBase *base)
{
	if (base == NULL)
		return -1;

	base->destroy(base);
	return 0;
}


/*
	FlashBoard
*/
static FlashBoardCfg *getFlashBoardCfg(int minor)
{
	FlashBoardCfg *cfg;
	int dev = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, minor);

	HAL_BoardIoctl(HAL_BIR_GET_CFG, dev, (uint32_t)&cfg);
	if (cfg == NULL)
		FD_ERROR("flash config error");

	return cfg;
}


/*
	Flash
*/
struct FlashDev
{
	struct list_head node;
	HAL_Mutex lock;
	uint32_t usercnt; /*not thread safe*/

	uint32_t flash;
	FlashDrvierBase *drv;
	FlashChipBase *chip;
	FlashReadMode rmode;
	FlashPageProgramMode wmode;
};

struct list_head flashNodeHead = {
		.next = &flashNodeHead,
		.prev = &flashNodeHead
};

FlashChipBase *getFlashChip(FlashDev *dev)
{
	return dev->chip;
}

FlashReadMode getFlashMode(FlashDev *dev)
{
	return dev->rmode;
}

FlashDev *getFlashDev(uint32_t flash)
{
	FlashDev *dev = NULL;
	FlashDev *itor = NULL;

	list_for_each_entry(itor, &flashNodeHead, node)
	{
		if (itor->flash == flash) {
			dev = itor;
			break;
		}
	}

	if (dev == NULL)
		FD_ERROR("no this flash or not inited");

	return dev;
}

static int deleteFlashDev(FlashDev *dev)
{
	if (dev == NULL) {
		FD_ERROR("NULL flash device");
		return -1;
	}

	list_del(&dev->node);

	return 0;
}

static int addFlashDev(FlashDev *dev)
{
	if (dev == NULL) {
		FD_ERROR("NULL flash device");
		return -1;
	}

	list_add_tail(&dev->node, &flashNodeHead);

	return 0;
}


HAL_Status HAL_Flash_Init		(uint32_t flash)
{
	HAL_Status ret;
	FlashDrvierBase *drv = NULL;
	FlashChipBase *chip = NULL;
	FlashDev *dev = NULL;
	FlashBoardCfg *cfg = NULL;

	cfg = getFlashBoardCfg(flash);
	if (cfg == NULL)
		goto failed;

	drv = flashDriverCreate(flash);
	if (drv == NULL)
		goto failed;
	FD_DEBUG("drv created");

	chip = FlashChipCreate(drv);
	if (chip == NULL)
		goto failed;
	FD_DEBUG("chip created");

	dev = HAL_Malloc(sizeof(*dev));
	if (dev == NULL)
		goto failed;
	HAL_Memset(dev, 0, sizeof(*dev));

	/*TODO: get read mode from board, and init*/

	dev->chip = chip;
	dev->drv = drv;
	dev->flash = flash;
	dev->rmode = cfg->mode;
	dev->wmode = FLASH_PAGEPROGRAM;
	dev->usercnt = 0;
	INIT_LIST_HEAD(&dev->node);
	ret = HAL_MutexInit(&dev->lock);
	if (ret != HAL_OK) {
		FD_ERROR("mutex init failed: %d", ret);
		goto failed;
	}

	addFlashDev(dev);

	return HAL_OK;

failed:
	FD_ERROR("flash init failed");

	if (drv != NULL)
		flashDriverDestory(drv);
	if (dev != NULL)
		HAL_Free(dev);

	return HAL_ERROR;
}

HAL_Status HAL_Flash_Deinit		(uint32_t flash)
{
	FlashDev *dev = getFlashDev(flash);

	/*not thread safe*/
	if (dev->usercnt != 0)
		return HAL_TIMEOUT;

	deleteFlashDev(dev);
	HAL_MutexDeinit(&dev->lock);
	if (dev->drv != NULL)
		flashDriverDestory(dev->drv);
	if (dev != NULL)
		HAL_Free(dev);

	return HAL_OK;
}

HAL_Status HAL_Flash_Open		(uint32_t flash, uint32_t timeout_ms)
{
	FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;

	ret = HAL_MutexLock(&dev->lock, timeout_ms);
	if (ret == HAL_OK)
		dev->usercnt++;

	return ret;
}

HAL_Status HAL_Flash_Close		(uint32_t flash)
{
	FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;

	ret = HAL_MutexUnlock(&dev->lock);
	if (ret == HAL_OK)
		dev->usercnt--;

	return ret;
}

static HAL_Status HAL_Flash_WaitCompl(FlashDev *dev, int32_t timeout_ms)
{
	while (dev->chip->isBusy(dev->chip) > 0)
	{
		dev->drv->msleep(dev->drv, 20);
		timeout_ms -= 20;
		if (timeout_ms <= 0)
			return HAL_TIMEOUT;
	}
	return HAL_OK;
}

HAL_Status HAL_Flash_Control	(uint32_t flash, FlashControlCmd attr, uint32_t arg)
{
	/*TODO: tbc...*/

	/*TODO: 1.return min erase size */

	return HAL_INVALID;
}

HAL_Status HAL_Flash_Overwrite	(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
	FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint8_t *buf = NULL;
	uint8_t *ptr = data;
	uint32_t paddr = addr;
	int32_t  left = (int32_t)size;
	uint32_t pp_size;
	uint32_t saddr;

	if (!(dev->chip->mEraseSizeSupport & FLASH_ERASE_4KB))
		return HAL_INVALID;

	buf = HAL_Malloc(FLASH_ERASE_4KB);
	if (buf != NULL)
		goto out;

	while (left > 0)
	{
		HAL_Flash_MemoryOf(flash, FLASH_ERASE_4KB, paddr, &saddr);
		HAL_Flash_Read(flash, saddr, buf, FLASH_ERASE_4KB);
		ret = HAL_Flash_Erase(flash, FLASH_ERASE_4KB, saddr, 1);
		if (ret != HAL_OK)
			return ret;

		pp_size = MIN(left, FLASH_ERASE_4KB - (paddr - saddr));
		HAL_Memcpy(buf + (paddr - saddr), ptr, pp_size);

		ret = HAL_Flash_Write(flash, saddr, buf, FLASH_ERASE_4KB);
		if (ret != HAL_OK)
			goto out;

		ptr += pp_size;
		paddr += pp_size;
		left -= pp_size;
	}

out:
	if (buf != NULL)
		HAL_Free(buf);

	return ret;
}

HAL_Status HAL_Flash_Write		(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
	FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint32_t address = addr;
	uint32_t left = size;
	uint8_t *ptr = data;
	uint32_t pp_size;

	FD_DEBUG("%d: w%d, a: 0x%x", flash, size, addr);

	if (dev == NULL)
		return HAL_INVALID;
	if (dev->chip->pageProgram == NULL)
		return HAL_INVALID;

	dev->drv->open(dev->drv);

	while (left > 0)
	{
		pp_size = MIN(left, dev->chip->mPageSize - (address % dev->chip->mPageSize));

		dev->chip->writeEnable(dev->chip);
		FD_DEBUG("WE");
		ret = dev->chip->pageProgram(dev->chip, dev->wmode, address, ptr, pp_size);
		FD_DEBUG("PP");
		dev->chip->writeDisable(dev->chip);
		FD_DEBUG("WD");

		if (ret < 0)
			break;

		HAL_Flash_WaitCompl(dev, 5000);

		address += pp_size;
		ptr += pp_size;
		left -= pp_size;
	}

	dev->drv->close(dev->drv);

	return ret;
}

HAL_Status HAL_Flash_Read		(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
	FlashDev *dev = getFlashDev(flash);
	HAL_Status ret;

	FD_DEBUG("%d: r%d, a: 0x%x", flash, size, addr);

	if (dev == NULL)
		return HAL_INVALID;
	if (dev->chip->read == NULL)
		return HAL_INVALID;

	dev->drv->open(dev->drv);
	ret = dev->chip->read(dev->chip, dev->rmode, addr, data, size);
	dev->drv->close(dev->drv);

	return ret;
}

HAL_Status HAL_Flash_Erase		(uint32_t flash, FlashEraseMode blk_size, uint32_t addr, uint32_t blk_cnt)
{
	FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint32_t esize = blk_size;
	uint32_t eaddr = addr;

	FD_DEBUG("%d: e%d * %d, a: 0x%x", flash, (uint32_t)blk_size, blk_cnt, addr);

	if ((addr + blk_size * blk_cnt) > dev->chip->mSize) {
		FD_ERROR("erase memory is over flash memory\n");
		return HAL_INVALID;
	}
	if ((blk_size == FLASH_ERASE_CHIP) && (blk_cnt != 1))
		FD_DEBUG("chip erase will be execute more than 1");
	if (addr % blk_size)
		FD_DEBUG("chip erase on a incompatible address");

	dev->drv->open(dev->drv);

	while (blk_cnt-- > 0)
	{
		dev->chip->writeEnable(dev->chip);
		ret = dev->chip->erase(dev->chip, blk_size, eaddr);
		dev->chip->writeDisable(dev->chip);

		if (ret < 0) {
			FD_ERROR("erase failed: %d", ret);
			break;
		}

		HAL_Flash_WaitCompl(dev, 5000);

		eaddr += esize;
	}

	dev->drv->close(dev->drv);

	return ret;
}

HAL_Status HAL_Flash_MemoryOf	(uint32_t flash, FlashEraseMode size, uint32_t addr, uint32_t *start)
{
	FlashDev *dev = getFlashDev(flash);
	uint32_t page_mask;
	HAL_Status ret = HAL_OK;

	if (!(size & dev->chip->mEraseSizeSupport))
		return HAL_INVALID;

	page_mask = ~((uint32_t)(size - 1));
	*start = addr & page_mask;

	return ret;
}

int HAL_Flash_Check	(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
#define FLASH_CHECK_BUF_SIZE (128)

	uint8_t *pdata = data;
	uint8_t *pbuf;
	uint8_t *buf;
	uint8_t src;
	uint8_t dst;
	uint32_t left = size;
	uint32_t paddr = addr;
	int32_t ret = 0;

	buf = HAL_Malloc(FLASH_CHECK_BUF_SIZE);
	pbuf = buf + FLASH_CHECK_BUF_SIZE;
	if (buf == NULL)
		return -1;

	while (left > 0)
	{
		if ((pbuf - buf) == FLASH_CHECK_BUF_SIZE) {
			HAL_Flash_Read(flash, paddr, buf, FLASH_CHECK_BUF_SIZE);
			pbuf = buf;
		}

		src = *pbuf++;
		dst = *pdata++;
		left--;

		dst ^= src;
		if (dst == 0)
			continue; /* src == dst */

		ret = 1; /* src != dst */
		if (dst & src) {
			ret = 2; /* src has bit '1', need to erase */
			break;
		}
	}

	if (buf != NULL)
		HAL_Free(buf);

	return ret;
}

