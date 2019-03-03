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

#include "driver/chip/flashchip/flash_chip.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_xip.h"
#include "../hal_base.h"
#include "flash_default.h"
#include "sys/xr_debug.h"

#ifndef __CONFIG_BOOTLOADER
#define FLASH_DBG_ON	DBG_OFF
#define FLASH_ALE_ON	DBG_ON
#define FLASH_ERR_ON	DBG_ON
#define FLASH_NWA_ON	DBG_ON
#else
#define FLASH_DBG_ON	DBG_OFF
#define FLASH_ALE_ON	DBG_OFF
#define FLASH_ERR_ON	DBG_OFF
#define FLASH_NWA_ON	DBG_OFF
#endif

#define FLASH_DEBUG(fmt, arg...)	XR_DEBUG((FLASH_DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash chip D] <%s:%d> " fmt "\n", __func__, __LINE__, ##arg)
#define FLASH_ALERT(fmt, arg...)	XR_ALERT((FLASH_ALE_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash chip A] <%s:%d> " fmt "\n", __func__, __LINE__, ##arg)
#define FLASH_ERROR(fmt, arg...)	XR_ERROR((FLASH_ERR_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash chip E] <%s:%d> " fmt "\n", __func__, __LINE__, ##arg)
#define FLASH_NOWAY()			XR_ERROR((FLASH_NWA_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash chip N] <%s:%d> \n", __func__, __LINE__)
#define FLASH_NOTSUPPORT() 		FLASH_ALERT("not support CMD")

typedef enum {
	FLASH_INSTRUCTION_WREN = 0x06,				/* write enable */
	FLASH_INSTRUCTION_WRDI = 0x04,				/* write disable */
	FLASH_INSTRUCTION_RDID = 0x9F,				/* jedec id */
	FLASH_INSTRUCTION_RDSR1 = 0x05,				/* read status register-1 */
	FLASH_INSTRUCTION_WRSR1 = 0x01,				/* write status register-1 */
	FLASH_INSTRUCTION_READ = 0x03,				/* read data */
	FLASH_INSTRUCTION_FAST_READ = 0x0B,			/* fast read */
	FLASH_INSTRUCTION_PP = 0x02,					/* page program */
	FLASH_INSTRUCTION_ERASE_64KB = 0xD8,			/* erase block(sector) 64k */
	FLASH_INSTRUCTION_ERASE_32KB = 0x52,			/* erase block(sector) 32k */
	FLASH_INSTRUCTION_ERASE_4KB = 0x20,			/* erase sector 4k */
	FLASH_INSTRUCTION_ERASE_CHIP = 0xC7,			/* chip erase */
	FLASH_INSTRUCTION_WRSR = 0X01,				/* write status register */
	FLASH_INSTRUCTION_FAST_READ_DO = 0x3B,		/* fast read dual output */
	FLASH_INSTRUCTION_RDSR2 = 0x35,
 	FLASH_INSTRUCTION_RDSR3 = 0x15,
	FLASH_INSTRUCTION_WRSR2 = 0x31,
	FLASH_INSTRUCTION_WRSR3 = 0x11,
	FLASH_INSTRUCTION_SRWREN = 0x50,
	FLASH_INSTRUCTION_CE = 0x60,
	FLASH_INSTRUCTION_EPSP = 0x75,
	FLASH_INSTRUCTION_EPRS = 0x7A,
	FLASH_INSTRUCTION_PWDN = 0xB9,
	FLASH_INSTRUCTION_REL = 0xAB,
	FLASH_INSTRUCTION_FAST_READ_DIO = 0xBB,
	FLASH_INSTRUCTION_FAST_READ_QO = 0x6B,
	FLASH_INSTRUCTION_FAST_READ_QIO = 0xEB,
	FLASH_INSTRUCTION_EN_QPI = 0x38,
	FLASH_INSTRUCTION_DIS_QPI = 0xFF,
	FLASH_INSTRUCTION_RSEN = 0x66,
	FLASH_INSTRUCTION_RESET = 0x99,
	FLASH_INSTRUCTION_QPP = 0x32,
	FLASH_INSTRUCTION_SRP = 0xC0,
} eSF_Instruction;

#ifdef FLASH_DEFAULTCHIP
extern FlashChipCtor DefaultFlashChip;
#endif
#ifdef FLASH_XT25F16B
extern FlashChipCtor  XT25F16B_FlashChip;
#endif
#ifdef FLASH_P25Q80H
extern FlashChipCtor  P25Q80H_FlashChip;
#endif
#ifdef FLASH_P25Q16H
extern FlashChipCtor  P25Q16H_FlashChip;
#endif
#ifdef FLASH_EN25QH64A
extern FlashChipCtor  EN25QH64A_FlashChip;
#endif
#ifdef FLASH_XM25QH64A
extern FlashChipCtor  XM25QH64A_FlashChip;
#endif

FlashChipCtor *flashChipList[] = {
#ifdef FLASH_DEFAULTCHIP
		&DefaultFlashChip, /*default chip must be at the first*/
#endif
#ifdef FLASH_XT25F16B
		&XT25F16B_FlashChip,
#endif
#ifdef FLASH_P25Q80H
		&P25Q80H_FlashChip,
#endif
#ifdef FLASH_P25Q16H
		&P25Q16H_FlashChip,
#endif
#ifdef FLASH_EN25QH64A
		&EN25QH64A_FlashChip,
#endif
#ifdef FLASH_XM25QH64A
		&XM25QH64A_FlashChip,
#endif
};

/* internal macros for flash chip instruction */
#define FCI_CMD(idx)    instruction[idx]
#define FCI_ADDR(idx)   instruction[idx]
#define FCI_DUMMY(idx)  instruction[idx]
#define FCI_DATA(idx)   instruction[idx]

static uint32_t getJedecID(FlashDrvierBase *driver)
{
	int ret;
	PCHECK(driver);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));
	FCI_CMD(0).data = FLASH_INSTRUCTION_RDID;
	FCI_CMD(0).len = 1;
	FCI_CMD(0).line = 1;
	FCI_DATA(1).pdata = (uint8_t *)&FCI_DATA(1).data;
	FCI_DATA(1).line = 1;
	FCI_DATA(1).len = 3;

	driver->open(driver);
	ret = driver->read(driver, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));
	if (ret != HAL_OK)
		FLASH_ERROR("driver go some wrong: %d", ret);
	driver->close(driver);

	return FCI_DATA(1).data;
}

FlashChipBase *FlashChipCreate(FlashDrvierBase *driver)
{
	uint32_t list_size = sizeof(flashChipList) / sizeof(flashChipList[0]);
	uint32_t jedec = getJedecID(driver);
	FlashChipBase *base = NULL;
	FlashChipCtor *ctor = NULL;

	FLASH_DEBUG("jedec: 0x%x, list_size: %d", jedec, list_size);

	while (list_size--)
	{
		ctor = flashChipList[list_size];
		if (ctor->mJedecId == jedec)
			break;
	}

	if (ctor == NULL)
		return NULL;
	base = ctor->create(jedec);
/*
	base->writeEnable = defaultWriteEnable;
	base->writeDisable = defaultWriteDisable;
	base->readStatus = defaultReadStatus;
	base->erase = defaultErase;
	base->jedecID = defaultGetJedecID;
	base->pageProgram = defaultPageProgram;
	base->read = defaultRead;

	base->driverWrite = defaultDriverWrite;
	base->driverRead = defaultdriverRead;
	base->setFreq = defaultSetFreq;
	base->switchReadMode = defaultSwitchReadMode;
	base->enableXIP = defaultEnableXIP;
	base->disableXIP = defaultDisableXIP;
	base->isBusy = defaultIsBusy;
	base->control = defaultControl;
	base->minEraseSize = defaultGetMinEraseSize;

	base->writeStatus = NULL;
	base->suspendErasePageprogram = NULL;
	base->resumeErasePageprogram = NULL;
	base->powerDown = NULL;
	base->releasePowerDown = NULL;
	base->enableQPIMode = NULL;
	base->disableQPIMode = NULL;
	base->enableReset = NULL;
	base->reset = NULL;
	base->uniqueID = NULL;
*/
	ctor->init(base);
	base->mDriver = driver;

	return base;
}


/*
	Default Flash Chip Interface
*/
typedef struct {
	uint8_t SRP0: 1;
	uint8_t SEC: 1;
	uint8_t TB: 1;
	uint8_t BP2: 1;
	uint8_t BP1: 1;
	uint8_t BP0: 1;
	uint8_t WEL: 1;
	uint8_t BUSY: 1;
} DefaultFlash_StatusRegister1;	//this can be different in different flash chip.

typedef struct {
	union {
		struct {
			uint8_t SRP1: 1;
			uint8_t QE: 1;
			uint8_t RESERVED: 1;
			uint8_t LB1: 1;
			uint8_t LB2: 1;
			uint8_t LB3: 1;
			uint8_t CMP: 1;
			uint8_t SUS: 1;
		};
		uint8_t status;
	};
} DefaultFlash_StatusRegister2;	//this can be different in different flash chip.


void defaultWriteEnable(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));

	cmd.data = FLASH_INSTRUCTION_WREN;
	cmd.line = 1;

	base->driverWrite(base, &cmd, NULL, NULL, NULL);
}

void defaultWriteDisable(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));

	cmd.data = FLASH_INSTRUCTION_WRDI;
	cmd.line = 1;

	base->driverWrite(base, &cmd, NULL, NULL, NULL);
}

int defaultReadStatus(FlashChipBase *base, FlashStatus reg, uint8_t *status)
{
	PCHECK(base);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (!(reg & base->mReadStausSupport)) {
		FLASH_NOTSUPPORT();
		return -1;
	}

	if (reg == FLASH_STATUS1)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_RDSR1;
	}
	else if (reg == FLASH_STATUS2)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_RDSR2;
	}
	else if (reg == FLASH_STATUS3)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_RDSR3;
	}
	else
	{
		FLASH_NOWAY();
	}

	FCI_DATA(1).pdata = (uint8_t *)status;
	FCI_DATA(1).len = 1;
	FCI_DATA(1).line = 1;

	return base->driverRead(base, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));
}

int defaultWriteStatus(FlashChipBase *base, FlashStatus reg, uint8_t *status)
{
	PCHECK(base);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (!(reg & base->mWriteStatusSupport)) {
		FLASH_NOTSUPPORT();
		return HAL_INVALID;
	}

	FCI_CMD(0).data = FLASH_INSTRUCTION_SRWREN;
	FCI_CMD(0).line = 1;

	base->driverWrite(base, &FCI_CMD(0), NULL, NULL, NULL);

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (reg == FLASH_STATUS1)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_WRSR1;
	}
	else if (reg == FLASH_STATUS2)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_WRSR2;
	}
	else if (reg == FLASH_STATUS3)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_WRSR3;
	}
	else
	{
		FLASH_NOWAY();
	}

	FCI_DATA(1).pdata = (uint8_t *)status;
	FCI_DATA(1).len = 1;
	FCI_DATA(1).line = 1;

	return base->driverWrite(base, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));
}

int defaultErase(FlashChipBase *base, FlashEraseMode mode, uint32_t eaddr)
{
	PCHECK(base);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (!(mode & base->mEraseSizeSupport)) {
		FLASH_NOTSUPPORT();
		return HAL_INVALID;
	}

	FLASH_DEBUG("mode: 0x%x; base->mEraseSizeSupport: 0x%x", mode, base->mEraseSizeSupport);

	if (mode == FLASH_ERASE_CHIP)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_ERASE_CHIP;
		base->driverWrite(base, &FCI_CMD(0), NULL, NULL, NULL);
		return 0;
	}
	else if (mode == FLASH_ERASE_32KB)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_ERASE_32KB;
	}
	else if (mode == FLASH_ERASE_64KB)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_ERASE_64KB;
	}
	else if (mode == FLASH_ERASE_4KB)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_ERASE_4KB;
	}
	else
	{
		FLASH_NOWAY();
	}

	FCI_ADDR(1).data = eaddr;
	FCI_ADDR(1).line = 1;

	return base->driverWrite(base, &FCI_CMD(0), &FCI_ADDR(1), NULL, NULL);
}

int defaultSuspendErasePageprogram(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_EPSP;
	return base->driverWrite(base, &cmd, NULL, NULL, NULL);
}

int defaultResumeErasePageprogram(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_EPRS;
	return base->driverWrite(base, &cmd, NULL, NULL, NULL);
}

int defaultPowerDown(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_PWDN;
	return base->driverWrite(base, &cmd, NULL, NULL, NULL);
}

int defaultReleasePowerDown(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField instruction[3];

	HAL_Memset(&instruction, 0, sizeof(instruction));
	FCI_CMD(0).data = FLASH_INSTRUCTION_REL;
	FCI_DUMMY(1).len = 3;
	FCI_DUMMY(1).line = 1;
	FCI_DATA(2).len = 1;
	FCI_DATA(2).line = 1;

	return base->driverWrite(base, &FCI_CMD(0), NULL, &FCI_DUMMY(1), &FCI_DATA(2));
}

int defaultGetJedecID(FlashChipBase *base, uint32_t *jedec)
{
	PCHECK(base);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));
	FCI_CMD(0).data = FLASH_INSTRUCTION_RDID;
	FCI_DATA(1).pdata = (uint8_t *)jedec;
	FCI_DATA(1).line = 1;
	FCI_DATA(1).len = 3;
	return base->driverRead(base, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));
}

int defaultEnableQPIMode(FlashChipBase *base)
{
	int ret;
	uint32_t tmp;
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_EN_QPI;
	ret = base->driverWrite(base, &cmd, NULL, NULL, NULL);

	if (ret >= 0)
		base->mFlashStatus |= FLASH_READ_QPI_MODE;
	else
		return ret;

	tmp = 0x03;
	ret = base->control(base, DEFAULT_FLASH_SET_QPI_READ_P5P4, &tmp);
	if (ret >= 0)
		base->mDummyCount = 4;

	return ret;
}

int defaultDisableQPIMode(FlashChipBase *base)
{
	int ret;
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_DIS_QPI;
	cmd.line = 4;
	ret = base->driverWrite(base, &cmd, NULL, NULL, NULL);
	if (ret == 0)
		base->mFlashStatus &= ~FLASH_READ_QPI_MODE;
	return ret;
}

/*
int defaultEnableReset(FlashChipBase *base)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_RSEN;
	return base->driverWrite(base, &cmd, NULL, NULL, NULL);
}
*/

int defaultReset(FlashChipBase *base)
{
	int ret;
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_RSEN;
	ret = base->driverWrite(base, &cmd, NULL, NULL, NULL);
	if (ret < 0)
		return ret;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	cmd.data = FLASH_INSTRUCTION_RESET;
	return base->driverWrite(base, &cmd, NULL, NULL, NULL);
}

int defaultGetUniqueID(FlashChipBase *base, uint8_t uid[8])
{
	PCHECK(base);
	InstructionField instruction[3];

	HAL_Memset(&instruction, 0, sizeof(instruction));
	FCI_CMD(0).data = FLASH_INSTRUCTION_RESET;
	FCI_DUMMY(1).line = 1;
	FCI_DUMMY(1).len = 4;
	FCI_DATA(2).pdata = uid;
	FCI_DATA(2).line = 1;
	FCI_DATA(2).len = 8;
	return base->driverRead(base, &FCI_CMD(0), NULL, &FCI_DUMMY(1), &FCI_DATA(2));
}

int defaultPageProgram(FlashChipBase *base, FlashPageProgramMode mode, uint32_t waddr, const uint8_t *wdata, uint32_t size)
{
	PCHECK(base);
	InstructionField instruction[3];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (size > base->mPageSize)
		return -1;

	if (!(mode & base->mPageProgramSupport)) {
		FLASH_NOTSUPPORT();
		return HAL_INVALID;
	}

	if (((waddr + size) > base->mSize) || (size > base->mPageSize))
		return -1;

	if (mode == FLASH_PAGEPROGRAM)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_PP;
		FCI_DATA(2).line = 1;
	}
	else if (mode == FLASH_QUAD_PAGEPROGRAM)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_QPP;
		FCI_DATA(2).line = 4;
	}
	else
	{
		FLASH_NOWAY();
	}

	FCI_ADDR(1).data = waddr;
	FCI_ADDR(1).line = 1;
	FCI_DATA(2).pdata = (uint8_t *)wdata;
	FCI_DATA(2).len = size;
	return base->driverWrite(base, &FCI_CMD(0), &FCI_ADDR(1), NULL, &FCI_DATA(2));
}

int defaultRead(FlashChipBase *base, FlashReadMode mode, uint32_t raddr, uint8_t *rdata, uint32_t size)
{
	PCHECK(base);
	InstructionField instruction[4];

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (!(mode & base->mReadSupport)) {
		FLASH_DEBUG("this flash chip not support read mode: %d", mode);
		FLASH_NOTSUPPORT();
		return HAL_INVALID;
	}

	if ((raddr + size) > base->mSize)
		return -1;

	FLASH_DEBUG("size: %d; mode: %d", size, (uint32_t)mode);

	switch (mode)
	{
	/* !!! NOTICE: m7~m0 is count to dummy byte. !!! */
		case FLASH_READ_NORMAL_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_READ;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 1;
			FCI_DUMMY(2).len = 0;
			break;
		case FLASH_READ_FAST_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 1;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 1;
			break;
		case FLASH_READ_DUAL_O_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_DO;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 2;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 1;
			break;
		case FLASH_READ_DUAL_IO_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_DIO;
			FCI_ADDR(1).line = 2;
			FCI_DATA(3).line = 2;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 2;
			break;
		case FLASH_READ_QUAD_O_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_QO;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 4;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 1;
			break;
		case FLASH_READ_QUAD_IO_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_QIO;
			FCI_ADDR(1).line = 4;
			FCI_DATA(3).line = 4;
			FCI_DUMMY(2).len = 3;
			FCI_DUMMY(2).line = 4;
			break;
		case FLASH_READ_QPI_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_QIO;
			FCI_CMD(0).line = 4;
			FCI_DATA(3).line = 4;
			FCI_DUMMY(2).len = base->mDummyCount;
			FCI_DUMMY(2).line = 4;
			break;
		default:
			return -1;
	}

	FCI_ADDR(1).data = raddr;
	FCI_DATA(3).pdata = rdata;
	FCI_DATA(3).len = size;
	return base->driverRead(base, &FCI_CMD(0), &FCI_ADDR(1), &FCI_DUMMY(2), &FCI_DATA(3));
}

int defaultDriverWrite(FlashChipBase *base, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	if (base == NULL || cmd == NULL)
		return -1;

	if (!(base->mFlashStatus & FLASH_READ_QPI_MODE))
	{
		cmd->len = 1;
		cmd->line = 1;	//not in QPI

		if (addr != NULL)
			addr->len = 3;
		if (data != NULL && data->pdata == NULL && data->len <= 4)
			data->pdata = (uint8_t *)&data->data;
	}
	else
	{
		cmd->len = 1;
		cmd->line = 4;	//not in QPI

		if (addr != NULL)
		{
			addr->len = 3;
			addr->line = 4;
		}
		if (dummy != NULL)
			dummy->line = 4;
		if (data != NULL)
			data->line = 4;
		if (data != NULL && data->pdata == NULL && data->len <= 4)
			data->pdata = (uint8_t *)&data->data;
	}

	if (base->mDriver == NULL || base->mDriver->write == NULL)
		return -1;

	FLASH_DEBUG("cmd: 0x%x", cmd->data);

	return base->mDriver->write(base->mDriver, cmd, addr, dummy, data);
}

int defaultDriverRead(FlashChipBase *base, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	if (base == NULL || cmd == NULL)
		return -1;

	if (!(base->mFlashStatus & FLASH_READ_QPI_MODE))
	{
		cmd->len = 1;
		cmd->line = 1;	//not in QPI

		if (addr != NULL)
			addr->len = 3;
		if (data != NULL && data->pdata == NULL && data->len <= 4)
			data->pdata = (uint8_t *)&data->data;
	}
	else /* in QPI mode */
	{
		cmd->len = 1;
		cmd->line = 4;	//not in QPI

		if (addr != NULL)
		{
			addr->len = 3;
			addr->line = 4;
		}
		if (dummy != NULL)
			dummy->line = 4;
		if (data != NULL)
			data->line = 4;
		if (data != NULL && data->pdata == NULL && data->len <= 4)
			data->pdata = (uint8_t *)&data->data;
	}

	if (base->mDriver == NULL || base->mDriver->read == NULL)
		return -1;

	FLASH_DEBUG("cmd: 0x%x", cmd->data);

	return base->mDriver->read(base->mDriver, cmd, addr, dummy, data);
}

int defaultXipDriverCfg(FlashChipBase *base, FlashReadMode mode)
{
	PCHECK(base);
	InstructionField instruction[4];
	uint32_t continueMode = 0;	/* flashc exit continue mode, needed a read with dummy */

	if (base->mXip == NULL)
		return -1;

	HAL_Memset(&instruction, 0, sizeof(instruction));

	FCI_CMD(0).len = 1;
	FCI_CMD(0).line = 1;	//not in QPI
	FCI_ADDR(1).len = 3;
	switch (mode)
	{
	/* !!! NOTICE: m7~m0 is count to dummy byte. !!! */
		case FLASH_READ_NORMAL_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_READ;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 1;
			FCI_DUMMY(2).len = 0;
			FCI_DUMMY(2).line = 1;
			continueMode = 0;
			break;
		case FLASH_READ_FAST_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 1;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 1;
			continueMode = 0;
			break;
		case FLASH_READ_DUAL_O_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_DO;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 2;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 1;
			continueMode = 0;
			break;
		case FLASH_READ_DUAL_IO_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_DIO;
			FCI_ADDR(1).line = 2;
			FCI_DATA(3).line = 2;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 2;
			break;
		case FLASH_READ_QUAD_O_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_QO;
			FCI_ADDR(1).line = 1;
			FCI_DATA(3).line = 4;
			FCI_DUMMY(2).len = 1;
			FCI_DUMMY(2).line = 1;
			continueMode = 0;
			break;
		case FLASH_READ_QUAD_IO_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_QIO;
			FCI_ADDR(1).line = 4;
			FCI_DATA(3).line = 4;
			FCI_DUMMY(2).len = 3;
			FCI_DUMMY(2).line = 4;
			break;
		case FLASH_READ_QPI_MODE:
			FCI_CMD(0).data = FLASH_INSTRUCTION_FAST_READ_QIO;
			FCI_CMD(0).line = 4;
			FCI_DATA(3).line = 4;
			FCI_DUMMY(2).len = base->mDummyCount;
			FCI_DUMMY(2).line = 4;
			FCI_ADDR(1).line = 4;
			break;
		default:
			return -1;
	}

	base->mXip->setCmd(base->mXip, &FCI_CMD(0), &FCI_ADDR(1), &FCI_DUMMY(2), &FCI_DATA(3));
	base->mXip->setContinue(base->mXip, continueMode, NULL);
	/*TODO: xip set delay*/

	return 0;
}

int defaultSetFreq(FlashChipBase *base, uint32_t freq)
{
	PCHECK(base);

	return base->mDriver->setFreq(base->mDriver, freq);
}

int defaultSwitchReadMode(FlashChipBase *base, FlashReadMode mode)
{
	PCHECK(base);
	uint8_t status;
	int ret;

	if (!(mode & base->mReadSupport)) {
		FLASH_NOTSUPPORT();
		return HAL_INVALID;
	}

	if (!((base->mReadStausSupport & FLASH_STATUS2) && (base->mWriteStatusSupport & FLASH_STATUS2))) {
		//do not need switch
		return 0;
	}

	if (mode == FLASH_READ_QUAD_O_MODE || mode == FLASH_READ_QUAD_IO_MODE || mode == FLASH_READ_QPI_MODE)
	{
		ret = base->readStatus(base, FLASH_STATUS2, &status);
		if (ret < 0)
			return -1;
		status |= 1 << 1;
		ret = base->writeStatus(base, FLASH_STATUS2, &status);
	}
	else
	{
		ret = base->readStatus(base, FLASH_STATUS2, &status);
		if (ret < 0)
			return -1;
		status &= ~(1 << 1);
		ret = base->writeStatus(base, FLASH_STATUS2, &status);
	}

	return ret;
}

int defaultEnableXIP(FlashChipBase *base)
{
	PCHECK(base);

	/*TODO: it should mean the continue mode, so it would not use for now. */

	return 0;
}

int defaultDisableXIP(FlashChipBase *base)
{
	PCHECK(base);

	/*TODO: it should mean the continue mode, so it would not use for now. */

	return 0;
}

int defaultIsBusy(FlashChipBase *base)
{
	PCHECK(base);
	uint8_t status;
	int ret;

	ret = base->readStatus(base, FLASH_STATUS1, &status);
	if (ret < 0)
		return -1;

	return !!(status & (1 << 0));
}

static int defaultSetReadParam(FlashChipBase *base, uint8_t param)
{
	PCHECK(base);
	InstructionField instruction[2];

	HAL_Memset(&instruction, 0, sizeof(instruction));
	FCI_CMD(0).data = FLASH_INSTRUCTION_SRP;
	FCI_DATA(1).pdata = (uint8_t *)&param;
	FCI_DATA(1).line = 4;
	FCI_DATA(1).len = 1;
	return base->driverWrite(base, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));
}

int defaultControl(FlashChipBase *base, int op, void *param)
{
	PCHECK(base);
	InstructionField cmd;

	HAL_Memset(&cmd, 0, sizeof(cmd));
	switch (op)
	{
		case DEFAULT_FLASH_SET_QPI_READ_P5P4:
			if (*(uint8_t *)param > 0x03)
				return -1;
			defaultSetReadParam(base, (*(uint8_t *)param) << 4);
			break;
		case DEFAULT_FLASH_POWERDOWN:
			cmd.data = FLASH_INSTRUCTION_PWDN;
			return base->driverWrite(base, &cmd, NULL, NULL, NULL);
			break;
	}
	return 0;
}

FlashEraseMode defaultGetMinEraseSize(FlashChipBase *base)
{
	PCHECK(base);

	if (base->mEraseSizeSupport & FLASH_ERASE_4KB)
		return FLASH_ERASE_4KB;
	else if (base->mEraseSizeSupport & FLASH_ERASE_32KB)
		return FLASH_ERASE_32KB;
	else if (base->mEraseSizeSupport & FLASH_ERASE_64KB)
		return FLASH_ERASE_64KB;
	else
		return FLASH_ERASE_NOSUPPORT;
}


/*
	Default Flash Chip, Only support basic Interface
*/
#if 0
typedef struct DefaultFlash
{
	FlashChipBase base;
} DefaultFlash;

static int DefaultFlashInit(FlashChipBase * base)
{
	PCHECK(base);

	DefaultFlash *impl = __containerof(base, DefaultFlash, base);
	impl->base.writeEnable = defaultWriteEnable;
	impl->base.writeDisable = defaultWriteDisable;
	impl->base.readStatus = defaultReadStatus;
	impl->base.erase = defaultErase;
	impl->base.jedecID = defaultGetJedecID;
	impl->base.pageProgram = defaultPageProgram;
	impl->base.read = defaultRead;

	impl->base.driverWrite = defaultDriverWrite;
	impl->base.driverRead = defaultDriverRead;
	impl->base.xipDriverCfg = defaultXipDriverCfg;
	impl->base.setFreq = defaultSetFreq;
	impl->base.switchReadMode = defaultSwitchReadMode;
	impl->base.enableXIP = defaultEnableXIP;
	impl->base.disableXIP = defaultDisableXIP;
	impl->base.isBusy = defaultIsBusy;
	impl->base.control = defaultControl;
	impl->base.minEraseSize = defaultGetMinEraseSize;

	impl->base.writeStatus = NULL;
	impl->base.suspendErasePageprogram = NULL;
	impl->base.resumeErasePageprogram = NULL;
	impl->base.powerDown = NULL;
	impl->base.releasePowerDown = NULL;
	impl->base.enableQPIMode = NULL;
	impl->base.disableQPIMode = NULL;
	impl->base.enableReset = NULL;
	impl->base.reset = NULL;
	impl->base.uniqueID = NULL;
	/*TODO: a NULL interface for showing invalid interface*/

	FLASH_DEBUG("default chip inited");

	return 0;
}

static int DefaultFlashDeinit(FlashChipBase * base)
{
	PCHECK(base);

	DefaultFlash *impl = __containerof(base, DefaultFlash, base);
	free(impl);

	return 0;
}

static FlashChipBase *DefaultFlashCtor(void)
{
	DefaultFlash *impl = malloc(sizeof(DefaultFlash));
	PCHECK(impl);
	HAL_Memset(impl, 0, sizeof(DefaultFlash));

	FLASH_DEBUG("create default chip");

	impl->base.mPageSize = 256;
	impl->base.mSize = -1;
	impl->base.mMaxFreq = -1;
	impl->base.mMaxReadFreq = -1;
/*
	impl->base.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_CHIP;
	impl->base.mPageProgramSupport = FLASH_PAGEPROGRAM;
	impl->base.mReadStausSupport = FLASH_STATUS1;
	impl->base.mWriteStatusSupport = FLASH_STATUS1;
	impl->base.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE;
*/
	impl->base.mEraseSizeSupport = FLASH_ERASE_4KB | FLASH_ERASE_32KB | FLASH_ERASE_64KB | FLASH_ERASE_CHIP;
	impl->base.mPageProgramSupport = FLASH_PAGEPROGRAM;
	impl->base.mReadStausSupport = FLASH_STATUS1;
	impl->base.mWriteStatusSupport = FLASH_STATUS1;
	impl->base.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE;

	return &impl->base;
}

static FlashChipCtor DefaultFlashChip = {
		.mJedecId = 0,
		.create = DefaultFlashCtor,
		.init = DefaultFlashInit,
		.destory = DefaultFlashDeinit,
};
#endif
