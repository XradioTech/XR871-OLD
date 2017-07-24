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

#include "driver/chip/hal_norflash.h"
#include "driver/chip/hal_spi.h"
#include "hal_base.h"
#include "sys/param.h"
#include <stdbool.h>

/************************ private *************************************/
typedef enum {
	eSF_INSTRUCTION_WREN = 0x06,				/* write enable */
	eSF_INSTRUCTION_WRDI = 0x04,				/* write disable */
	eSF_INSTRUCTION_RDID = 0x9F,				/* jedec id */
	eSF_INSTRUCTION_RDSR1 = 0x05,				/* read status register-1 */
	eSF_INSTRUCTION_WRSR1 = 0x01,				/* write status register-1 */
	eSF_INSTRUCTION_READ = 0x03,				/* read data */
	eSF_INSTRUCTION_FAST_READ = 0x0B,			/* fast read */
	eSF_INSTRUCTION_PP = 0x02,					/* page program */
	eSF_INSTRUCTION_ERASE_64KB = 0xD8,			/* erase block(sector) 64k */
	eSF_INSTRUCTION_ERASE_32KB = 0x52,			/* erase block(sector) 32k */
	eSF_INSTRUCTION_ERASE_4KB = 0x20,			/* erase sector 4k */
	eSF_INSTRUCTION_ERASE_CHIP = 0xC7,			/* chip erase */
	eSF_INSTRUCTION_WRSR = 0X01,				/* write status register */
	eSF_INSTRUCTION_FAST_READ_DO = 0x3B,		/* fast read dual output */

	eSF_INSTRUCTION_RDSR2 = 0x35,
 /*
 	eSF_INSTRUCTION_RDSR3 = 0x15,
	eSF_INSTRUCTION_WRSR2 = 0x31,
	eSF_INSTRUCTION_WRSR3 = 0x11,
	eSF_INSTRUCTION_SRWREN = 0x50,*/
//	eSF_INSTRUCTION_CE = 0x60,
/*	eSF_INSTRUCTION_EPSP = 0x75,
	eSF_INSTRUCTION_EPRS = 0x7A,
	eSF_INSTRUCTION_PWDN = 0xB9,
	eSF_INSTRUCTION_RES = 0xAB,*/
/*	eSF_INSTRUCTION_FAST_READ_DIO = 0xBB,
	eSF_INSTRUCTION_FAST_READ_QO = 0x6B,
	eSF_INSTRUCTION_FAST_READ_QIO = 0xEB,
	eSF_INSTRUCTION_QPI = 0x38,
	eSF_INSTRUCTION_RSEN = 0x66,
	eSF_INSTRUCTION_RESET = 0x99,
	eSF_INSTRUCTION_QPP = 0x32*/
} eSF_Instruction;

typedef struct {
	uint8_t wren;
	uint8_t wrdi;
	uint8_t erase_64KB;
	uint8_t erase_32KB;
	uint8_t erase_4KB;
	uint8_t erase_chip;
	uint8_t read_jedec_id;
	uint8_t read_status1;
	uint8_t write_status1;
	uint8_t read;
	uint8_t fast_read;
	uint8_t fast_read_dual_output;
	uint8_t page_program;
} SF_Chip_Support;

typedef struct {
	uint32_t page_size;
	uint32_t sector_size;
	uint32_t n_sectors;
	uint32_t max_read_freq;
	bool chip_select_level;
} SF_Chip_Parameter;

typedef struct {
	uint8_t id;
	uint16_t jedec_id;
	SF_Chip_Support support;
	SF_Chip_Parameter param;
} SF_Chip;

typedef HAL_Status (*SF_Func)(SPI_Port port, uint32_t addr, uint8_t *data, uint32_t size);

typedef struct {
	SPI_Port spi_n;
	SPI_CS spi_cs_n;
	SPI_Config spi_config;
	SF_Chip *chip;
	SF_Func pp;
	SF_Func read;
	HAL_Mutex lock;
} SF_Device;

typedef struct {
	eSF_Instruction 	instruction;
	uint8_t 			dummy_bytes: 	4;
	uint8_t 			addr_on: 		4;
	uint32_t 			addr: 			24;
} SF_CMD;

static HAL_Status SF_SendCMD(SPI_Port port, SF_CMD *cmd)
{
	uint8_t buf[10];
	uint8_t i = 0;

	buf[i++] = cmd->instruction;
	if (cmd->addr_on != 0) {
		buf[i++] = cmd->addr >> 16;
		buf[i++] = cmd->addr >> 8;
		buf[i++] = cmd->addr;
	}
	if (cmd->dummy_bytes != 0) {
		uint8_t cnt = cmd->dummy_bytes;
		while (cnt--) {
			buf[i++] = 0;
			SF_ASSERT(i < 10);
		}
	}

	return HAL_SPI_Transmit(port, buf, i);
}

static inline void SF_Enable(SPI_Port port)
{
	HAL_SPI_CS(port, 1);
}

static inline void SF_Disable(SPI_Port port)
{
	HAL_SPI_CS(port, 0);
}

static void SF_EnableWrite(SPI_Port port)
{
	SF_CMD cmd;
	cmd.instruction = eSF_INSTRUCTION_WREN;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	SF_Disable(port);
}

static void SF_DisableWrite(SPI_Port port)
{
	SF_CMD cmd;
	cmd.instruction = eSF_INSTRUCTION_WRDI;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	SF_Disable(port);
}

typedef struct {
	uint8_t manufacturer_id;
	uint8_t memory_type;
	uint8_t memory_capacity;
} SF_Jedec;

static void SF_ReadJedecId(SPI_Port port, SF_Jedec *jedec)
{
	SF_CMD cmd;
	cmd.instruction = eSF_INSTRUCTION_RDID;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Receive(port, (uint8_t *)jedec, sizeof(*jedec));
	SF_Disable(port);
}

typedef struct {
	uint8_t SRP0: 1;
	uint8_t SEC: 1;
	uint8_t TB: 1;
	uint8_t BP2: 1;
	uint8_t BP1: 1;
	uint8_t BP0: 1;
	uint8_t WEL: 1;
	uint8_t BUSY: 1;
} SF_StatusRegister1;	//this can be different in different flash chip.

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
} SF_StatusRegister2;	//this can be different in different flash chip.


#define SF_STATUS1_BUSY_SHIFT		(0)
#define SF_STATUS1_BUSY_VMASK		(0x01)

#define SF_STATUS1_WEL_SHIFT		(1)
#define SF_STATUS1_WEL_VMASK		(0x01)

#define SF_STATUS1_BP0_SHIFT		(2)
#define SF_STATUS1_BP0_VMASK		(0x01)

#define SF_STATUS1_BP1_SHIFT		(3)
#define SF_STATUS1_BP1_VMASK		(0x01)

#define SF_STATUS1_BP2_SHIFT		(4)
#define SF_STATUS1_BP2_VMASK		(0x01)

#define SF_STATUS1_TB_SHIFT			(5)
#define SF_STATUS1_TB_VMASK			(0x01)

#define SF_STATUS1_SEC_SHIFT		(6)
#define SF_STATUS1_SEC_VMASK		(0x01)

#define SF_STATUS1_SRP0_SHIFT		(7)
#define SF_STATUS1_SRP0_VMASK		(0x01)

static HAL_Status SF_ReadStatus1(SPI_Port port, SF_StatusRegister1 *status)
{
	SF_CMD cmd;
	uint8_t buf;

	cmd.instruction = eSF_INSTRUCTION_RDSR1;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Receive(port, &buf, 1);
	SF_Disable(port);

	status->BUSY = HAL_GET_BIT_VAL(buf, SF_STATUS1_BUSY_SHIFT, SF_STATUS1_BUSY_VMASK);
	status->WEL = HAL_GET_BIT_VAL(buf, SF_STATUS1_WEL_SHIFT, SF_STATUS1_WEL_VMASK);
	status->BP0 = HAL_GET_BIT_VAL(buf, SF_STATUS1_BP0_SHIFT, SF_STATUS1_BP0_VMASK);
	status->BP1 = HAL_GET_BIT_VAL(buf, SF_STATUS1_BP1_SHIFT, SF_STATUS1_BP1_VMASK);
	status->BP2 = HAL_GET_BIT_VAL(buf, SF_STATUS1_BP2_SHIFT, SF_STATUS1_BP2_VMASK);
	status->TB = HAL_GET_BIT_VAL(buf, SF_STATUS1_TB_SHIFT, SF_STATUS1_TB_VMASK);
	status->SEC = HAL_GET_BIT_VAL(buf, SF_STATUS1_SEC_SHIFT, SF_STATUS1_SEC_VMASK);
	status->SRP0 = HAL_GET_BIT_VAL(buf, SF_STATUS1_SRP0_SHIFT, SF_STATUS1_SRP0_VMASK);

	return HAL_OK;
}

static HAL_Status SF_ReadStatus2(SPI_Port port, SF_StatusRegister2 *status)
{
	SF_CMD cmd;
	uint8_t buf;

	cmd.instruction = eSF_INSTRUCTION_RDSR2;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Receive(port, &buf, 1);
	SF_Disable(port);

	status->status = buf;

	return HAL_OK;
}


/*static HAL_Status SF_WriteStatus1(SPI_Port port, SF_StatusRegister1 *status)
{
	SF_CMD cmd;
	uint8_t buf;

	cmd.instruction = eSF_INSTRUCTION_;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	buf = (status->BUSY << SF_STATUS1_BUSY_SHIFT)
		| (status->WEL << SF_STATUS1_WEL_SHIFT)
		| (status->BP0 << SF_STATUS1_BP0_SHIFT)
		| (status->BP1 << SF_STATUS1_BP1_SHIFT)
		| (status->BP2 << SF_STATUS1_BP2_SHIFT)
		| (status->TB << SF_STATUS1_TB_SHIFT)
		| (status->SEC << SF_STATUS1_SEC_SHIFT)
		| (status->SRP0 << SF_STATUS1_SRP0_SHIFT);

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Transmit(port, &buf, 1);
	SF_Disable(port);

	return HAL_OK;
}*/
static HAL_Status SF_WriteStatus(SPI_Port port, SF_StatusRegister1 *status1, SF_StatusRegister2 *status2)
{
	SF_CMD cmd;
	uint8_t buf[2];

	cmd.instruction = eSF_INSTRUCTION_WRSR;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	buf[0] = (status1->BUSY << SF_STATUS1_BUSY_SHIFT)
		| (status1->WEL << SF_STATUS1_WEL_SHIFT)
		| (status1->BP0 << SF_STATUS1_BP0_SHIFT)
		| (status1->BP1 << SF_STATUS1_BP1_SHIFT)
		| (status1->BP2 << SF_STATUS1_BP2_SHIFT)
		| (status1->TB << SF_STATUS1_TB_SHIFT)
		| (status1->SEC << SF_STATUS1_SEC_SHIFT)
		| (status1->SRP0 << SF_STATUS1_SRP0_SHIFT);

	buf[1] = *(uint8_t *)status2;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	if (status2 == NULL)
		HAL_SPI_Transmit(port, buf, 1);
	else
		HAL_SPI_Transmit(port, buf, 2);
	SF_Disable(port);

	return HAL_OK;
}

/*static HAL_Status SF_Read(SPI_Port port, uint32_t addr, uint8_t *data, uint32_t size)
{
	SF_CMD cmd;

	SF_ASSERT((addr & 0xFF000000) == 0);

	cmd.instruction = eSF_INSTRUCTION_READ;
	cmd.addr_on = 1;
	cmd.addr = addr;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Receive(port, data, size);
	SF_Disable(port);

	return HAL_OK;
}*/

static HAL_Status SF_FastRead(SPI_Port port, uint32_t addr, uint8_t *data, uint32_t size)
{
	SF_CMD cmd;

	SF_ASSERT((addr & 0xFF000000) == 0);

	cmd.instruction = eSF_INSTRUCTION_FAST_READ;
	cmd.addr_on = 1;
	cmd.addr = addr;
	cmd.dummy_bytes = 1;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Receive(port, data, size);
	SF_Disable(port);

	return HAL_OK;
}

static HAL_Status SF_PageProgram(SPI_Port port, uint32_t addr, uint8_t *data, uint32_t size)
{
	SF_CMD cmd;

	SF_ASSERT((addr & 0xFF000000) == 0);


#ifdef SF_MODULE_DEBUG
	if (size > 256) {
		SF_DEBUG("pp size over %d.\n", 256);
	} else if (size == 0)
		return HAL_ERROR;
	if (size > 256 - (addr % 256))
		SF_ALERT("pp data will over the sector edge, this will rewrite this sector\n");
#endif

	cmd.instruction = eSF_INSTRUCTION_PP;
	cmd.addr_on = 1;
	cmd.addr = addr;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Transmit(port, data, size);
	SF_Disable(port);

	return HAL_OK;
}

static HAL_Status SF_Erase(SPI_Port port, SF_Chip_Support support, SF_Erase_Size size,  uint32_t addr)
{
	SF_CMD cmd;

	SF_ASSERT((addr & 0xFF000000) == 0);

	if ((size == SF_ERASE_SIZE_64KB) && (support.erase_64KB != 0))
		cmd.instruction = eSF_INSTRUCTION_ERASE_64KB;
	else if ((size == SF_ERASE_SIZE_32KB) && (support.erase_32KB != 0))
		cmd.instruction = eSF_INSTRUCTION_ERASE_32KB;
	else if ((size == SF_ERASE_SIZE_4KB) && (support.erase_4KB != 0))
		cmd.instruction = eSF_INSTRUCTION_ERASE_4KB;
	else
		return HAL_INVALID;

	cmd.addr_on = 1;
	cmd.addr = addr;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	SF_Disable(port);

	return HAL_OK;
}

HAL_Status SF_EraseChip(SPI_Port port)
{
	SF_CMD cmd;
	cmd.instruction = eSF_INSTRUCTION_ERASE_CHIP;
	cmd.addr_on = 0;
	cmd.dummy_bytes = 0;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	SF_Disable(port);

	return HAL_OK;
}

static HAL_Status SF_FastReadDualOutput(SPI_Port port, uint32_t addr, uint8_t *data, uint32_t size)
{
	SF_CMD cmd;

	SF_ASSERT((addr & 0xFF000000) == 0);

	cmd.instruction = eSF_INSTRUCTION_FAST_READ_DO;
	cmd.addr_on = 1;
	cmd.addr = addr;
	cmd.dummy_bytes = 1;

	SF_Enable(port);
	SF_SendCMD(port, &cmd);
	HAL_SPI_Receive(port, data, size);
	SF_Disable(port);

	return HAL_OK;
}


/************************ private *************************************/
#ifdef FLASH_M25P64
#define M25P64_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 0, \
	.erase_4KB = 0, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 0, \
	.page_program = 1 \
}

#define M25P64_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x10000, \
	.n_sectors = 128, \
	.max_read_freq = 20 * 1000 * 1000, \
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_W25Q16FW
#define W25Q16FW_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 1, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define W25Q16FW_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 50 * 1000 * 1000, \
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_PN25F08
#define PN25F08_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 1, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define PN25F08_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 16 * 16, \
	.max_read_freq = 55 * 1000 * 1000, \
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_PN25F16
#define PN25F16_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 1, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define PN25F16_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 55 * 1000 * 1000, \
	.chip_select_level = 0 \
}
#endif


#ifdef FLASH_MX25L1636E
#define MX25L1636E_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 0, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define MX25L1636E_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 50 * 1000 * 1000, \
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_MX25L1633E
#define MX25L1633E_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 0, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define MX25L1633E_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 33 * 1000 * 1000, \
	/* other cmd with its max freq too */ \
	.chip_select_level = 0 \
}
#endif


#ifdef FLASH_MX25L1606E
#define MX25L1606E_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 0, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define MX25L1606E_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 33 * 1000 * 1000, \
	/* other cmd with its max freq too */\
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_PN25F16B
#define PN25F16B_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 1, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define PN25F16B_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 55 * 1000 * 1000, \
	/* other cmd with its max freq too */\
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_PN25F08B
#define PN25F08B_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 1, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define PN25F08B_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 16 * 16, \
	.max_read_freq = 55 * 1000 * 1000, \
	/* other cmd with its max freq too */\
	.chip_select_level = 0 \
}
#endif

#ifdef FLASH_EN25QH16A
#define EN25QH16A_SUPPORT { \
	.wren = 1, \
	.wrdi = 1, \
	.erase_64KB = 1, \
	.erase_32KB = 1, \
	.erase_4KB = 1, \
	.erase_chip = 1, \
	.read_jedec_id = 1, \
	.read_status1 = 1, \
	.write_status1 = 1, \
	.read = 1, \
	.fast_read = 1, \
	.fast_read_dual_output = 1, \
	.page_program = 1 \
}

#define EN25QH16A_PARAMETER { \
	.page_size = 256, \
	.sector_size = 0x1000, \
	.n_sectors = 32 * 16, \
	.max_read_freq = 50 * 1000 * 1000, \
	.chip_select_level = 0 \
}
#endif


static SF_Chip chip_info[] = {
#ifdef FLASH_M25P64
		{
				/* M25P64 */
				.id = 0x20,
				.jedec_id = 0x2017,
				.support = M25P64_SUPPORT,
				.param = M25P64_PARAMETER
		},
#endif
#ifdef FLASH_W25Q16FW
		{
				/* W25Q16FW */
				.id = 0xEF,
				.jedec_id = 0x6015,
				.support = W25Q16FW_SUPPORT,
				.param = W25Q16FW_PARAMETER
		},
#endif
#ifdef FLASH_PN25F08
		{
				/* PN25F08 */
				.id = 0xE0,
				.jedec_id = 0x4014,
				.support = PN25F08_SUPPORT,
				.param = PN25F08_PARAMETER
		},
#endif
#ifdef FLASH_MX25L1636E
		{
				/* MX25L1636E */
				.id = 0xC2,
				.jedec_id = 0x2515,
				.support = MX25L1636E_SUPPORT,
				.param = MX25L1636E_PARAMETER
		},
#endif
#ifdef FLASH_MX25L1633E
		{
				/* MX25L1636E */
				.id = 0xC2,
				.jedec_id = 0x2415,
				.support = MX25L1633E_SUPPORT,
				.param = MX25L1633E_PARAMETER
		},
#endif
#ifdef FLASH_MX25L1606E
		{
				/* MX25L1636E */
				.id = 0xC2,
				.jedec_id = 0x2015,
				.support = MX25L1606E_SUPPORT,
				.param = MX25L1606E_PARAMETER
		},
#endif
#ifdef FLASH_PN25F16B
		{
				/* MX25L1636E */
				.id = 0x5E,
				.jedec_id = 0x4015,
				.support = PN25F16B_SUPPORT,
				.param = PN25F16B_PARAMETER
		},
#endif
#ifdef FLASH_PN25F08B
		{
				/* MX25L1636E */
				.id = 0x5E,
				.jedec_id = 0x4014,
				.support = PN25F08B_SUPPORT,
				.param = PN25F08B_PARAMETER
		},
#endif
#ifdef FLASH_EN25QH16A
		{
				/* EN25QH16A */
				.id = 0x1C,
				.jedec_id = 0x7015,
				.support = EN25QH16A_SUPPORT,
				.param = EN25QH16A_PARAMETER
		},
#endif
#ifdef FLASH_PN25F16
	{
		/* PN25F16 */
		.id = 0xE0,
		.jedec_id = 0x4015,
		.support = PN25F16_SUPPORT,
		.param = PN25F16_PARAMETER
	},

#endif

};

#include "driver/chip/hal_flashctrl.h"

bool HAL_XIP_IsRunning();
HAL_Status HAL_XIP_Stop();
HAL_Status HAL_XIP_Start();

void udelay(unsigned int us);

static HAL_Status HAL_SF_WaitComplete(SF_Device *dev, int32_t msec, uint32_t xip_on)
{
	SF_StatusRegister1 status1;
	uint32_t per_delay_ms = 10;

	if (SF_ReadStatus1(dev->spi_n, &status1) != HAL_OK)
		return HAL_ERROR;

	while (status1.BUSY != 0) {
		if (msec <= 0) {
			SF_ALERT(" wait flash busy releasing timeout, msec: %d\n", msec);
			return HAL_TIMEOUT;
		}

#if 0	//xip can't read
		HAL_SPI_Close(dev->spi_n);
		if (xip_on) {
			HAL_XIP_Start();
			OS_ThreadResumeScheduler();
		}
		HAL_MSleep(per_delay_ms);
		if (xip_on) {
			OS_ThreadSuspendScheduler();
			HAL_XIP_Stop();
		}
		if ((HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, 2 * SF_MAX_WAIT_TIME)) != HAL_OK) {
			SF_ALERT("xip->flash: open spi failed\n");
			return HAL_ERROR;
		}
#else
		udelay(1000 * per_delay_ms);
#endif
		if (SF_ReadStatus1(dev->spi_n, &status1) != HAL_OK)
			return HAL_ERROR;
		msec -= per_delay_ms;
	}

	return HAL_OK;
}

/************************ public **************************************/

HAL_Status HAL_SF_Init(SF_Handler *hdl, const SF_Config *config)
{
	HAL_Status ret;
	SF_Jedec id;
	uint8_t chip = 0;
	uint8_t support_num = 0;
	SF_Device *dev = malloc(sizeof(SF_Device));

	SF_ENTRY();

	bool xip = HAL_XIP_IsRunning();
	if (xip) {
		OS_ThreadSuspendScheduler();
		HAL_XIP_Stop();
	}

	*hdl = dev;
	memset(dev, 0, sizeof(SF_Device));

	if ((ret = HAL_MutexInit(&dev->lock)) != HAL_OK){
		SF_DEBUG("mutex init failed\n");
		goto failed;
	}

	dev->pp = SF_PageProgram;
	dev->read = SF_FastRead;
	dev->spi_cs_n = config->cs;
	dev->spi_n = config->spi;

//	dev->spi_config.csIdle = 1;
//	dev->spi_config.csMode = SPI_CS_MODE_MANUAL;
	dev->spi_config.firstBit = SPI_TCTRL_FBS_MSB;
	dev->spi_config.mode = SPI_CTRL_MODE_MASTER;
	if ((config->cs) && (config->dma_en))	// for xip
		dev->spi_config.opMode = SPI_OPERATION_MODE_DMA;
	else
		dev->spi_config.opMode = SPI_OPERATION_MODE_POLL;
	dev->spi_config.sclkMode = SPI_SCLK_Mode0;
	dev->spi_config.sclk = config->sclk;

	if ((ret = HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto failed;

	if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK) {
		HAL_SPI_Close(dev->spi_n);
		goto failed;
	}

	SF_ReadJedecId(dev->spi_n, &id);

	HAL_SPI_Close(dev->spi_n);

	support_num = sizeof(chip_info) / sizeof(SF_Chip);
	SF_DEBUG("support flash number %d, mfid=0x%x, idh=0x%x, idl=0x%x.\n",
			 support_num, id.manufacturer_id, id.memory_type, id.memory_capacity);

	do {
		if ((chip_info[chip].id == id.manufacturer_id)
			&& (chip_info[chip].jedec_id == ((id.memory_type << 8) | (id.memory_capacity << 0))))
			break;
	} while ((++chip) < support_num);
	if (chip == support_num) {
		ret = HAL_INVALID;
		goto failed;
	}

	dev->chip = &chip_info[chip];

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	SF_EXIT_VAL(HAL_OK);
	return HAL_OK;
failed:
	free(dev);
	*hdl = NULL;

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	SF_EXIT_VAL(ret);
	return ret;
}

HAL_Status HAL_SF_Deinit(SF_Handler *hdl)
{
	HAL_Status ret;
	SF_Device *dev = *hdl;

	SF_ENTRY();

	if ((ret = HAL_MutexLock(&dev->lock, SF_MAX_WAIT_TIME)) != HAL_OK)
		return ret;

	bool xip = HAL_XIP_IsRunning();
	if (xip) {
		OS_ThreadSuspendScheduler();
		HAL_XIP_Stop();
	}

	if ((ret = HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, SF_MAX_WAIT_TIME)) != HAL_OK)
		return ret;
	ret = HAL_SF_WaitComplete(dev, -1, xip);
	HAL_SPI_Close(dev->spi_n);

	HAL_MutexUnlock(&dev->lock);
	HAL_MutexDeinit(&dev->lock);

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	*hdl = NULL;
	free(dev);
	SF_EXIT_VAL(ret);
	return ret;
}

HAL_Status HAL_SF_Config(SF_Handler *hdl, SF_Attribution attr, uint32_t arg)
{
	SF_Device *dev = *hdl;
	HAL_Status ret = HAL_OK;

	SF_ENTRY();
	if ((ret = HAL_MutexLock(&dev->lock, SF_MAX_WAIT_TIME)) != HAL_OK)
		return ret;

	bool xip = HAL_XIP_IsRunning();
	if (xip) {
		OS_ThreadSuspendScheduler();
		HAL_XIP_Stop();
	}

	if (attr == SF_ATTRIBUTION_READ_MODE) {
		if (arg == SF_READ_MODE_NORMAL_IO) {
			dev->read = SF_FastRead;
			HAL_SPI_Config(dev->spi_n, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);
		} else if (arg == SF_READ_MODE_DUAL_OUTPUT) {
			if (dev->chip->support.fast_read_dual_output == 0) {
				ret = HAL_INVALID;
				goto failed;
			}
			dev->read = SF_FastReadDualOutput;
			HAL_SPI_Config(dev->spi_n, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_DUAL_RX);
		} else if (arg == SF_READ_MODE_QUAD_IO) {
			if ((ret = HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, SF_MAX_WAIT_TIME)) != HAL_OK)
				goto failed;

			if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK) {
				HAL_SPI_Close(dev->spi_n);
				goto failed;
			}

			SF_StatusRegister1 status1;
			SF_StatusRegister2 status2;

			SF_ReadStatus1(dev->spi_n, &status1);
			SF_ReadStatus2(dev->spi_n, &status2);
			status2.QE = 1;

			SF_EnableWrite(dev->spi_n);
			SF_WriteStatus(dev->spi_n, &status1, &status2);
			SF_DisableWrite(dev->spi_n);

			HAL_SPI_Close(dev->spi_n);
		} else {
			ret = HAL_INVALID;
			goto failed;
		}
	} else {
		ret = HAL_INVALID;
		goto failed;
	}

failed:

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	HAL_MutexUnlock(&dev->lock);
	SF_EXIT_VAL(ret);
	return ret;
}

HAL_Status HAL_SF_Write(SF_Handler *hdl, uint32_t addr, uint8_t *data, uint32_t size)
{
	SF_Device *dev = *hdl;
	uint32_t page_size = dev->chip->param.page_size;
	uint32_t pp_size;
	uint32_t left = size;
	uint32_t address = addr;
	uint8_t *ptr = data;
	HAL_Status ret = HAL_OK;

	bool xip = HAL_XIP_IsRunning();
	if (xip) {
		OS_ThreadSuspendScheduler();
		HAL_XIP_Stop();
	}

	SF_ENTRY();
	SF_ASSERT(dev->pp != NULL);
	if ((addr + size) > (dev->chip->param.n_sectors * dev->chip->param.sector_size)) {
		SF_DEBUG("write memory is over flash memory\n");
		ret = HAL_INVALID;
		goto out;
	}

	if ((ret = HAL_MutexLock(&dev->lock, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto out;
	if ((ret = HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto failed_open;
//	SF_EnableWrite(dev->spi_n);

	while (left > 0) {
		pp_size = MIN(left, page_size - (address % page_size));

		if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK)
			goto failed_op;
		SF_EnableWrite(dev->spi_n);
		if ((ret = dev->pp(dev->spi_n, address, ptr, pp_size)) != HAL_OK)
			goto failed_op;
		SF_DisableWrite(dev->spi_n);
		if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK)
			goto failed_op;

		address += pp_size;
		ptr += pp_size;
		left -= pp_size;
	}

failed_op:
	HAL_SPI_Close(dev->spi_n);
failed_open:
	HAL_MutexUnlock(&dev->lock);
out:

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	SF_EXIT_VAL(ret);
	return ret;
}

HAL_Status HAL_SF_Read(SF_Handler *hdl, uint32_t addr, uint8_t *data, uint32_t size)
{
	SF_Device *dev = *hdl;
	HAL_Status ret = HAL_OK;

	bool xip = HAL_XIP_IsRunning();
	if (xip) {
		OS_ThreadSuspendScheduler();
		HAL_XIP_Stop();
	}

	SF_ENTRY();
	SF_ASSERT(dev->read != NULL);
	if ((addr + size) > (dev->chip->param.n_sectors * dev->chip->param.sector_size)) {
		SF_DEBUG("read memory is over flash memory\n");
		ret = HAL_ERROR;
		goto out;
	}

	if ((ret = HAL_MutexLock(&dev->lock, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto out;
	if ((ret = HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto failed_open;

	if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK)
		goto failed_op;

	if ((ret = dev->read(dev->spi_n, addr, data, size)) != HAL_OK)
		goto failed_op;

failed_op:
	HAL_SPI_Close(dev->spi_n);
failed_open:
	HAL_MutexUnlock(&dev->lock);
out:

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	SF_EXIT_VAL(ret);
	return ret;
}


HAL_Status HAL_SF_Erase(SF_Handler *hdl, SF_Erase_Size blk_size, uint32_t addr, uint32_t blk_cnt)
{
	SF_Device *dev = *hdl;
	HAL_Status ret = HAL_OK;
	uint32_t esize = 0;

	SF_ENTRY();

	bool xip = HAL_XIP_IsRunning();
	if (xip) {
		OS_ThreadSuspendScheduler();
		HAL_XIP_Stop();
	}

	if ((addr + blk_size * blk_cnt) > (dev->chip->param.n_sectors * dev->chip->param.sector_size)) {
		SF_DEBUG("erase memory is over flash memory\n");
		ret = HAL_INVALID;
		goto out;
	}

	if ((blk_size == SF_ERASE_SIZE_64KB) && (dev->chip->support.erase_64KB != 0))
		esize = 0x10000;
	else if ((blk_size == SF_ERASE_SIZE_32KB) && (dev->chip->support.erase_32KB != 0))
		esize = 0x08000;
	else if ((blk_size == SF_ERASE_SIZE_4KB) && (dev->chip->support.erase_4KB != 0))
		esize = 0x01000;
	else {
		ret = HAL_INVALID;
		goto out;
	}

	if ((ret = HAL_MutexLock(&dev->lock, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto out;
	if ((ret = HAL_SPI_Open(dev->spi_n, dev->spi_cs_n, &dev->spi_config, SF_MAX_WAIT_TIME)) != HAL_OK)
		goto failed_open;

	while (blk_cnt-- > 0) {
		if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK) {
			SF_ALERT("Wait Flash Release BUSY/WIP timeout\n");
			goto failed_op;
		}

		SF_EnableWrite(dev->spi_n);

		if (blk_size == SF_ERASE_SIZE_CHIP) {
			if ((ret = SF_EraseChip(dev->spi_n)) != HAL_OK)
				goto failed_op;
		} else {
			if ((ret = SF_Erase(dev->spi_n, dev->chip->support, blk_size, addr)) != HAL_OK)
				goto failed_op;
		}

		SF_DisableWrite(dev->spi_n);

		addr += esize;
	}
	if ((ret = HAL_SF_WaitComplete(dev, SF_MAX_WAIT_TIME, xip)) != HAL_OK) {
		SF_ALERT("Wait Flash Release BUSY/WIP timeout\n");
		goto failed_op;
	}

failed_op:
	SF_DisableWrite(dev->spi_n);
	HAL_SPI_Close(dev->spi_n);
failed_open:
	HAL_MutexUnlock(&dev->lock);
out:

	if (xip) {
		HAL_XIP_Start();
		OS_ThreadResumeScheduler();
	}

	SF_EXIT_VAL(ret);
	return ret;
}

HAL_Status HAL_SF_MemoryOf(SF_Handler *hdl, SF_Erase_Size size, uint32_t addr, uint32_t *start)
{
	SF_Device *dev = *hdl;
//	uint32_t page_size;
	uint32_t page_mask;
//	uint32_t next_page = 0;
	HAL_Status ret = HAL_OK;

	SF_ENTRY();
	if ((size == SF_ERASE_SIZE_64KB) && (dev->chip->support.erase_64KB != 0))
		//page_size = 0x10000;
		page_mask = 0xFFFF0000;
	else if ((size == SF_ERASE_SIZE_32KB) && (dev->chip->support.erase_32KB != 0))
		//page_size = 0x08000;
		page_mask = 0xFFFF8000;
	else if ((size == SF_ERASE_SIZE_4KB) && (dev->chip->support.erase_4KB != 0))
		//page_size = 0x01000;
		page_mask = 0xFFFFF000;
	else {
		ret = HAL_INVALID;
		goto out;
	}

/*	while (next_page <= addr)
		next_page += page_size;
	*start = next_page - page_size;*/
	*start = addr & page_mask;

out:
	SF_EXIT_VAL(ret);
	return ret;
}
