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

#include <stdio.h>
#include <string.h>

#include "kernel/os/os.h"

#include "common/framework/platform_init.h"

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/flashchip/flash_chip.h"
/*
 * W25Q16DV WRITE PROTECT BY SOFTWARE PROTECT DEMO
*/
#define MFLASH	0
#define FLASH_OPEN_TIMEOUT	(5000)
#define WP_IO2	2

#if 1
typedef enum W25Q16DV_FlashRegisterStatus1
{
	FLASH_REGS_BUSY 	= 1 << 0, //S0
	FLASH_REGS_WEL		= 1 << 1, //S1
	FLASH_REGS_BP0 		= 1 << 2,
	FLASH_REGS_BP1		= 1 << 3,
	FLASH_REGS_BP2		= 1 << 4,
	FLASH_REGS_TB		= 1 << 5,
	FLASH_REGS_SEC		= 1 << 6,
	FLASH_REGS_SRP0		= 1 << 7,//S7
} W25Q16DV_FlashRegisterStatus1;

typedef enum W25Q16DV_FlashRegisterStatus2
{
	FLASH_REGS_SRP1		= 1 << 0, //S8
	FLASH_REGS_QE		= 1 << 1, //S9
	FLASH_REGS_R 		= 1 << 2,//S10
	FLASH_REGS_LB1 		= 1 << 3,//S11
	FLASH_REGS_LB2 		= 1 << 4,//S12
	FLASH_REGS_LB3		= 1 << 5,//S13
	FLASH_REGS_CMP		= 1 << 6,
	FLASH_REGS_SUS		= 1 << 7,//S15
} W25Q16DV_FlashRegisterStatus2;
#endif

#define TEST_FLASH_BUF_SIZE (0x100)
static int flash_erase_ex(uint32_t addr, FlashEraseMode size_type)
{
	int32_t size;
	uint8_t buf[TEST_FLASH_BUF_SIZE];

	if (size_type == FLASH_ERASE_CHIP) {
		size = 0;
	} else if (size_type == FLASH_ERASE_64KB) {
		size = 0x10000;
	} else if (size_type == FLASH_ERASE_32KB) {
		size = 0x8000;
	} else if (size_type == FLASH_ERASE_4KB) {
		size = 0x1000;
	} else {
		printf("invalid size %x\n", size_type);
		return -1;
	}

	/* erase */
	HAL_Flash_MemoryOf(MFLASH, size_type, addr, &addr);
	if (HAL_Flash_Erase(MFLASH, size_type, addr, 1) != HAL_OK) {
		printf("flash erase failed\n");
		return -1;
	}

	while (size > 0) {
		int32_t tmp_size = (size < TEST_FLASH_BUF_SIZE) ? size : TEST_FLASH_BUF_SIZE;

		if (HAL_Flash_Read(MFLASH, addr, buf, tmp_size) != HAL_OK) {
			printf("flash read failed\n");
			return -1;
		}

		size -= tmp_size;
		addr += tmp_size;

		while (--tmp_size >= 0) {
			if ((uint8_t)(~(buf[tmp_size])) != 0) {
				printf("flash write failed: read data from flash != 0xFF, ~data = 0x%x, tmp_size = %d\n",
					(uint8_t)(~(buf[tmp_size])), tmp_size);
				return -1;
			}
		}

	}

	return 0;
}

static void FlashWriteProtectDemo(void) //w25q16dv demo
{
	int flash;
	uint8_t status_data;
	uint8_t status_data2;
	HAL_Status ret = HAL_ERROR;
	FlashControlStatus control_status1 = {
			.status = FLASH_STATUS1,
			.data = &status_data,
	};
	FlashControlStatus control_status2 = {
			.status = FLASH_STATUS2,
			.data = &status_data2,
	};
	int testdata1_len,testdata2_len;
	uint8_t testdata1[7] = {1,2,3,4,5,6,7};
	uint8_t testdata2[7] = {7,6,5,4,3,2,1};
	uint32_t addr;
	uint8_t *rbuf;
	int i;
	FlashEraseMode erase_type;

	testdata1_len = sizeof(testdata1);
	testdata2_len = sizeof(testdata2);
	rbuf = malloc(testdata1_len);
	addr = 0x1E2000; //test protect 30-31 block,1E0000h-1FFFFFh,128k
	flash = MFLASH;
	erase_type = FLASH_ERASE_4KB;

	if (HAL_Flash_Open(flash, FLASH_OPEN_TIMEOUT) != HAL_OK) {
		printf("open %d fail\n", MFLASH);
		return;
	}

	//erase addr content;after erase,the erased memory is all 0xff.
	//write testdata1 to addr, and read testdata1 from addr to check
	ret = flash_erase_ex(addr, erase_type);
	if(ret != HAL_OK) {
		printf("flash erase fail\n");
		goto out;
	}
	if (HAL_Flash_Write(MFLASH, addr, testdata1, testdata1_len) != HAL_OK) {
		printf("flash write failed\n");
		goto out;
	}
	if (HAL_Flash_Read(MFLASH, addr, rbuf, testdata1_len) != HAL_OK) {
		printf("flash read failed\n");
		goto out;
	}

	for(i=0; i < testdata1_len; i++) {
		printf("rbuf[%d]=%d\n", i, rbuf[i]);
		if(testdata1[i] != rbuf[i]) {
			printf("read content is not write content\n");
		}
	}

	//sw protect,set Status Register2 CMP 0;CMP default 0;
	ret = HAL_Flash_Control(flash,FLASH_READ_STATUS, (uint32_t)&control_status2);
	if(ret < 0) {
		printf("read register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	printf("status_data2=%x,%d\n", status_data2, __LINE__);
	status_data2 = status_data2 & (~FLASH_REGS_CMP);
	ret = HAL_Flash_Control(flash,FLASH_WRITE_STATUS, (uint32_t)&control_status2);
	if(ret < 0) {
		printf("write register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	ret = HAL_Flash_Control(flash,FLASH_READ_STATUS, (uint32_t)&control_status2);
	if(ret < 0) {
		printf("read register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	printf("status_data2=%x, %d\n", status_data2, __LINE__);

	//sw protect,set Status Register BP0/BP1/BP2/SEC/TB bit;SEC/TB default 0;
	ret = HAL_Flash_Control(flash,FLASH_READ_STATUS, (uint32_t)&control_status1);
	if(ret < 0) {
		printf("read register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	printf("status_data=%x,%d\n", status_data, __LINE__);
	//status_data = status_data | FLASH_REGS_BP1 | FLASH_REGS_BP2; //all 2M
	status_data = (status_data & (~FLASH_REGS_TB) & (~FLASH_REGS_SEC) & (~FLASH_REGS_BP0) &
	(~FLASH_REGS_BP2)) | FLASH_REGS_BP1; //30-31 block,1E000h-1FFFFFh,128k
	ret = HAL_Flash_Control(flash,FLASH_WRITE_STATUS, (uint32_t)&control_status1);
	if(ret < 0) {
		printf("write register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	ret = HAL_Flash_Control(flash,FLASH_READ_STATUS, (uint32_t)&control_status1);
	if(ret < 0) {
		printf("read register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	printf("status_data=%x, %d\n", status_data, __LINE__);

	//erase test,should fail
	ret = flash_erase_ex(addr, erase_type);
	if(ret == HAL_OK) {
		printf("write protect is fail\n");
		goto out;
	}

	//write protect test, write testdata2 to addr, check the addr content.
	if (HAL_Flash_Write(MFLASH, addr, testdata2, testdata2_len) != HAL_OK) {
		printf("flash write failed\n");
	}
	if (HAL_Flash_Read(MFLASH, addr, rbuf, testdata2_len) != HAL_OK) {
		printf("flash read failed\n");
		goto out;
	}

	for(i=0; i < testdata2_len; i++) {
		printf("rbuf[%d] = %d\n", i, rbuf[i]);
		if(rbuf[i] != testdata1[i]) {
			printf("write protect function is invalid\n");
			break;
		}
	}

	//delete write protect
	status_data = status_data & (~FLASH_REGS_BP0) & (~FLASH_REGS_BP1) & (~FLASH_REGS_BP2);//none
	ret = HAL_Flash_Control(flash,FLASH_WRITE_STATUS, (uint32_t)&control_status1);
	if(ret < 0) {
		printf("write register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	ret = HAL_Flash_Control(flash,FLASH_READ_STATUS, (uint32_t)&control_status1);
	if(ret < 0) {
		printf("read register status failed,ret=%d,line=%d\n", ret, __LINE__);
		goto out;
	}
	printf("status_data=%x, %d\n", status_data, __LINE__);

	// delete write protect test, write testdata2 to addr
	ret = flash_erase_ex(addr, erase_type);
	if(ret != HAL_OK) {
		printf("flash erase fail\n");
		goto out;
	}

	if (HAL_Flash_Write(MFLASH, addr, testdata2, testdata2_len) != HAL_OK) {
		printf("flash write failed\n");
		goto out;
	}

	if (HAL_Flash_Read(MFLASH, addr, rbuf, testdata2_len) != HAL_OK) {
		printf("flash read failed\n");
		goto out;
	}
	for(i=0; i < testdata2_len; i++) {
		printf("rbuf[%d]=%d\n", i, rbuf[i]);
		if( rbuf[i]!= testdata2[i]) {
			printf("delete write protect is not valid\n");
		}
	}

out:
	if (HAL_Flash_Close(flash) != HAL_OK) {
		printf("close %d fail\n", MFLASH);
	}
	 free(rbuf);
}

int main(void)
{
	platform_init();

	printf("Flash Write Protect demo started.\n\n");

	FlashWriteProtectDemo();

	printf("\nFlash Write Protect  demo over\n");
	return 0;
}
