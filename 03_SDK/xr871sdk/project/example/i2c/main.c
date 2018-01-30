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
#include "kernel/os/os.h"
#include "driver/chip/hal_i2c.h"

#define IIC_ID I2C0_ID
#define IIC_FREQ 400000

#define DEV_ADDR_BME280 0x76
#define DEV_MEM_ADDR 0xF5

/*Run this demo, please connect the sensor board.*/
int main(void)
{
	printf("i2c demo started\n\n");
	OS_MSleep(10);
	HAL_Status status = HAL_ERROR;

	I2C_InitParam initParam;
	initParam.addrMode = I2C_ADDR_MODE_7BIT;
	initParam.clockFreq = IIC_FREQ;

	status = HAL_I2C_Init(IIC_ID, &initParam);
	if (status != HAL_OK)
		printf("IIC init error %d\n", status);

	uint8_t send_data = 0x11;
	uint8_t read_data;

	/*write data to dirver mem*/
	printf("i2c write data 0x%02x\n", send_data);
	HAL_I2C_Master_Transmit_Mem_IT(IIC_ID, DEV_ADDR_BME280, DEV_MEM_ADDR, I2C_MEMADDR_SIZE_8BIT, &send_data, 1);
	OS_MSleep(5);
	/*read data for driver mem*/
	HAL_I2C_Master_Receive_Mem_IT(IIC_ID, DEV_ADDR_BME280, DEV_MEM_ADDR, I2C_MEMADDR_SIZE_8BIT, &read_data, 1);
	printf("i2c read data 0x%02x\n\n", read_data);

	status = HAL_I2C_DeInit(IIC_ID);
	if (status != HAL_OK)
		printf("IIC deinit error %d\n", status);

	printf("i2c demo over\n");

	while (1) {
		OS_MSleep(10000);
	}

	return 0;
}

