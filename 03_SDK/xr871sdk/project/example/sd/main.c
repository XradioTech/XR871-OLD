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
#include "driver/chip/hal_def.h"
#include "driver/chip/sdmmc/hal_sdhost.h"
#include "driver/chip/sdmmc/sdmmc.h"

#define MMC_DETECT_MODE         CARD_ALWAYS_PRESENT

struct mmc_card card;

#ifdef CONFIG_DETECT_CARD
static void card_detect(uint32_t present)
{
	if (present) {
		printf("card exist\n");
	} else {
		printf("card not exist\n");
	}
}
#endif

void sd_init()
{
	SDC_InitTypeDef sdc_param;
#ifdef CONFIG_DETECT_CARD
	uint32_t cd_mode = MMC_DETECT_MODE;

	sdc_param.cd_mode = cd_mode;
	sdc_param.cd_cb = &card_detect;
#endif
	HAL_SDC_Init(0, &sdc_param);
}

struct mmc_card *sd_card_scan()
{
	if (!mmc_rescan(&card, 0))
		return &card;
	else
		return NULL;
}

void sd_deinit()
{
	if (!card.host)
		return;

	mmc_card_deinit(&card);
	HAL_SDC_Deinit(0);
}

static uint8_t data_buf[513];

void sd_write()
{
	memset(data_buf, 0, 512);
	sprintf((char *)data_buf, "hello word!");
	if (mmc_block_write(&card, data_buf, 0, sizeof(data_buf) / 512) == -1)
		printf("sd write error\n");

	printf("write \"hello word!\" to sd card\n");
}

void sd_read()
{
	memset(data_buf, 0, 512);
	if (mmc_block_read(&card, data_buf, 0, sizeof(data_buf) / 512) == -1)
		printf("sd read error\n");

	printf("read data: %s\n", (uint8_t*)data_buf);
}

/*Run this demo, please connect the audio board, and insert tf card*/
int main(void)
{
	printf("sd demo started\n\n");

	sd_init();
	if (sd_card_scan() == NULL)
		printf("sd scan error\n");

	sd_write();
	sd_read();

	printf("\nsd demo over\n");

	while (1) {
		OS_MSleep(10000);
	}

	return 0;
}

