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
#include "driver/chip/hal_dma.h"

void Stop_Dma(void *arg)
{
	DMA_Channel ch = (DMA_Channel)arg;
	HAL_DMA_Stop(ch);
	HAL_DMA_Release(ch);
}

DMA_Channel Dma_Reque()
{
	DMA_Channel ch;
	/*request a free channel.*/
	ch = HAL_DMA_Request();
	if (ch == DMA_CHANNEL_INVALID) {
		return DMA_CHANNEL_INVALID;
	}

	DMA_ChannelInitParam param;
	param.cfg =  HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
											DMA_WAIT_CYCLE_1,
											DMA_BYTE_CNT_MODE_NORMAL,

			                                DMA_DATA_WIDTH_32BIT,
			                                DMA_BURST_LEN_4,
											DMA_ADDR_MODE_INC,
			                                DMA_PERIPH_SRAM,

											DMA_DATA_WIDTH_32BIT,
											DMA_BURST_LEN_4,
											DMA_ADDR_MODE_INC,
											DMA_PERIPH_SRAM);
	param.endArg = (void *)ch;
	param.endCallback = Stop_Dma;
	param.irqType = DMA_IRQ_TYPE_END;
	HAL_DMA_Init(ch, &param);
	return ch;
}

uint32_t src_addr[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
uint32_t dest_addr[12];

/*This demo using dma to copy data to another memory.*/
int main(void)
{
	printf("dma demo started\n\n");

	DMA_Channel ch = Dma_Reque();
	if (ch == DMA_CHANNEL_INVALID) {
		printf("DMA reque error\n");
		return 1;
	}

	HAL_DMA_Start(ch, (uint32_t)src_addr, (uint32_t)dest_addr, sizeof(src_addr));
	OS_MSleep(10);

	int i = 0;
	printf("dest_addr[] = {");
	while(i < 12)
		printf(" %d,", dest_addr[i++]);
	printf("}\n\n");

	HAL_DMA_DeInit(ch);

	printf("dma demo over\n");

	while (1) {
		OS_MSleep(10000);
	}

	return 0;
}

