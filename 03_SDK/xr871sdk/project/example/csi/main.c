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
#include "driver/chip/hal_csi.h"
#include "driver/chip/hal_dma.h"

void csi_stop_dma(void *arg)
{
	DMA_Channel ch = (DMA_Channel)arg;
	HAL_DMA_Stop(ch);
	HAL_DMA_Release(ch);
}

DMA_Channel csi_dma_reque()
{
	DMA_Channel ch;
	ch = HAL_DMA_Request();
	if (ch == DMA_CHANNEL_INVALID) {
		printf("dma error\n");
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
	param.endCallback = csi_stop_dma;
	param.irqType = DMA_IRQ_TYPE_END;
	HAL_DMA_Init(ch, &param);
	return ch;
}

volatile uint8_t private_frame_done = 0;
volatile uint32_t private_image_buff_addr = 0;
volatile uint32_t private_image_data_count = 0;

void set_save_image_buff(uint32_t image_buff_addr)
{
	private_image_data_count = 0;
	private_image_buff_addr = image_buff_addr;
}

void read_fifo_a(uint32_t len)
{
	if (private_image_buff_addr == 0) {
		printf("image_buff is invalid\n");
		return;
	}

	DMA_Channel csi_dma_ch = csi_dma_reque();
	HAL_DMA_Start(csi_dma_ch, CSI_FIFO_A,
	(private_image_buff_addr + private_image_data_count), len);

	private_image_data_count += len;
}

void read_fifo_b(uint32_t len)
{
	if (private_image_buff_addr == 0) {
		printf("image_buff is invalid\n");
		return;
	}

	DMA_Channel csi_dma_ch = csi_dma_reque();
	HAL_DMA_Start(csi_dma_ch, CSI_FIFO_B,
	(private_image_buff_addr + private_image_data_count), len);

	private_image_data_count += len;
}

void csi_callback(void *arg)
{
	uint32_t irq_sta = HAL_CSI_Interrupt_Sta();
	HAL_CSI_Interrupt_Clear();
	CSI_FIFO_Data_Len len = HAL_CSI_FIFO_Data_Len();

    if (irq_sta & CSI_FIFO_0_A_READY_IRQ)
		read_fifo_a(len.FIFO_0_A_Data_Len);
	else if (irq_sta & CSI_FIFO_0_B_READY_IRQ)
		read_fifo_b(len.FIFO_0_B_Data_Len);
	else if (irq_sta & CSI_FRAME_DONE_IRQ)
		private_frame_done = 1;
}

void csi_init()
{
	CSI_Config csi_cfg;
	HAL_CSI_Moudle_Enalbe(CSI_DISABLE);

	/*CSI mclk src clock (mclk is the clock for camera):CCM_AHB_PERIPH_CLK_SRC_HFCLK, 24mhz*/
	csi_cfg.src_Clk.clk =  CCM_AHB_PERIPH_CLK_SRC_HFCLK;
	/*The src clock div 2 (CCM_PERIPH_CLK_DIV_N_1 * CCM_PERIPH_CLK_DIV_M_2 = 1 * 2)*/
	/*The mclk is 12mhz.*/
	csi_cfg.src_Clk.divN = CCM_PERIPH_CLK_DIV_N_1;
	csi_cfg.src_Clk.divM = CCM_PERIPH_CLK_DIV_M_2;

	HAL_CSI_Config(&csi_cfg);

	CSI_Sync_Signal signal;
	/*Set the sync signal polarity*/
	signal.herf = CSI_POSITIVE;
	signal.p_Clk = CSI_POSITIVE;
	signal.vsync = CSI_POSITIVE;
	HAL_CSI_Sync_Signal_Polarity_Cfg(&signal);

	CSI_Picture_Size size;
	/*The picture size: one line 640 pixel*/
	size.hor_len = 640;
	/*Start capture form 0 pixel*/
	size.hor_start = 0;
	HAL_CSI_Set_Picture_Size(&size);
	HAL_CSI_Double_FIFO_Mode_Enable(CSI_ENABLE);

	HAL_CSI_Interrupt_Cfg(CSI_FRAME_DONE_IRQ, CSI_ENABLE);
	HAL_CSI_Interrupt_Cfg(CSI_FIFO_0_A_READY_IRQ, CSI_ENABLE);
	HAL_CSI_Interrupt_Cfg(CSI_FIFO_0_B_READY_IRQ, CSI_ENABLE);

	CSI_Call_Back cb;
	cb.arg = NULL;
	cb.callBack = csi_callback;
	HAL_CSI_Interrupt_Enable(&cb, CSI_ENABLE);
	HAL_CSI_Moudle_Enalbe(CSI_ENABLE);
	HAL_CSI_Capture_Enable(CSI_STILL_MODE, CSI_ENABLE);

}

void csi_reset_image_buff()
{
	private_image_data_count = 0;
}

uint8_t image_buff[153600];

/*Run this demo, please connect a camera, and config it.*/
int main(void)
{
	printf("csi demo start\n");

	csi_init();
	set_save_image_buff((uint32_t)image_buff);

	int i = 0;
	/*Wait for picture.*/
	while (private_frame_done == 0) {
		i++;
		if (i >= 10) {
			printf("capture picture time out\n");
			break;
		}
		OS_MSleep(100);
	}
	printf("csi capture one picture\n");

	csi_reset_image_buff();
	HAL_CSI_DeInit();

	printf("csi demo over\n");

	while (1) {
		OS_MSleep(10000);
	}

	return 0;
}
