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

#include "stdio.h"
#include "string.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_adc.h"
#include "driver/component/oled/drv_oled.h"
#include "oled_ui.h"

#define UI_DBG 1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_UI_DBG(fmt, arg...)	\
			LOG(UI_DBG, "[UI] "fmt, ##arg)

unsigned char bright_logo_16_16 [] = {
	0x00,0x00,0x00,0x00,0x0C,0x06,0x03,0x3B,0x3F,0x03,0x07,0x0C,0x08,0x00,0x00,0x00,
	0x00,0x00,0xC0,0xC0,0xCC,0xD8,0xE0,0x7E,0x3E,0xE0,0xD8,0xDC,0xCC,0xC0,0x00,0x00,
};

void ui_clear_screen(void)
{
	int i = 0;
	const uint8_t c[128] = {0};
	for (; i <= 5; i++)
    DRV_Oled_P8xnstr(0, i, c, 128);
}

int ui_d_time = 0;
int set_time = 0;
OS_Time_Info ui_os_time = {0, 0, 0};

void OS_Tick_Reset()
{
	ui_d_time = OS_JiffiesToSecs(OS_GetJiffies());
	set_time = 0;
}

void ui_set_time (uint32_t time)
{
	OS_Tick_Reset();
	set_time = time;
}

OS_Time_Info ui_current_time()
{
	return ui_os_time;
}

void ui_draw_time()
{
	char time_str[10] = {0};
	uint32_t time = OS_JiffiesToSecs(OS_GetJiffies()) - ui_d_time;
	time +=  set_time;

	ui_os_time.hour = time / 3600;
	ui_os_time.min = (time % 3600) / 60;
	ui_os_time.s = time % 3600 % 60;

	if (ui_os_time.hour >= 24) {
		OS_Tick_Reset();
		ui_os_time.hour = 0;
		ui_os_time.min = 0;
		ui_os_time.s = 0;
	}
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
	         ui_os_time.hour, ui_os_time.min, ui_os_time.s);
	DRV_Oled_Show_Str_1608(64, 0, time_str);

}

#define BRGHT_AD_CH ADC_CHANNEL_5
#define BIRGHTNESS_RESOLUTION 200
uint32_t ad_bright_value = 0;

void ui_birght_cb(void *arg)
{
	static uint32_t ad_value = 0, i = 0;
	uint32_t *value = (uint32_t *)arg;
	ad_value += HAL_ADC_GetValue(BRGHT_AD_CH);
	i++;
	if (i >= 10) {
		i = 0;
		*value = ad_value / 10;
		ad_value = 0;
		HAL_ADC_ConfigChannel(BRGHT_AD_CH, ADC_SELECT_ENABLE,
						  	  ADC_IRQ_LOW_HIGH,
						  	  *value - BIRGHTNESS_RESOLUTION,
						      *value + BIRGHTNESS_RESOLUTION);
	}
}

Component_Status ui_brightness_init()
{
	ADC_InitParam AD_initParam;
	AD_initParam.delay = 10;
	AD_initParam.freq = 500000;
	AD_initParam.mode = ADC_CONTI_CONV;

	HAL_Status sta = HAL_ADC_Init(&AD_initParam);

	if (sta == HAL_OK || sta == HAL_BUSY) {
		if (sta == HAL_OK)
			HAL_ADC_Start_Conv_IT();
	} else {
		COMPONENT_WARN("ADC init error %d\n", sta);
		return COMP_ERROR;
	}
	uint32_t ret = 0;

	ret = HAL_ADC_ConfigChannel(BRGHT_AD_CH, ADC_SELECT_ENABLE, ADC_IRQ_DATA, 0, 0);
	if (HAL_OK != ret) {
		COMPONENT_WARN("ADC config error %d\n", ret);
		return COMP_ERROR;
	}

	ret = HAL_ADC_EnableIRQCallback(BRGHT_AD_CH, ui_birght_cb, &ad_bright_value);
	if (HAL_OK != ret) {
		COMPONENT_WARN("ADC irq en error %d\n", ret);
		return COMP_ERROR;
	}

	COMPONENT_TRACK("end\n");
	return COMP_OK;
}


void ui_set_bright()
{
	if (ad_bright_value) {
		char bright_str[8];
		int bright = 255 - 255 * ((double)ad_bright_value / 4095.0 * 2);
		ad_bright_value = 0;
		if (bright < 1)
			bright = 1;
		snprintf(bright_str, sizeof(bright_str), " %03d", bright);
		DRV_Oled_Show_Str_1608(16, 0, bright_str);
		DRV_Oled_Set_Brightness(bright);
	}
}

void ui_brightness_deinit()
{
	HAL_ADC_ConfigChannel(BRGHT_AD_CH, ADC_SELECT_DISABLE, ADC_IRQ_NONE,
						  0, 0);
	HAL_ADC_DisableIRQCallback(BRGHT_AD_CH);
}

#define UI_THREAD_STACK_SIZE	512
OS_Thread_t ui_task_thread;
uint8_t ui_task_run = 0;

void ui_task(void *arg)
{
	uint32_t i = 0;
	while (ui_task_run) {
		OS_MSleep(200);
		DRV_Oled_Pnxm_Bmp(0, 0, 16, 16, bright_logo_16_16);
		ui_set_bright();
		ui_draw_time();
		i++;
	}
	COMPONENT_TRACK("end\n");
	OS_ThreadDelete(&ui_task_thread);
}

Component_Status ui_task_deinit()
{
	ui_task_run = 0;
	while((OS_ThreadIsValid(&ui_task_thread)))
		OS_MSleep(10);
	ui_brightness_deinit();
	DRV_Oled_DeInit();
	return COMP_OK;
}

#define OLED_SPI_MCLK  6000000

Component_Status ui_task_init()
{

	Oled_Config oled_cfg;
	oled_cfg.oled_SPI_ID = SPI1;
	oled_cfg.oled_SPI_MCLK = OLED_SPI_MCLK;
	oled_cfg.oled_SPI_CS = SPI_TCTRL_SS_SEL_SS0;
	oled_cfg.oled_dsPin = GPIO_PIN_1;
	oled_cfg.oled_dsPort = GPIO_PORT_A;
	oled_cfg.oled_reset_Pin = GPIO_PIN_16;
	oled_cfg.oled_reset_Port = GPIO_PORT_A;

	if (DRV_Oled_Init(&oled_cfg) != COMP_OK) {
		COMPONENT_WARN("oled init error\n");
		return COMP_ERROR;
	}
	if (ui_brightness_init() != COMP_OK) {
		COMPONENT_WARN("brightness init error\n");
		return COMP_ERROR;
	}
	ui_task_run = 1;
	if (OS_ThreadCreate(&ui_task_thread,
		                "",
		                ui_task,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                UI_THREAD_STACK_SIZE) != OS_OK) {
		COMPONENT_WARN("thread create error\n");
		return COMP_ERROR;
	}
	COMPONENT_TRACK("end\n");
	return COMP_OK;
}


