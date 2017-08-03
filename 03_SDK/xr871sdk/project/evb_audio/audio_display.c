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
#include "driver/component/oled/drv_oled.h"
#include "audio_display.h"

#define UI_DBG 1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_UI_DBG(fmt, arg...)	\
			LOG(UI_DBG, "[UI] "fmt, ##arg)

static uint32_t ui_time = 0;

void ui_reset_time()
{
	ui_time = 0;
}

void ui_draw_time(uint32_t period_ms)
{
	ui_time += period_ms;

	char time_str[15] = {0};
	uint32_t time = ui_time / 1000;
	uint32_t hours = time / 3600;
	uint32_t minutes = (time % 3600) / 60;
	uint32_t sec = time % 3600 % 60;
    sprintf(time_str, "%02d:%02d:%02d", hours, minutes, sec);
	DRV_Oled_Show_Str_1608(32, 0, time_str);

}

static char song_name_buf[400] = {0};
static char temp[17];

void ui_set_songs_name(char *name)
{
	memset(song_name_buf, ' ', 16);
	memcpy(&song_name_buf[15], name, strlen(name));
	song_name_buf[strlen(name) + 15] = 0;
}

void move_string_left(char *str, int mov)
{
    if (NULL == str || mov <= 0)
        return;
    char tmp[mov];
    int i;
    int len = strlen(str);
    if (len == 0)
        return;
    mov %= len;
    if (mov == 0)
        return;
    for (i = 0; i < sizeof tmp; i++)
        tmp[i] = str[i];
    tmp[i] = '\0';
    for (i = 0; i < len-mov; i++)
        str[i] = str[i+mov];
    for (; i < len; i++)
        str[i] = tmp[i-(len-mov)];
}

void ui_show_songs_name()
{
	memset(temp, ' ', 16);
	memcpy(temp, song_name_buf, 16);
	move_string_left(song_name_buf, 1);
	temp[16] = 0;
	DRV_Oled_Show_Str_1608(0, 3, temp);
}

void ui_show_volume(int volume)
{
	int vol = (double)volume / 31.0 * 100;
	char temp[11];

	sprintf(temp, "vol: %03d%%", vol);
	DRV_Oled_Show_Str_1608(3, 6, temp);
}

#define OLED_SPI_MCLK  6000000
Component_Status ui_init()
{
	Oled_Config oled_cfg;
	oled_cfg.oled_SPI_ID = SPI1;
	oled_cfg.oled_SPI_MCLK = OLED_SPI_MCLK;
	oled_cfg.oled_SPI_CS = SPI_TCTRL_SS_SEL_SS0;
	oled_cfg.oled_dsPin= GPIO_PIN_20;
	oled_cfg.oled_dsPort= GPIO_PORT_A;
	oled_cfg.oled_reset_Port= GPIO_PORT_A;
	oled_cfg.oled_reset_Pin= GPIO_PIN_8;

	if (DRV_Oled_Init(&oled_cfg) != COMP_OK) {
		COMPONENT_WARN("oled init error\n");
		return COMP_ERROR;
	}
	return COMP_OK;
}

void ui_de_init()
{
	DRV_Oled_DeInit();
}

