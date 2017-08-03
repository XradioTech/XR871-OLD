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
#include "kernel/os/os.h"
#include "common/framework/ctrl_msg.h"
#include "driver/component/ad_button/drv_ad_button.h"
#include "ad_button_task.h"


#define AD_BUTTON_CTRL_DBG		0
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_AD_BUTTON_CTRL_DBG(fmt, arg...)	\
			LOG(AD_BUTTON_CTRL_DBG, "[AD BUTTON CTRL] "fmt, ##arg)



/***********************************************************************

************************************************************************/
#define MAX_TIME 4294967295
#define AD_VALUE_DEVAATION 50

static AD_Button_Config AD_Button_Cfg = {
	ADC_CHANNEL_1,
	ADC_IRQ_LOW,
	3000,
	3500,
};


typedef enum {
	AD_BUTTON_0_VALUE = 2753,
	AD_BUTTON_1_VALUE = 1750,
	AD_BUTTON_2_VALUE = 829,
	AD_BUTTON_VALUE_NULL = -1,
}AD_BUTTON_VALUE;


typedef struct {
	uint16_t repeat_Time_Ms;
	uint16_t repeat_Period_Ms;
}AD_Button_RepeatPressMode;

typedef struct {
	AD_Button_RepeatPressMode *RepeatMode;
	uint16_t long_Press_hold_Time_Ms;
	uint16_t short_Press_hold_Time_Ms;
	AD_BUTTON_VALUE button_Ad_Value;
	AD_BUTTON_ID	button_Id;
}AD_Button_Info;

AD_Button_RepeatPressMode Ad_Button_2_Repeat = {700, 10};

AD_Button_Info AD_Button_Register[AD_BUTTON_NUM] = {
	{NULL, 0, 10, AD_BUTTON_0_VALUE, AD_BUTTON_0},
	{NULL, 0, 10, AD_BUTTON_1_VALUE, AD_BUTTON_1},
	{&Ad_Button_2_Repeat, 0, 10, AD_BUTTON_2_VALUE, AD_BUTTON_2},
};

static AD_Button_Cmd_Info AD_Button_Cmd;

/************************BUTTON CMD *************************************
*************************************************************************/
AD_BUTTON_ID AD_Button = AD_BUTTON_NUM;
uint8_t AD_Button_Is_Trigger = 0;
uint32_t AD_Button_Press_Time = 0;


/**************************************************************************
**************************************************************************/

int ad_button_send_vkey(AD_Button_Cmd_Info *data)
{
	struct ctrl_msg msg;
	msg.type = CTRL_MSG_TYPE_VKEY;
	msg.subtype = CTRL_MSG_SUB_TYPE_AD_BUTTON;
	msg.data = (uint32_t)data;
	if (ctrl_msg_send(&msg, 1) != 0) {
		COMPONENT_WARN("send vkey error\n");
		return -1;
	}
	return 0;
}

/**************************************************************************
**************************************************************************/
uint32_t ad_button_d_time(uint32_t time1, uint32_t time2)
{
	uint32_t d_time = 0;
	if (time1 <= time2)
		d_time = time2 - time1;
	else
		d_time = MAX_TIME - time1 + time2;
	return d_time;
}

uint32_t ad_d_Value(uint32_t v1, uint32_t v2)
{
	uint32_t d_value = 0;
	if (v1 > v2)
		d_value = v1 - v2;
	else
		d_value = v2 - v1;

	return d_value;
}


/**************************************************************************
**************************************************************************/

void ad_button_id_refresh(uint32_t ad_value)
{
	int16_t i = 0;
	uint32_t d_v = 0;
	AD_Button_Info *p = AD_Button_Register;

	for (i = 0; i < AD_BUTTON_NUM; i++) {
	 	d_v = ad_d_Value(ad_value, p->button_Ad_Value);
		if (d_v <= AD_VALUE_DEVAATION) {
			AD_Button = p->button_Id;
			return;
		}
		p ++;
	}
	AD_Button = AD_BUTTON_NUM;
}

/**********************************************************
***********************************************************/

void ad_button_Cb(void *arg, ADC_IRQState sta, uint32_t ad_value)
{
	if (sta == ADC_LOW_IRQ) {
		if (ad_value)
			ad_button_id_refresh(ad_value);
	}else if(sta == ADC_HIGH_IRQ)
		AD_Button = AD_BUTTON_ALL_RELEASE;
}

void ad_button_repeat(AD_Button_Info *button, uint32_t os_time)
{
	static uint32_t last_d_time = 0;
	if (button->RepeatMode != NULL) {
		uint32_t repeat_time = button->RepeatMode->repeat_Time_Ms;
		uint32_t repeat_period = button->RepeatMode->repeat_Period_Ms;
		uint32_t d_time = ad_button_d_time(AD_Button_Press_Time, os_time);
		if (d_time >= repeat_time) {
			AD_Button_Is_Trigger = 1;
			if (d_time - last_d_time >= repeat_period) {
				AD_Button_Cmd_Info * p = &AD_Button_Cmd;
				p->cmd = AD_BUTTON_CMD_REPEAT;
				p->id = button->button_Id;
				ad_button_send_vkey(p);
				last_d_time = d_time;
			}
		}
	}
}

void ad_button_long_press(AD_Button_Info *button, uint32_t os_time)
{
	if (button->long_Press_hold_Time_Ms > button->short_Press_hold_Time_Ms) {
		uint32_t d_time = ad_button_d_time(AD_Button_Press_Time, os_time);
		if (d_time >= button->long_Press_hold_Time_Ms && AD_Button_Is_Trigger == 0) {
			AD_Button_Cmd_Info * p = &AD_Button_Cmd;
			p->cmd = AD_BUTTON_CMD_LONG_PRESS;
			p->id = button->button_Id;
			ad_button_send_vkey(p);
			AD_Button_Is_Trigger = 1;
		}
	}
}

void ad_button_short_press(AD_Button_Info *button)
{
	if (AD_Button_Is_Trigger == 0) {
		uint32_t os_time = OS_JiffiesToMSecs(OS_GetJiffies());
		uint32_t d_time = ad_button_d_time(AD_Button_Press_Time, os_time);
		if (d_time >= button->short_Press_hold_Time_Ms) {
			AD_Button_Cmd_Info * p = &AD_Button_Cmd;
			p->cmd = AD_BUTTON_CMD_SHORT_PRESS;
			p->id = button->button_Id;
			ad_button_send_vkey(p);
		}
	}
}

void ad_button_check(AD_Button_Info *button)
{
	uint32_t os_time = OS_JiffiesToMSecs(OS_GetJiffies());
	ad_button_repeat(button, os_time);
	ad_button_long_press(button, os_time);
}

void ad_button_release_cmd (AD_Button_Info *button)
{
	if (AD_Button_Is_Trigger) {
		AD_Button_Cmd_Info * p = &AD_Button_Cmd;
		p->cmd = AD_BUTTON_CMD_RELEASE;
		p->id = button->button_Id;
		ad_button_send_vkey(p);
	}
}
void ad_button_ctrl_task(void *arg)
{
	DRV_AD_BUTTON_CTRL_DBG("%s\n", __func__);
	AD_BUTTON_ID button_id = AD_BUTTON_NUM;
	while (1) {
		AD_Button_Info *button_info = NULL;
		if (AD_Button != AD_BUTTON_NUM && AD_Button != AD_BUTTON_ALL_RELEASE) {
			if (AD_Button != button_id && AD_Button_Is_Trigger == 0) {
				button_id = AD_Button;
				DRV_AD_BUTTON_CTRL_DBG("AD_Button %d\n", AD_Button);
				AD_Button_Press_Time = OS_JiffiesToMSecs(OS_GetJiffies());
				DRV_AD_BUTTON_CTRL_DBG("AD_Button_Press_Time %d\n", AD_Button_Press_Time);
			}
		} else if (button_id != AD_BUTTON_NUM && AD_Button == AD_BUTTON_ALL_RELEASE) {
			//RELEASE
			DRV_AD_BUTTON_CTRL_DBG("release\n");
			button_info = &AD_Button_Register[button_id];
			ad_button_short_press(button_info);
			ad_button_release_cmd (button_info);
			button_id = AD_BUTTON_NUM;
			AD_Button_Is_Trigger = 0;
		}

		if (button_id != AD_BUTTON_NUM) {
			button_info = &AD_Button_Register[button_id];
			ad_button_check(button_info);
		}
		OS_MSleep(10);
	}
}

OS_Thread_t g_ad_button_ctrl_thread;
#define AD_BUTTON_CTRL_THREAD_STACK_SIZE	(1 * 1024)

void ad_button_init()
{
	AD_Button_Irq irq;
	DRV_AD_ButtonInit(&AD_Button_Cfg);
	irq.arg = NULL;
	irq.buttonCallback = ad_button_Cb;
	DRV_AD_ButtonCallBackRegister(&irq);
	if (OS_ThreadCreate(&g_ad_button_ctrl_thread,
						"",
						ad_button_ctrl_task,
						NULL,
						OS_THREAD_PRIO_APP,
						AD_BUTTON_CTRL_THREAD_STACK_SIZE) != OS_OK) {
		COMPONENT_WARN("thread create error\n");
	}
	COMPONENT_TRACK("end\n");
}

void ad_button_deinit()
{
	DRV_AD_ButtonDeInit();
	OS_ThreadDelete(&g_ad_button_ctrl_thread);
	COMPONENT_TRACK("end\n");
}
#if 1
void ad_button_ctrl(AD_Button_Cmd_Info *button_cmd)
{
	printf("button[%d]", button_cmd->id);
	switch(button_cmd->cmd) {
		case AD_BUTTON_CMD_LONG_PRESS :
			printf("long press\n");
			break;
		case AD_BUTTON_CMD_SHORT_PRESS :
			printf("short press\n");
			break;
		case AD_BUTTON_CMD_REPEAT :
			printf("repeat press\n");
			break;
		case AD_BUTTON_CMD_RELEASE :
			printf("release press\n");
			break;
		default :
			break;
	}
}
#endif

