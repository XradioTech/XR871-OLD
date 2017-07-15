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
#include "common/ctrl_msg/ctrl_msg.h"
#include "driver/component/gpio_button/drv_gpio_button.h"
#include "kernel/os/os.h"
#include "gpio_button_task.h"

#define GPIO_BUTTON_CTRL_DBG		0
#define HAL_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_GPIO_BUTTON_CTRL_DBG(fmt, arg...)	\
			//HAL_LOG(GPIO_BUTTON_CTRL_DBG, "[GPIO BUTTON CTRL] "fmt, ##arg)
/*****************************************************************************
*************************io register********************************************/
#define BUTTON_IO_REGISTER_LEN 2
static GPIO_Button_IO Button_Io_Register[BUTTON_IO_REGISTER_LEN] = {
	/*BUTTON 0 IO*/
	{GPIO_PORT_A, GPIO_PIN_6, HIGH_LEVEL},
	/*BUTTON 1 IO*/
	{GPIO_PORT_A, GPIO_PIN_7, HIGH_LEVEL},
};

/********************************************************************************
*********************************************************************************/

typedef struct {
	uint32_t repeat_Press_Hold_Time_Ms;
	uint32_t repeat_Period_Ms;
}Gpio_Button_Repeat;

typedef struct {
	uint32_t button_Id;
	uint32_t button_Value;
	uint32_t short_Press_Hold_Time_Ms;
	uint32_t long_Press_Hold_Time_Ms;
	Gpio_Button_Repeat *repeat_Mode;
}Gpio_Button_Info;

typedef enum {
	GPIO_BUTTON_PRESS_EVENT,
	GPIO_BUTTON_RELEASE_EVENT,
	GPIO_BUTTON_NULL_EVENT,
}GPIO_BUTTON_EVENT;

#define MAX_TIME 4294967295
static uint32_t gpio_button_combination = 0;
static GPIO_BUTTON_EVENT gpio_button_event = GPIO_BUTTON_NULL_EVENT;

Gpio_Button_Repeat Button_1_Repeat = {700, 10};

static Gpio_Button_Info gpio_button[BUTTON_NUM] = {
/*button_0*/
{BUTTON_0, (1 << BUTTON_0), 10, 0, NULL},
/*button_1*/
{BUTTON_1, (1 << BUTTON_1), 10, 0, &Button_1_Repeat},
};

static GPIO_Button_Cmd_Info gpio_button_cmd;

/********************************************************************************
*********************************************************************************/
static int gpio_button_send_vkey(GPIO_Button_Cmd_Info *cmd)
{
	struct ctrl_msg msg;
	msg.type = CTRL_MSG_TYPE_VKEY;
	msg.subtype = CTRL_MSG_SUB_TYPE_GPIO_BUTTON;
	msg.data = (uint32_t)cmd;
	if (ctrl_msg_send(&msg, 1) != 0) {
		DRV_GPIO_BUTTON_CTRL_DBG("send vkey error\n");
		return 0;
	}
	return 1;
}

static uint32_t gpio_button_value(uint32_t button_id)
{
	return (1 << button_id);
}

static uint32_t gpio_button_d_time(uint32_t time1, uint32_t time2)
{
	uint32_t d_time = 0;
	if (time1 <= time2)
		d_time = time2 - time1;
	else
		d_time = MAX_TIME - time1 + time2;
	return d_time;
}

static uint8_t Button_Func_Is_Trigger = 0;

static void gpio_button_repeat(Gpio_Button_Info *button, uint32_t button_press_time, uint32_t os_time)
{
	static uint32_t last_d_time = 0;
	if (button->repeat_Mode != NULL) {
		uint32_t repeat_hold_time = button->repeat_Mode->repeat_Press_Hold_Time_Ms;
		uint32_t repeat_period = button->repeat_Mode->repeat_Period_Ms;
		uint32_t d_time = gpio_button_d_time(button_press_time, os_time);

		if (d_time >= repeat_hold_time) {
			Button_Func_Is_Trigger = 1;
			if (d_time - last_d_time >= repeat_period) {
				GPIO_Button_Cmd_Info * p = &gpio_button_cmd;
				p->cmd = GPIO_BUTTON_CMD_REPEAT;
				p->id = button->button_Id;
				gpio_button_send_vkey(p);
				last_d_time = d_time;
			}
		}
	}
}

static void gpio_button_long_press(Gpio_Button_Info *button, uint32_t button_press_time,uint32_t os_time)
{
	if (button->long_Press_Hold_Time_Ms > button->short_Press_Hold_Time_Ms) {
		uint32_t d_time = gpio_button_d_time(button_press_time, os_time);

		if (d_time >= button->long_Press_Hold_Time_Ms && Button_Func_Is_Trigger == 0) {
			GPIO_Button_Cmd_Info * p = &gpio_button_cmd;
			p->cmd = GPIO_BUTTON_CMD_LONG_PRESS;
			p->id = button->button_Id;
			gpio_button_send_vkey(p);
			Button_Func_Is_Trigger = 1;
		}
	}
}

static void gpio_button_short_press(Gpio_Button_Info *button, uint32_t button_press_time)
{
	if (Button_Func_Is_Trigger == 0) {
		uint32_t os_time = OS_JiffiesToMSecs(OS_GetJiffies());
		uint32_t d_time = gpio_button_d_time(button_press_time, os_time);
		if (d_time >= button->short_Press_Hold_Time_Ms) {
			GPIO_Button_Cmd_Info * p = &gpio_button_cmd;
			p->cmd = GPIO_BUTTON_CMD_SHORT_PRESS;
			p->id = button->button_Id;
			gpio_button_send_vkey(p);
		}
	}
}

static void gpio_button_release_cmd (Gpio_Button_Info *button)
{
	if (Button_Func_Is_Trigger) {
		GPIO_Button_Cmd_Info * p = &gpio_button_cmd;
		p->cmd = GPIO_BUTTON_CMD_RELEASE;
		p->id = button->button_Id;
		gpio_button_send_vkey(p);
	}
}

static void gpio_button_check(Gpio_Button_Info *button,  uint32_t button_press_time)
{
	uint32_t os_time = OS_JiffiesToMSecs(OS_GetJiffies());
	gpio_button_repeat(button, button_press_time, os_time);
	gpio_button_long_press(button, button_press_time, os_time);
}

static void gpio_button_cb(void *arg, GPIO_IrqEvent Irq_Edge)
{
	uint32_t id = (uint32_t)arg;

	if(Irq_Edge == GPIO_IRQ_EVT_FALLING_EDGE) { //press event
		gpio_button_event = GPIO_BUTTON_PRESS_EVENT;
		gpio_button_combination += gpio_button_value(id);
		//DRV_GPIO_BUTTON_CTRL_DBG("%s, %d, press event; gpio_button_combination = %d\n",
		//						 __func__, __LINE__, gpio_button_combination);
	} else { //release event
		gpio_button_event = GPIO_BUTTON_RELEASE_EVENT;
		gpio_button_combination -= gpio_button_value(id);
		//DRV_GPIO_BUTTON_CTRL_DBG("%s, %d, release event; gpio_button_combination = %d\n",
		//						 __func__, __LINE__, gpio_button_combination);
	}
}

static void gpio_button_cb_init()
{

	GPIO_Button_Irq irq;
	irq.arg = (void *)BUTTON_0;
	irq.buttonCallBack = gpio_button_cb;
	DRV_GPIO_ButtonCallBackRegister(BUTTON_0, &irq);
	irq.arg = (void *)BUTTON_1;
	DRV_GPIO_ButtonCallBackRegister(BUTTON_1, &irq);
}

typedef struct {
	GPIO_BUTTON_EVENT event;
	uint32_t button_Value;
} Gpio_Button_Event;

static Gpio_Button_Event gpio_button_event_handle()
{
	Gpio_Button_Event button_event;
	button_event.event = GPIO_BUTTON_NULL_EVENT;
	button_event.button_Value= 0;

	if (gpio_button_event == GPIO_BUTTON_PRESS_EVENT) {
		button_event.event = GPIO_BUTTON_PRESS_EVENT;
		button_event.button_Value = gpio_button_combination;
	} else if (gpio_button_event == GPIO_BUTTON_RELEASE_EVENT) {
		button_event.event = GPIO_BUTTON_RELEASE_EVENT;
		button_event.button_Value = gpio_button_combination;
	}
	gpio_button_event = GPIO_BUTTON_NULL_EVENT;
	return button_event;
}

typedef struct {
	GPIO_BUTTON_EVENT event;
	GPIO_BUTTON_ID button_Id;
} Gpio_Button_Report;

static Gpio_Button_Report gpio_button_report()
{
	Gpio_Button_Event event = gpio_button_event_handle();
	Gpio_Button_Report report = {GPIO_BUTTON_NULL_EVENT, BUTTON_NUM};
	static uint32_t button_value = 0;

	int i = 0;
	if (event.event == GPIO_BUTTON_PRESS_EVENT) {
		if (button_value != event.button_Value) {
			report.event = GPIO_BUTTON_PRESS_EVENT;
			for(i = 0; i < BUTTON_NUM; i++) {
				if(event.button_Value == gpio_button[i].button_Value) {
					button_value = event.button_Value;
					report.button_Id = gpio_button[i].button_Id;
					return report;
				}
			}
			report.button_Id = BUTTON_NUM;
		}
	} else if (event.event == GPIO_BUTTON_RELEASE_EVENT) {
		if ((button_value & event.button_Value) < button_value) {
			for(i = 0; i < BUTTON_NUM; i++) {
				if(button_value == gpio_button[i].button_Value) {
					report.event = GPIO_BUTTON_RELEASE_EVENT;
					report.button_Id = gpio_button[i].button_Id;
				}
			}
			button_value = 0;
		}
	}
	return report;
}

static OS_Thread_t g_button_ctrl_thread;
#define BUTTON_CTRL_THREAD_STACK_SIZE	(1 * 1024)
static uint8_t button_ctrl_run = 0;

void gpio_button_ctrl_task(void *arg)
{
	DRV_GPIO_BUTTON_CTRL_DBG("button_ctrl_task\n");
	uint32_t button_press_time = 0;
	Gpio_Button_Info *button = NULL;
	while (button_ctrl_run) {
		Gpio_Button_Report report = gpio_button_report();

		if (report.event == GPIO_BUTTON_PRESS_EVENT) {
			if (report.button_Id < BUTTON_NUM) {
				DRV_GPIO_BUTTON_CTRL_DBG("%s, press button[%d]\n",
					                     __func__, report.button_Id);
				button_press_time = OS_JiffiesToMSecs(OS_GetJiffies());
				button = &gpio_button[report.button_Id];
			} else {
				DRV_GPIO_BUTTON_CTRL_DBG("%s, press  button[null]\n",
				                     __func__);

				button = NULL;
				continue;
			}
		} else if (report.event == GPIO_BUTTON_RELEASE_EVENT) {
			DRV_GPIO_BUTTON_CTRL_DBG("%s, release button[%d]\n",
				                     __func__, report.button_Id);
			button = &gpio_button[report.button_Id];
			gpio_button_short_press(button, button_press_time);
			gpio_button_release_cmd (button);
			button = NULL;
			Button_Func_Is_Trigger = 0;

		}

		if (button != NULL)
			gpio_button_check(button, button_press_time);

		OS_MSleep(10);
	}
	OS_ThreadDelete(&g_button_ctrl_thread);
}

void gpio_button_ctrl_init(void)
{
	DRV_GPIO_BUTTON_CTRL_DBG("button_ctrl_init\n");
	DRV_Board_GPIO_Button_Cfg(BUTTON_INIT, Button_Io_Register, BUTTON_IO_REGISTER_LEN);

	gpio_button_cb_init();
	button_ctrl_run = 1;
	/* start system control task */
	if (OS_ThreadCreate(&g_button_ctrl_thread,
	                    "",
	                    gpio_button_ctrl_task,
	                    NULL,
	                    OS_THREAD_PRIO_APP,
	                    BUTTON_CTRL_THREAD_STACK_SIZE) != OS_OK) {
		DRV_GPIO_BUTTON_CTRL_DBG("create button ctrl task failed\n");
	}
		COMPONENT_TRACK("end\n");
}

void gpio_button_ctrl_deinit(void)
{
	button_ctrl_run = 0;

	while (OS_ThreadIsValid(&g_button_ctrl_thread))
		OS_MSleep(10);
	gpio_button_combination = 0;
	gpio_button_event = GPIO_BUTTON_NULL_EVENT;
	DRV_Board_GPIO_Button_Cfg(BUTTON_DEINIT, Button_Io_Register, BUTTON_IO_REGISTER_LEN);
}

#if 1
void gpio_button_ctrl(GPIO_Button_Cmd_Info *button_cmd)
{
	printf("gpio button[%d]", button_cmd->id);
	switch(button_cmd->cmd) {
		case GPIO_BUTTON_CMD_LONG_PRESS :
			printf("long press\n");
			break;
		case GPIO_BUTTON_CMD_SHORT_PRESS :
			printf("short press\n");
			break;
		case GPIO_BUTTON_CMD_REPEAT :
			printf("repeat press\n");
			break;
		case GPIO_BUTTON_CMD_RELEASE :
			printf("release press\n");
			break;
		default :
			break;
	}
}
#endif
