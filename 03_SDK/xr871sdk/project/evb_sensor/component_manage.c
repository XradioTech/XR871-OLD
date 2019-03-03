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

#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"

#include "pm/pm.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/component/oled/drv_oled.h"
#include "driver/component/bme280/drv_bme280.h"
#include "driver/component/motor/drv_motor_ctrl.h"
#include "driver/component/rgb_led/drv_rgb_led.h"
#include "smartlink/sc_assistant.h"
#include "smartlink/airkiss/wlan_airkiss.h"

#include "common/framework/net_ctrl.h"

#include "oled_ui.h"
#include "component_manage.h"
#include "command.h"
#include "sensor_buttons.h"

#include "common/framework/sys_ctrl/sys_ctrl.h"


#define COMPONENT_MAIN_DBG 0
#define HAL_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define HAL_COMP_MAIN_DBG(fmt, arg...)	\
			//HAL_LOG(COMPONENT_MAIN_DBG, "[HAL COMP_MAIN] "fmt, ##arg)


typedef struct {
	void(*child0_menu)(void *child0_arg);
	void *child0_arg;
	void(*child1_menu)(void *child1_arg);
	void *child1_arg;
	void(*child2_menu)(void *child2_arg);
	void *child2_arg;
	char cursor_postion;
}ui_menu;

typedef enum {
	CURSOR_DOWN,
	CURSOR_UP,
}CURSOR_CTRL;

#define BME280_I2C_ID I2C0_ID
#define BME280_I2C_FREQ 400000

COMPONENT_CTRL sensor_button_cmd = COMPONENT_CTRL_NULL;

COMPONENT_CTRL read_component_ctrl_cmd()
{
	COMPONENT_CTRL cmd = COMPONENT_CTRL_NULL;

	if (sensor_button_cmd != COMPONENT_CTRL_NULL) {
		cmd = sensor_button_cmd;
		sensor_button_cmd = COMPONENT_CTRL_NULL;
	}

	return cmd;
}

Component_Status set_cursor_postion(char postion)
{
	char pos1 = 0, pos2 = 0;
	if (postion > 3 )
		postion = 3;
	if (postion < 1)
		postion = 1;

	if (postion == 1) {
		pos1 = 4;
		pos2 = 6;
	} else if (postion == 2) {
		pos1 = 2;
		pos2 = 6;
	} else if (postion == 3) {
		pos1 = 2;
		pos2 = 4;
	}
	DRV_Oled_Show_Str_1608(0, pos1, " ");
	DRV_Oled_Show_Str_1608(0, pos2, " ");
	DRV_Oled_Show_Str_1608(0, postion * 2, ">");
	return COMP_OK;
}

void move_cursor_ctrl(char *cursor_postion, CURSOR_CTRL ctrl)
{
	if (ctrl == CURSOR_UP) {
		*cursor_postion += 1;
		if (*cursor_postion > 3)
			*cursor_postion = 1;
	}else {
		*cursor_postion -= 1;
		if(*cursor_postion < 1)
			*cursor_postion = 3;
	}
	set_cursor_postion(*cursor_postion);
}

void menu_into(ui_menu *data)
{
	if (data->cursor_postion == 1) {
		HAL_COMP_MAIN_DBG("into child0 enum\n");
		if (data->child0_menu != NULL)
			data->child0_menu(data->child0_arg);
	} else if (data->cursor_postion == 2) {
		HAL_COMP_MAIN_DBG("into child1 enum\n");
		if (data->child1_menu != NULL)
			data->child1_menu(data->child1_arg);
	} else if (data->cursor_postion == 3) {
		HAL_COMP_MAIN_DBG("into child2 enum\n");
		if (data->child2_menu != NULL)
			data->child2_menu(data->child2_arg);
	}
}

void component_sleep()
{
	HAL_COMP_MAIN_DBG("%s, %d\n", __func__, __LINE__);
	sensor_buttons_deinit();
	DRV_BME280_Disable();
	DRV_Oled_OnOff(0);
	ui_task_deinit();
	pm_enter_mode(PM_MODE_STANDBY); /* PM_MODE_STANDBY/HIBERNATION */
	OS_MSleep(1);
	sensor_buttons_init();
	ui_task_init();
	DRV_BME280_Enable(BME280_I2C_ID, BME280_I2C_FREQ);
	DRV_BME280_Sleep();
	HAL_COMP_MAIN_DBG("%s, %d\n", __func__, __LINE__);
}

/*************************************************************
						BME280
**************************************************************/
void bme280_temper()
{
	ui_clear_screen();
	while (1) {
		uint32_t temperature = DRV_BME280_Read(TEMPERATURE);
		float value = (float)temperature / 100.0;
		char temp[10];
		sprintf(temp, "%.2f C", value);
		DRV_Oled_Show_Str_1608(16, 2, "TEMPER");
		DRV_Oled_Show_Str_1608(16, 4, temp);

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_BREAK :
					ui_clear_screen();
					return;
				default :
				break;
			}
		} else
		OS_MSleep(50);
	}
}

void bme280_pressure()
{
	ui_clear_screen();
	while (1) {
		uint32_t pressure = DRV_BME280_Read(PRESSURE);
		float value = (float)pressure / 100.0;
		char temp[10];
		sprintf(temp, "%.2f hpa", value);
		DRV_Oled_Show_Str_1608(16, 2, "PRESSURE");
		DRV_Oled_Show_Str_1608(16, 4, temp);

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_BREAK :
					ui_clear_screen();
					return;
				default :
				break;
			}
		} else
			OS_MSleep(50);
	}
}

void bme280_humidity()
{
	ui_clear_screen();
	while (1) {
		uint32_t humidity = DRV_BME280_Read(HUMIDITY);
		char temp[5];
		sprintf(temp, "%04u", humidity);
		DRV_Oled_Show_Str_1608(16, 2, "HUMIDITY");
		DRV_Oled_Show_Str_1608(16, 4, temp);

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_BREAK :
					ui_clear_screen();
					return;
				default :
				break;
			}
		} else
		OS_MSleep(50);
	}
}

ui_menu menu_bme280 = {
	bme280_temper,
	NULL,
	bme280_pressure,
	NULL,
	bme280_humidity,
	NULL,
	1,
};

void bme280_menu_1608str(void *arg)
{
	ui_menu *data = (ui_menu *)arg;
	HAL_COMP_MAIN_DBG("%s\n", __func__);
	ui_clear_screen();
	DRV_BME280_WakeUp();
	set_cursor_postion(data->cursor_postion);
	while (1) {
		DRV_Oled_Show_Str_1608(16, 2, "TEMPER");
		DRV_Oled_Show_Str_1608(16, 4, "PRESSURE");
		DRV_Oled_Show_Str_1608(16, 6, "HUMIDITY");

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_INTO :
					menu_into(data);
					set_cursor_postion(data->cursor_postion);
					break;
				case COMPONENT_CTRL_BREAK :
					ui_clear_screen();
					DRV_BME280_Sleep();
					return;
				case COMPONENT_CTRL_UP :
					move_cursor_ctrl(&data->cursor_postion, CURSOR_UP);
					break;
				case COMPONENT_CTRL_DOWN :
					move_cursor_ctrl(&data->cursor_postion, CURSOR_DOWN);
					break;
				case COMPONENT_CTRL_SLEEP :
					//DRV_Oled_OnOff(0);
					break;
				default :
				break;
			}
		}else
			OS_MSleep(50);
	}
}


static Rgb_Led_Value rgb_colour[7] = { {1, 0, 0}, //red
								{0, 1, 0}, //green
								{0, 0, 1}, //blue
								{1, 1, 0}, //yellow
								{0, 1, 1}, //turquoise
								{1, 0, 1}, //azaleine
								{1, 1, 1}, //white
							};
void Rgb_IO_Pull_Down()
{
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F1_OUTPUT;
	param.pull = GPIO_PULL_DOWN;
	HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_8, &param);
	HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_9, &param);
	HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_10, &param);
	HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_8, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_9, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_10, GPIO_PIN_LOW);
}

void Rgb_Led()
{

	Rgb_Led_Info led_info;
	led_info.b_Led = PWM_GROUP0_CH0;
	led_info.g_Led = PWM_GROUP0_CH1;
	led_info.r_Led = PWM_GROUP1_CH2;
	led_info.ledFrequency = 500;
	led_info.type = RGB_HIGH_LEVEL_VALID;

	MaxBrightness maxBrightness = Drv_Rgb_Led_Cfg(&led_info);
	Drv_Rgb_LedEnable();

	int brightness = 0;
	char temp[5];
	sprintf(temp, "%04d", brightness);
	ui_clear_screen();
	DRV_Oled_Show_Str_1608(16, 2, "birghtness");
	DRV_Oled_Show_Str_1608(16, 4, temp);
	uint32_t count = 0, colour_count = 0;
	while (1) {
		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_BREAK :
					Drv_Rgb_LedDisable();
					Drv_Rgb_Led_DeInit();
					Rgb_IO_Pull_Down();
					ui_clear_screen();
					return;
				case COMPONENT_CTRL_UP :
					brightness += 10;
					if(brightness > maxBrightness)
						brightness = maxBrightness;
					sprintf(temp, "%04d", brightness);
					DRV_Oled_Show_Str_1608(16, 4, temp);
					break;
				case COMPONENT_CTRL_DOWN :
					brightness -= 10;
					if(brightness < 0)
						brightness = 0;
					sprintf(temp, "%04d", brightness);
					DRV_Oled_Show_Str_1608(16, 4, temp);
					break;
				default :
					break;
			}
		} else
			OS_MSleep(50);

		count ++;
		if(count >= 40) {
			count = 0;
			colour_count++;
			if(colour_count >= 7)
				colour_count = 0;

			Rgb_Led_Value colour;
			colour.r_Value = brightness * rgb_colour[colour_count].r_Value;
			colour.g_Value = brightness * rgb_colour[colour_count].g_Value;
			colour.b_Value = brightness * rgb_colour[colour_count].b_Value;
			Drv_Rgb_Led_Set(&colour);
		}
	}
}


void motor_ctrl()
{
	Motor_Ctrl motor_ctrl= {10000, PWM_GROUP3_CH6, MOTOR_HIGH_LEVEL};
	uint32_t max_speed = DRV_Motor_Ctrl_Init(&motor_ctrl);
	int speed = 0;
	DRV_Motor_Enable();
	ui_clear_screen();

	char temp[5];
	sprintf(temp, "%04d", speed);
	DRV_Oled_Show_Str_1608(16, 2, "SPEED");
	DRV_Oled_Show_Str_1608(16, 4, temp);
	while (1) {
		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_BREAK :
					DRV_Motor_Speed_Ctrl(0);
					DRV_Motor_Ctrl_DeInit();
					ui_clear_screen();
					return;
				case COMPONENT_CTRL_UP :
					speed += 10;
					if(speed > max_speed)
						speed = max_speed;
					DRV_Motor_Speed_Ctrl(speed);
					sprintf(temp, "%04d", speed);
					DRV_Oled_Show_Str_1608(16, 4, temp);
					break;
				case COMPONENT_CTRL_DOWN :
					speed -= 10;
					if(speed < 0)
						speed = 0;
					DRV_Motor_Speed_Ctrl(speed);
					sprintf(temp, "%04d", speed);
					DRV_Oled_Show_Str_1608(16, 4, temp);
					break;
				default :
					break;
			}
		} else
			OS_MSleep(50);
	}
}

ui_menu sensor_func = {
	bme280_menu_1608str,
	&menu_bme280,

	motor_ctrl,
	NULL,

	Rgb_Led,
	NULL,

	1,
};

void Sensor_Func(void *arg)
{
	ui_menu *data = (ui_menu *)arg;
	ui_clear_screen();
	set_cursor_postion(data->cursor_postion);
	while (1) {
		DRV_Oled_Show_Str_1608(16, 2, "BME280");
		DRV_Oled_Show_Str_1608(16, 4, "MOTOR");
		DRV_Oled_Show_Str_1608(16, 6, "RGB");

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if ( cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_INTO :
					menu_into(data);
					set_cursor_postion(data->cursor_postion);
					break;
				case COMPONENT_CTRL_BREAK :
					ui_clear_screen();
					return;
				case COMPONENT_CTRL_UP :
					move_cursor_ctrl(&data->cursor_postion, CURSOR_UP);
					break;
				case COMPONENT_CTRL_DOWN :
					move_cursor_ctrl(&data->cursor_postion, CURSOR_DOWN);
					break;
				case COMPONENT_CTRL_SLEEP:
					//component_sleep(NULL);
					break;
				default :
				break;
			}
		}else
		OS_MSleep(50);
	}
}

typedef enum {
	TIME_H = 1,
	TIME_M = 2,
	TIME_S = 3,
}OS_TIME_SET_MODE;

void Set_Time(OS_TIME_SET_MODE MODE, OS_Time_Info *data)
{
	while(1) {
		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd == COMPONENT_CTRL_UP) {
			if (MODE == TIME_H) {
				data->hour++;
				if (data->hour >= 24)
					data->hour = 0;
			} else if (MODE == TIME_M) {
				data->min++;
				if (data->min >= 60)
					data->min = 0;
			} else {
				data->s++;
				if (data->s >= 60)
					data->s = 0;
			}
		} else if (cmd == COMPONENT_CTRL_DOWN) {
			if (MODE == TIME_H) {
				data->hour--;
				if (data->hour < 0)
					data->hour = 23;
			} else if (MODE == TIME_M) {
				data->min--;
				if (data->min < 0)
					data->min = 59;
			} else {
				data->s--;
				if (data->s < 0)
					data->s = 59;
			}
		} else if (cmd == COMPONENT_CTRL_BREAK) {
			break;
		}

		char temp[4];
		snprintf(temp, sizeof(temp), "%d ", data->hour);
		DRV_Oled_Show_Str_1608(48, 2, temp);
		snprintf(temp, sizeof(temp), "%d ", data->min);
		DRV_Oled_Show_Str_1608(48, 4, temp);
		snprintf(temp, sizeof(temp), "%d ", data->s);
		DRV_Oled_Show_Str_1608(48, 6, temp);

		OS_MSleep(50);
	}
}

void Set_Os_Time(void *arg)
{
	HAL_COMP_MAIN_DBG("%s\n", __func__);
	ui_clear_screen();
	char cursor_postion = 1;
	set_cursor_postion(cursor_postion);
	OS_Time_Info time = ui_current_time();

	DRV_Oled_Show_Str_1608(16, 2, "H :");
	DRV_Oled_Show_Str_1608(16, 4, "M :");
	DRV_Oled_Show_Str_1608(16, 6, "S :");
	while (1) {
		char temp[4];
		snprintf(temp, sizeof(temp), "%d ", time.hour);
		DRV_Oled_Show_Str_1608(48, 2, temp);
		snprintf(temp, sizeof(temp), "%d ", time.min);
		DRV_Oled_Show_Str_1608(48, 4, temp);
		snprintf(temp, sizeof(temp), "%d ", time.s);
		DRV_Oled_Show_Str_1608(48, 6, temp);

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if (cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_INTO :
					Set_Time(cursor_postion, &time);
					break;
				case COMPONENT_CTRL_BREAK :
					ui_clear_screen();
					uint32_t set_time = time.hour * 3600
						                + time.min * 60
						                + time.s;
					ui_set_time(set_time);
					return;
				case COMPONENT_CTRL_UP :
					move_cursor_ctrl(&cursor_postion, CURSOR_UP);
					break;
				case COMPONENT_CTRL_DOWN :
					move_cursor_ctrl(&cursor_postion, CURSOR_DOWN);
					break;
				case COMPONENT_CTRL_SLEEP :
					//component_sleep();
					break;
				default :
					break;
			}
		}else
		OS_MSleep(50);
	}
}

/*************************************************************
						MAIN MENU
**************************************************************/
ui_menu menu_main = {
	Sensor_Func,
	&sensor_func,

	Set_Os_Time,
	NULL,

	NULL,
	NULL,

	1,
};

void main_menu_1608str(void *arg)
{
	ui_menu *data = (ui_menu *)arg;
	set_cursor_postion(data->cursor_postion);
	while (1) {
		DRV_Oled_Show_Str_1608(16, 2, "COMPONMENT");
		DRV_Oled_Show_Str_1608(16, 4, "SET TIME");

		COMPONENT_CTRL cmd = read_component_ctrl_cmd();
		if ( cmd != COMPONENT_CTRL_NULL) {
			switch (cmd) {
				case COMPONENT_CTRL_INTO :
					menu_into(data);
					set_cursor_postion(data->cursor_postion);
					break;
				case COMPONENT_CTRL_BREAK :
					break;
				case COMPONENT_CTRL_UP :
					move_cursor_ctrl(&data->cursor_postion, CURSOR_UP);
					break;
				case COMPONENT_CTRL_DOWN :
					move_cursor_ctrl(&data->cursor_postion, CURSOR_DOWN);
					break;
				case COMPONENT_CTRL_SLEEP:
					//component_sleep();
					break;
				default :
				break;
			}
		}else
		OS_MSleep(50);
	}
}

static OS_Thread_t g_component_ctrl_thread;
#define COMPONENT_CTRL_THREAD_STACK_SIZE	(1 * 1024)

void component_ctrl_task(void *arg)
{
	ui_menu * menu = (ui_menu *)arg;
	DRV_BME280_Enable(BME280_I2C_ID, BME280_I2C_FREQ);
	DRV_BME280_Sleep();
	main_menu_1608str(menu);
}

void component_main()
{

	ui_task_init();
	HAL_COMP_MAIN_DBG("%s, %d\n", __func__, __LINE__);

	if (OS_ThreadCreate(&g_component_ctrl_thread,
	        	            "component_ctrl_task",
	            	        component_ctrl_task,
	                	    &menu_main,
	                    	OS_THREAD_PRIO_APP,
	                    	COMPONENT_CTRL_THREAD_STACK_SIZE) != OS_OK) {
		COMPONENT_WARN("create Component_ctrl_task failed\n");
	}
	sensor_buttons_init();

	COMPONENT_TRACK("end\n");
}
