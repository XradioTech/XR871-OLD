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

#include "cmd_debug.h"
#include "cmd_util.h"
#include "cmd_pwm.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_pwm.h"
#include "kernel/os/os.h"
#include "stdlib.h"
#include "string.h"
#include "kernel/os/os_timer.h"
#include "kernel/os/os_time.h"


#define HAL_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define HAL_DBG_CMD_PWM	 0
#define HAL_PWM_CMD_DBG(fmt, arg...)	\
	HAL_LOG(HAL_DBG_CMD_PWM, "[HAL CMD_PWM] "fmt, ##arg)

#define CMD_PWM_HZ_MIN_D 3
#define CMD_PWM_V_MIN_D 5

typedef enum {
	PWM_COMPLE,
	PWM_PLUSE,
	PWM_CYCLE,
	PWM_CAPTURE,
	PWM_DEADZONE,
	PWM_MODENUM,
}PWM_Mode;

typedef struct {
	PWM_ChGroup group;
	PWM_CHID ch;
}PWM_Info_;

static PWM_Info_ cmd_pwm_channel_analytic(PWM_Mode mode, uint32_t channel)
{
	PWM_Info_ info;
	if (mode == PWM_COMPLE || mode == PWM_DEADZONE) {
		if (channel > 3) {
			info.ch = PWM_CH_NULL;
			info.group = PWM_GROUP_NULL;
			return info;
		}
		info.group = channel;
		info.ch = PWM_CH_NULL;
		return info;
	} else if (channel > 7) {
		info.ch = PWM_CH_NULL;
		info.group = PWM_GROUP_NULL;
		return info;
	}

	info.ch = channel;
	info.group = channel / 2;

	return info;
}

static PWM_Mode cmd_pwm_mode_analytic(char *data)
{
	if (cmd_strcmp(data, "comple") == 0) {
		return PWM_COMPLE;
	} else if (cmd_strcmp(data, "pluse") == 0) {
		return PWM_PLUSE;
	} else if (cmd_strcmp(data, "cycle") == 0) {
		return PWM_CYCLE;
	} else if (cmd_strcmp(data, "capture") == 0) {
		return PWM_CAPTURE;
	} else if (cmd_strcmp(data, "deadzone") == 0) {
	 	return PWM_DEADZONE;
	}
	return PWM_MODENUM;
}

static HAL_Status  cmd_pwm_cmple(uint32_t hz, PWM_Info_ info)
{
	PWM_SrcClk srcClkSet;
	PWM_Complementary_Mode complementarySet;
	HAL_Status ret;

	srcClkSet.chGroup= info.group;
	srcClkSet.srcClkDiv = PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk = PWM_CLK_HOSC;

	complementarySet.lowChPolarity = PWM_HIGHLEVE;
	complementarySet.highChPolarity = PWM_LOWLEVE;
	complementarySet.hz = hz;
	complementarySet.chGroup= info.group;
	complementarySet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);

	ret = HAL_PWM_ComplementaryModeInit(&complementarySet);
	if (ret == HAL_ERROR) {
		CMD_DBG("%s%s","HAL_PWM_ComplementaryInit error, the Hz out of clock range.\n"\
			,"Please set a reasonable value for Pari_Clk or set Hz a other value\n");
	}
	return ret;
}

static HAL_Status  cmd_pwm_pluse(uint32_t hz, PWM_Info_ info)
{
	PWM_SrcClk srcClkSet;
	PWM_Output_Init PwmOutputSet;
	HAL_Status ret = 0;
	srcClkSet.chGroup= info.group;
	srcClkSet.srcClkDiv= PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk= PWM_CLK_HOSC;

	PwmOutputSet.ch= info.ch;
	PwmOutputSet.polarity = PWM_HIGHLEVE;
	PwmOutputSet.hz = hz;
	PwmOutputSet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);

	ret = HAL_PWM_PluseModeInit(&PwmOutputSet);
	if (ret == HAL_ERROR) {
		CMD_DBG("%s%s","HAL_PWM_PluseModeInit error, the Hz out of clock range.\n"\
			,"Please set a reasonable value for Pari_Clk or set Hz a other value\n");
	}
	HAL_PWM_OutModeEnableCh(info.ch);
	return ret;
}

static HAL_Status  cmd_pwm_cycle(uint32_t hz, PWM_Info_ info)
{

	PWM_SrcClk srcClkSet;
	PWM_Output_Init PwmOutputSet;
	HAL_Status ret = 0;

	srcClkSet.chGroup= info.group;
	srcClkSet.srcClkDiv= PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk= PWM_CLK_HOSC;


	PwmOutputSet.ch= info.ch;
	PwmOutputSet.polarity= PWM_HIGHLEVE;
	PwmOutputSet.hz = hz;
	PwmOutputSet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);

	ret = HAL_PWM_CycleModeInit(&PwmOutputSet);
	if (ret == HAL_ERROR) {
		CMD_DBG("%s%s","HAL_PWM_CycleModeInit error, the Hz out of clock range.\n"\
			,"Please set a reasonable value for Pari_Clk or set Hz a other value\n");
	}
	HAL_PWM_CMD_DBG("enter value %d\n", HAL_PWM_GetEnterCycleValue(info.ch));
	return ret;
}

typedef enum {
	PRINT_NULL = 0,
	PRINT_PERIOD = 1,
	PRINT_UPLEVEL = 2,
}cmd_capture_print_format;

typedef enum {
	CAP_ERROR,
	CAP_BUSY,
	CAP_OK,
}CAP_RESULT;

typedef struct {
	PWM_squareWaveInfo *_squareInfo;
	uint32_t num;
	uint32_t hz_deviation;
	uint32_t value_deviation;
	PWM_CHID ch;
	PWM_CHID inputch;
	cmd_capture_print_format format;
}cmd_capture;

#define CAPTURE_TASK_THREAD_STACK_SIZE	(2 * 1024)
OS_Thread_t g_capture_thread_t;
static uint8_t capture_open = 0;

cmd_capture cmd_cap_private[8] = {
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
	{NULL, 0, 0, PRINT_NULL, 0, 0, 0},
};

#define MAXCNTRVAL 65535

int capture_clk_div(uint32_t hz, uint32_t srcClockActualFreq)
{
		int chClkdiv = 0;
		uint32_t minFreq = 0;
		uint32_t temp1 = 0, temp2 = 0;

		if ((srcClockActualFreq % MAXCNTRVAL) > 0)
			temp1 = 1;
		if (((srcClockActualFreq + temp1) % 256) > 0)
			temp2 = 1;

		minFreq = (srcClockActualFreq / MAXCNTRVAL + temp1)/ 256 + temp2;
		if (hz > srcClockActualFreq || hz < minFreq)
			return -1;

		if (hz > (srcClockActualFreq / MAXCNTRVAL + temp1))
			chClkdiv = 0 ;
		else {
			chClkdiv =  (srcClockActualFreq / MAXCNTRVAL + temp1) % hz;
			if (chClkdiv)
				chClkdiv =  (srcClockActualFreq / MAXCNTRVAL + temp1) / hz;
			else
				chClkdiv =  (srcClockActualFreq / MAXCNTRVAL + temp1) / hz - 1;
		}
		return chClkdiv + 1;
}

HAL_Status  cmd_pwm_capture(uint32_t hz, PWM_Info_ info)
{
	PWM_SrcClk srcClkSet;
	PWM_Input_Init CaptureSet;
	srcClkSet.chGroup= info.group;
	srcClkSet.srcClkDiv= PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk= PWM_CLK_HOSC;

	CaptureSet.ch = info.ch;
	CaptureSet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);
	CaptureSet.chClkDiv = capture_clk_div(hz, CaptureSet.srcClkActualFreq);
	if (HAL_PWM_InputInit(&CaptureSet) == -1) {
		CMD_DBG("capture init error, hz out of range\n");
		return HAL_ERROR;
	}

	cmd_capture *p = &cmd_cap_private[info.ch];
	p->ch= info.ch;
	p->format = PRINT_NULL;
	if (p->_squareInfo != NULL) {
		free(p->_squareInfo);
		p->_squareInfo = NULL;
	}
	p->num = 0;

	PWM_InputIRQ captureIRQ;
	captureIRQ.arg = NULL;
	captureIRQ.callBack = NULL;
	captureIRQ.ch = info.ch;
	HAL_PWM_InputIRQInit(&captureIRQ);
	HAL_PWM_ModuleIRQEnable();
	return HAL_OK;
}

static HAL_Status cmd_pwm_mode_init(PWM_Mode mode,PWM_Info_ info, uint32_t hz)
{
	PWM_Init_Param pwmParam;
	pwmParam.ch = info.ch;
	switch(mode) {
		case PWM_COMPLE:
			pwmParam.ch = info.group * 2;
			HAL_PWM_CMD_DBG("comple l_ch ch%d\n", pwmParam.ch);
			HAL_PWM_IO_Init(&pwmParam);
			pwmParam.ch = info.group * 2 + 1;
			HAL_PWM_CMD_DBG("comple h_ch ch%d\n", pwmParam.ch);
			HAL_PWM_IO_Init(&pwmParam);
			HAL_PWM_CMD_DBG("comple group%d\n", info.group);
			if (cmd_pwm_cmple(hz, info) == HAL_ERROR) {
				CMD_DBG("cmd pwm cmple mode init error\n");
				return HAL_ERROR;
			}
			break;
		case PWM_PLUSE:
			HAL_PWM_IO_Init(&pwmParam);
			if (cmd_pwm_pluse(hz, info) == HAL_ERROR) {
				CMD_DBG("cmd pwm pluse mode init error\n");
				return HAL_ERROR;
			}
			break;
		case PWM_CYCLE:
			HAL_PWM_IO_Init(&pwmParam);
			if (cmd_pwm_cycle(hz, info) == HAL_ERROR) {
				CMD_DBG("cmd pwm cycle mode init error\n");
				return HAL_ERROR;
			}
			break;
		case PWM_CAPTURE:
				HAL_PWM_IO_Init(&pwmParam);
			if (cmd_pwm_capture(hz, info) == HAL_ERROR) {
				CMD_DBG("cmd pwm capture mode init error\n");
				return HAL_ERROR;
			}
			break;
		default :
			CMD_DBG("invalid PWM mode\n");
			return HAL_ERROR;
	}
	return HAL_OK;
}


/*
 * drv pwm config
 */
static enum cmd_status cmd_pwm_config_exec(char *cmd)
{
	uint32_t hz;
	uint32_t channel;
	char modeChar[8];
	int cnt;

	cnt = cmd_sscanf(cmd, "c=%u m=%s h=%u", &channel, modeChar, &hz);

	if (cnt != 3) {
		return CMD_STATUS_INVALID_ARG;
	}

	PWM_Mode mode = cmd_pwm_mode_analytic(modeChar);
	if (mode >= PWM_MODENUM) {
		CMD_ERR("invalid pwm mode %u\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}
	if (hz < 2 || hz > 260000) {
		CMD_ERR("pwm hz out of range %u\n", hz);
		return CMD_STATUS_FAIL;
	}
	HAL_PWM_CMD_DBG("config channel%d\n", channel);
	PWM_Info_ info = cmd_pwm_channel_analytic(mode, channel);
	HAL_PWM_CMD_DBG("config group%d\n", info.group);
	if (info.group >= PWM_GROUP_NULL)
		return CMD_STATUS_INVALID_ARG;

	if (cmd_pwm_mode_init(mode, info, hz) == HAL_ERROR)
		return CMD_STATUS_FAIL;

	return CMD_STATUS_OK;
}

enum cmd_status  cmd_pwm_start_exec(char *cmd)
{
	char modeChar[8];
	uint32_t channel;
	int cnt;

	cnt = cmd_sscanf(cmd, "c=%u m=%s", &channel, modeChar);
	if (cnt != 2)
		return CMD_STATUS_INVALID_ARG;

	PWM_Mode mode = cmd_pwm_mode_analytic(modeChar);
	if (mode >= PWM_MODENUM) {
		CMD_ERR("invalid pwm mode %u\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}
	PWM_Info_ info = cmd_pwm_channel_analytic(mode, channel);
	if (info.group == PWM_GROUP_NULL) {
		CMD_DBG("invalid pwm channel\n");
		return CMD_STATUS_INVALID_ARG;
	}
	switch (mode) {
		case PWM_COMPLE:
			HAL_PWM_CMD_DBG("comple enable group%d\n", info.group);
			HAL_PWM_ComplementaryEnable(info.group);
			break;
		case PWM_PLUSE:
			if (HAL_PWM_PluseStart(info.ch) == HAL_BUSY)
				return CMD_STATUS_FAIL;
			break;
		case PWM_CYCLE:
			HAL_PWM_OutModeEnableCh(info.ch);
			break;
		case PWM_CAPTURE:
			HAL_PWM_InputEnableCh(info.ch);
			HAL_PWM_InputIRQEnable(PWM_IRQ_BOTHEDGE, info.ch);
			break;
		case PWM_DEADZONE:
			HAL_PWM_DeadZoneEnable(info.group);
			break;
		default:
			CMD_DBG("invalid pwm mode %u\n", mode);
			return CMD_STATUS_INVALID_ARG;
	}
	return CMD_STATUS_OK;
}

enum cmd_status  cmd_pwm_set_exec(char *cmd)
{
	uint32_t value;
	uint32_t channel;
	char function[10];
	int cnt;
	PWM_Info_ info;
	cnt = cmd_sscanf(cmd, "c=%u m=%s v=%u", &channel, function, &value);

	if (cnt != 3)
		return CMD_STATUS_INVALID_ARG;
	if (cmd_strcmp(function, "comple") == 0 ||
		cmd_strcmp(function, "deadzone") == 0 )
		info = cmd_pwm_channel_analytic(PWM_COMPLE, channel);
	else
		info = cmd_pwm_channel_analytic(PWM_CYCLE, channel);

	if (info.group == PWM_GROUP_NULL) {
		CMD_DBG("invalid pwm channel\n");
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(function, "deadzone") == 0) {
		if (value > 255) {
			CMD_DBG("deadzone value out of range\n");
			return CMD_STATUS_INVALID_ARG;
		}
		HAL_PWM_SetDeadZoneTime(info.group, value);
	} else if (cmd_strcmp(function, "comple") == 0) {
		if (value > 0) {
			info.ch = info.group * 2;
			value = value *	HAL_PWM_GetEnterCycleValue(info.ch) / 10000;
			if (value <= 1)
				value = 2;
		}
		HAL_PWM_CMD_DBG("comple set value %d\n", value);
		HAL_PWM_ComplementarySetDutyRatio(info.group, value);
	} else if (cmd_strcmp(function, "cycle") == 0) {
		if (value > 0) {
			value = value *	HAL_PWM_GetEnterCycleValue(info.ch) / 10000;
			if (value <= 1)
				value = 2;
		}
		HAL_PWM_CMD_DBG("cycle set value %d\n", value);
		HAL_PWM_SetDutyRatio(info.ch, value);
	} else if (cmd_strcmp(function, "pluse") == 0) {
		if (value > 0) {
			value = value *	HAL_PWM_GetEnterCycleValue(info.ch) / 10000;
			if (value <= 1)
				value = 2;

		}
		HAL_PWM_CMD_DBG("pluse set value %d\n", value);
		HAL_PWM_SetDutyRatio(info.ch, value);
	} else
		return CMD_STATUS_INVALID_ARG;

	return CMD_STATUS_OK;
}

static uint32_t capture_count_[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/********************************************
when input signal is continuous, the first capture value
maybe error, so you should discarded first value.
*********************************************/

void cmd_capture_task(void *arg)
{
	int i = 0;
	HAL_PWM_CMD_DBG("capture task statr\n");
	while (1) {
		if (PWM->CIER == 0) {
			HAL_PWM_CMD_DBG("capture task break\n");
			capture_open = 0;
			break;
		}

		cmd_capture *Temp = &cmd_cap_private[i];
		uint32_t *count = &capture_count_[i];
	if (Temp->format == PRINT_PERIOD)
		if (Temp->_squareInfo != NULL) {
			PWM_squareWaveInfo *result = (Temp->_squareInfo + Temp->num - 1);
			if (result->periodTime == 0) {
				PWM_squareWaveInfo data = HAL_PWM_CaptureResult(Temp->ch);
				if (data.highLevelTime && data.lowLevelTime) {
					if (*count > 0) {
						result = (Temp->_squareInfo + *count - 1);
						*result = data;
					}
					*count += 1;
				}
			} else
				*count =0;
		}
		i ++;
		if (i >= 8)
			i = 0;
	}
	OS_ThreadDelete(&g_capture_thread_t);
}

void cmd_capture_pluse_cb(void *arg, void *arg_IrqSta)
{
	cmd_capture *Temp = (cmd_capture *)arg;
	PWM_IrqMode *irqsta = (PWM_IrqMode *)arg_IrqSta;
	if (*irqsta == PWM_IRQ_FALLEDGE)
		if (Temp->_squareInfo != NULL) {
			uint32_t *count = &capture_count_[Temp->ch];
				PWM_squareWaveInfo *result = (Temp->_squareInfo + Temp->num - 1);
				if (result->periodTime == 0) {
					PWM_squareWaveInfo data = HAL_PWM_CaptureResult(Temp->ch);
					if (data.highLevelTime && data.lowLevelTime) {
						if (*count > 0) {
							result = (Temp->_squareInfo + *count - 1);
							*result = data;
						}
						*count += 1;
					}
				} else
					*count =0;
	}
}

enum cmd_status  cmd_pwm_get_exec(char *cmd)
{
	uint32_t channel;
	uint32_t input_channel;
	char function[8];
	uint32_t num = 0;
	uint32_t value_deviation = 0;
	uint32_t hz_deviation = 0;
	int cnt;

	cnt = cmd_sscanf(cmd, "c=%u m=%s n=%u input_ch=%u dv=%u dh=%u", &channel, function, &num, &input_channel, &value_deviation, &hz_deviation);
	if (cnt != 6)
		return CMD_STATUS_INVALID_ARG;
	if (num > 1000) {
		CMD_DBG("The n value out of range\n");
		return CMD_STATUS_INVALID_ARG;
	}
	if (value_deviation > 100 || hz_deviation > 100) {
		CMD_DBG("The value out of range\n");
		return CMD_STATUS_FAIL;
	}

	PWM_Info_ info;
	info = cmd_pwm_channel_analytic(PWM_CAPTURE, input_channel);
	if (info.group== PWM_GROUP_NULL) {
		CMD_DBG("invalid inupt_ch\n");
		return CMD_STATUS_INVALID_ARG;
	}
	info = cmd_pwm_channel_analytic(PWM_CAPTURE, channel);
	if (info.group== PWM_GROUP_NULL) {
		CMD_DBG("invalid ch \n");
		return CMD_STATUS_INVALID_ARG;
	}
	HAL_PWM_CMD_DBG("ch id %d\n", info.ch);

	cmd_capture *p = &cmd_cap_private[info.ch];
	if (p->_squareInfo != NULL) {
		CMD_DBG("(p->_squareInfo != NULL)\n");
		free(p->_squareInfo);
	}
	HAL_PWM_ClearFallEdgeConterLockFlag(info.ch);
	HAL_PWM_ClearRiseEdgeConterLockFlag(info.ch);
	capture_count_[info.ch] = 0;
	if (cmd_strcmp(function, "period") == 0) {
		p->format = PRINT_PERIOD;
		p->num = num;
		p->inputch = input_channel;
		p->hz_deviation = hz_deviation + CMD_PWM_HZ_MIN_D;
		p->value_deviation = value_deviation + CMD_PWM_V_MIN_D;;
		p->_squareInfo = (PWM_squareWaveInfo *)
						malloc(sizeof(PWM_squareWaveInfo) * num);
		if (p->_squareInfo == NULL) {
			CMD_DBG("malloc error\n");
			return CMD_STATUS_FAIL;
		}
		memset(p->_squareInfo, 0, sizeof(PWM_squareWaveInfo) * num);
	} else if (cmd_strcmp(function, "pluse") == 0) {
		p->format = PRINT_UPLEVEL;
		p->num = num;
		p->inputch = input_channel;
		p->value_deviation = value_deviation;
		p->_squareInfo = (PWM_squareWaveInfo *)
						malloc(sizeof(PWM_squareWaveInfo) * num);
		if (p->_squareInfo == NULL) {
			CMD_DBG("malloc error\n");
			return CMD_STATUS_FAIL;
		}
		memset(p->_squareInfo, 0, sizeof(PWM_squareWaveInfo) * num);
		PWM_InputIRQ captureIRQ;
		captureIRQ.arg = &cmd_cap_private[info.ch];
		captureIRQ.callBack = cmd_capture_pluse_cb;
		captureIRQ.ch = info.ch;
		HAL_PWM_InputIRQInit(&captureIRQ);
	} else {
		return CMD_STATUS_INVALID_ARG;
	}

	if (capture_open == 0) {
		capture_open = 1;
		while (OS_ThreadIsValid(&g_capture_thread_t));
		if (OS_ThreadCreate(&g_capture_thread_t,
	                    "",
	                    cmd_capture_task,
	                    cmd_cap_private,
	                    OS_PRIORITY_ABOVE_NORMAL,
	                    CAPTURE_TASK_THREAD_STACK_SIZE) != OS_OK) {
			CMD_DBG("create sys ctrl task failed\n");
		}
	}
	return CMD_STATUS_OK;
}

static int ReadActCycle(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return -1;
	int p = (int)PWM->CH_REG[ch].PPR;
	return p;
}


static PWM_Out_polarity PWM_Polarity(PWM_CHID ch)
{
	if ((PWM->CH_REG[ch].PCR & PWM_PCR_ACT_STA) > 0)
		return PWM_HIGHLEVE;

	return PWM_LOWLEVE;

}

static int PWM_DeadTime(PWM_ChGroup chGroup)
{

	if (chGroup >= PWM_GROUP_NUM)
		return -1;

	uint32_t p = PWM->PDZCR[chGroup];
	p &= PWM_CH_DZ_INV;
	return (p >> 8);
}

int DeadZoneEnable(PWM_ChGroup chGroup)
{
	if (chGroup >= PWM_GROUP_NUM)
		return -1;

	return PWM->PDZCR[chGroup]&PWM_CH_DZ_EN;
}

CAP_RESULT cmd_CapResult(cmd_capture *arg) {
	CAP_RESULT ret;
	__IO cmd_capture *cap_private = arg;
	uint16_t inputRight_period = 0, inputRight_h_time = 0;
	uint8_t deadzone_time = 0;
	if (DeadZoneEnable(cap_private->inputch / 2)) {
		deadzone_time = PWM_DeadTime(cap_private->inputch / 2);
		HAL_PWM_CMD_DBG("deadzone enable \n");
	}
	HAL_PWM_CMD_DBG("deadzone_time %d\n", deadzone_time);
	if (cap_private->_squareInfo != NULL) {
		__IO PWM_squareWaveInfo *Temp = (cap_private->_squareInfo + cap_private->num - 1);

		if (Temp->periodTime > 0) {
			HAL_PWM_CMD_DBG("Temp->periodTime >0 , = %d \n", Temp->periodTime);
			int i = 0;
			uint32_t cap_period = 0;
			uint32_t cap_h_time = 0;

			inputRight_period = HAL_PWM_GetEnterCycleValue(cap_private->inputch);
			inputRight_h_time = ReadActCycle(cap_private->inputch);
			if (PWM_Polarity(cap_private->inputch)== PWM_LOWLEVE) {
				HAL_PWM_CMD_DBG("inputch is LOWLEVE \n");
				inputRight_h_time = inputRight_period- inputRight_h_time;
			}
			if (deadzone_time > 0) {
				inputRight_h_time -= deadzone_time;
			}
			HAL_PWM_CMD_DBG("inputRight_period %d\n", inputRight_period);
			HAL_PWM_CMD_DBG("inputRight_h_time %d\n", inputRight_h_time);

			for (i = 0; i < cap_private->num; i++) {
				Temp = cap_private->_squareInfo + i;
				if (cap_private->format == PRINT_PERIOD) {
					cap_period += Temp->periodTime;
					cap_h_time += Temp->highLevelTime;
				} else if (cap_private->format == PRINT_UPLEVEL) {
					cap_h_time += Temp->highLevelTime;
				}
			}

			if (cap_private->format == PRINT_PERIOD) {
				cap_period /= cap_private->num;
				cap_h_time /= cap_private->num;
				if (abs(cap_h_time - inputRight_h_time) <= cap_private->value_deviation &&
								abs(cap_period - inputRight_period) <= cap_private->hz_deviation)
					ret = CAP_OK;
				else
					ret = CAP_ERROR;
			} else {
				cap_h_time /= cap_private->num;
				if (abs(cap_h_time - inputRight_h_time) <= cap_private->value_deviation)
					ret = CAP_OK;
				else
					ret = CAP_ERROR;
			}
			return ret;
		}
	}
	return CAP_BUSY;
}

enum cmd_status  cmd_pwm_get_result_exec(char *cmd)
{
	uint32_t channel;
	int cnt;

	cnt = cmd_sscanf(cmd, "c=%u ", &channel);
	if (cnt != 1)
		return CMD_STATUS_INVALID_ARG;

	PWM_Info_ info = cmd_pwm_channel_analytic(PWM_CAPTURE, channel);
	if (info.group== PWM_GROUP_NULL) {
		CMD_DBG("invalid pwm channel\n");
		return CMD_STATUS_INVALID_ARG;
	}
	if (cmd_cap_private[info.ch]._squareInfo == NULL) {
		HAL_PWM_CMD_DBG("cmd_cap_private[info.ch]._squareInfo == NULL\n");
		return CMD_STATUS_FAIL;
	}

	CAP_RESULT ret = cmd_CapResult(&cmd_cap_private[info.ch]);

	if (ret != CAP_BUSY) {
		free(cmd_cap_private[info.ch]._squareInfo);
		cmd_cap_private[info.ch]._squareInfo = NULL;
	}

	if (ret == CAP_BUSY) {
		HAL_PWM_CMD_DBG("PWM%d Capture BUSY\n", info.ch);
		return CMD_STATUS_ERROR_MIN;
	} else if (ret == CAP_ERROR) {
		HAL_PWM_CMD_DBG("PWM%d Capture ERROR\n",info.ch);
		return CMD_STATUS_FAIL;
	} else if (ret == CAP_OK) {
		HAL_PWM_CMD_DBG("PWM%d Capture OK\n", info.ch);
		return CMD_STATUS_OK;
	}

	return CMD_STATUS_OK;
}

enum cmd_status cmd_pwm_stop_exec(char *cmd)
{
	char modeChar[8];
	uint32_t channel;
	int cnt;

	cnt = cmd_sscanf(cmd, "c=%u m=%s", &channel, modeChar);
	if (cnt != 2)
		return CMD_STATUS_INVALID_ARG;

	PWM_Mode mode = cmd_pwm_mode_analytic(modeChar);
	if (mode >= PWM_MODENUM) {
		CMD_ERR("invalid pwm mode %u\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}

	PWM_Info_ info = cmd_pwm_channel_analytic(mode, channel);
	if (info.group== PWM_GROUP_NULL) {
		CMD_ERR("invalid pwm channel\n");
		return CMD_STATUS_INVALID_ARG;
	}

	switch(mode) {
			case PWM_COMPLE:
				HAL_PWM_ComplementaryDisable(info.group);
				HAL_PWM_DeadZoneDisable(info.group);
				break;
			case PWM_PLUSE:
				HAL_PWM_OutModeDisableCh(info.ch);
				break;
			case PWM_CYCLE:
				HAL_PWM_OutModeDisableCh(info.ch);
				break;
			case PWM_CAPTURE:
				HAL_PWM_InputDisableCh(info.ch);
				HAL_PWM_InputIRQDisable(PWM_IRQ_BOTHEDGE, info.ch);
				HAL_PWM_IRQDeInit(info.ch);
				cmd_capture *p = &cmd_cap_private[info.ch];
				if (p->_squareInfo != NULL)
					free(p->_squareInfo);
				break;
			case PWM_DEADZONE:
				HAL_PWM_SetDeadZoneTime(info.group, 0);
				HAL_PWM_DeadZoneDisable(info.group);
				break;
			default :
				CMD_ERR("invalid pwm mode\n");
				return CMD_STATUS_INVALID_ARG;
		}
	return CMD_STATUS_OK;
}

enum cmd_status  cmd_pwm_deinit_exec(char *cmd)
{
	int cnt = 0;
	uint32_t channel;
	char modeChar[8];
	cnt = cmd_sscanf(cmd, "c=%u m=%s", &channel, modeChar);
	if (cnt != 2)
		return CMD_STATUS_INVALID_ARG;

	PWM_Mode mode = cmd_pwm_mode_analytic(modeChar);
	if (mode >= PWM_MODENUM) {
		CMD_ERR("invalid pwm mode %u\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}

	PWM_Info_ info = cmd_pwm_channel_analytic(mode, channel);
	if (info.group== PWM_GROUP_NULL) {
		CMD_ERR("invalid pwm channel\n");
		return CMD_STATUS_INVALID_ARG;
	}
	PWM_Init_Param param;
	if (mode == PWM_COMPLE) {
		HAL_PWM_ComplementaryDisable(info.group);
		HAL_PWM_DeadZoneDisable(info.group);
		param.ch = info.group * 2;
		HAL_PWM_OUT_IRQDisable(param.ch);
		HAL_PWM_DeInit(&param);
		param.ch = info.group * 2 + 1;
		HAL_PWM_OUT_IRQDisable(param.ch);
		HAL_PWM_DeInit(&param);
	} else if (mode == PWM_CAPTURE){
		HAL_PWM_InputDisableCh(info.ch);
		HAL_PWM_InputIRQDisable(PWM_IRQ_BOTHEDGE, info.ch);
		HAL_PWM_IRQDeInit(info.ch);
		cmd_capture *p = &cmd_cap_private[info.ch];
		if (p->_squareInfo != NULL)
			free(p->_squareInfo);
		param.ch = info.ch;
		HAL_PWM_DeInit(&param);
	} else {
		HAL_PWM_OutModeDisableCh(info.ch);
		HAL_PWM_OUT_IRQDisable(info.ch);
		param.ch = info.ch;
		HAL_PWM_DeInit(&param);
	}
	return CMD_STATUS_OK;
}

/*
 * driver commands
 */
static struct cmd_data g_pwm_cmds[] = {
	{ "deinit",	cmd_pwm_deinit_exec },
	{ "config",	cmd_pwm_config_exec },
	{ "start",	cmd_pwm_start_exec },
	{ "set",	cmd_pwm_set_exec },
	{ "get",	cmd_pwm_get_exec },
	{ "get_result", cmd_pwm_get_result_exec },
	{ "stop",	cmd_pwm_stop_exec },
};

enum cmd_status cmd_pwm_exec(char *cmd)
{
	return cmd_exec(cmd, g_pwm_cmds, cmd_nitems(g_pwm_cmds));
}

