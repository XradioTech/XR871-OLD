/**
  * @file  drv_volume_ctrl_knob.c
  * @author  XRADIO IOT WLAN Team
  */

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
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_adc.h"
#include "driver/component/volume_control_knob/drv_volume_ctrl_knob.h"

#define DRV_VOLUME_CTRL_KNOB_DBG 0

#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)


#define DRV_VOLUME_CTRL_DBG(fmt, arg...)	\
			//LOG(DRV_VOLUME_CTRL_KNOB_DBG, "[VOLUME CTRL] "fmt, ##arg)

/***************************Tcs4056 Io ******************************************
*******************************************************************************/
#define AD_MAX_VALUE 4095
#define AD_MIN_VALUE 0
#define AD_VALUE_AVERAGE_NUM 10

ADC_Channel VolumeAdCh = ADC_CHANNEL_2;
DRV_Volume_Value_Send_Cb VolumeSend;
static uint32_t VolumeResolution = 0;

void Volume_Read_Cb(void *arg)
{
	static uint32_t ad_value = 0, i = 0;
	uint32_t now_data =  HAL_ADC_GetValue(VolumeAdCh);
	ad_value += now_data;
	i++;
	if (i >= AD_VALUE_AVERAGE_NUM) {
		ad_value /= AD_VALUE_AVERAGE_NUM;
		i = 0;
		uint32_t low_value = 0, high_value = 0;

		if (ad_value < VolumeResolution) {
			low_value = AD_MIN_VALUE;
			ad_value = AD_MIN_VALUE;
		} else
			low_value = ad_value - VolumeResolution;

		if ((ad_value + VolumeResolution) < AD_MAX_VALUE)
			high_value = ad_value + VolumeResolution;
		else {
			high_value = AD_MAX_VALUE;
			ad_value = AD_MAX_VALUE;
		}
		/*send value to sys_ctrl*/
		VolumeSend(ad_value);
		if (ad_value == AD_MIN_VALUE) {
			low_value = AD_MIN_VALUE;
			high_value = VolumeResolution;
			HAL_ADC_ConfigChannel(VolumeAdCh, ADC_SELECT_ENABLE,
						  ADC_IRQ_HIGH, low_value , high_value);
			DRV_VOLUME_CTRL_DBG("%s, %d, advale = %d now_data = %d low = %d high = %d\n",
								__func__, __LINE__, ad_value, now_data, low_value, high_value);
			ad_value = 0;
			return;
		}

		if (ad_value == AD_MAX_VALUE) {
			low_value = AD_MAX_VALUE - VolumeResolution;
			high_value = AD_MAX_VALUE;
			HAL_ADC_ConfigChannel(VolumeAdCh, ADC_SELECT_ENABLE,
						  ADC_IRQ_LOW, low_value , high_value);
			DRV_VOLUME_CTRL_DBG("%s, %d, advale = %d now_data = %d low = %d high = %d\n",
				                 __func__, __LINE__, ad_value, now_data, low_value, high_value);
			ad_value = 0;
			return;
		}
		DRV_VOLUME_CTRL_DBG("%s, %d, advale = %d now_data = %d low = %d high = %d\n",
			                 __func__, __LINE__, ad_value, now_data, low_value, high_value);
		ad_value = 0;
		HAL_ADC_ConfigChannel(VolumeAdCh, ADC_SELECT_ENABLE,
						  ADC_IRQ_LOW_HIGH, low_value , high_value);
	}
}

/**
  * @brief Init the volume knob.
  * @param channel: The ad channel used for volume knob.
  * @retval Component_Status : The driver status.
  */
Component_Status DRV_Volume_Ctrl_Knob_Init(ADC_Channel channel)
{
	VolumeAdCh = channel;
	ADC_InitParam AD_initParam;
	AD_initParam.delay = 10;
	AD_initParam.freq = 500000;
	AD_initParam.mode = ADC_CONTI_CONV;

	HAL_Status sta = HAL_ADC_Init(&AD_initParam);

	if (sta == HAL_OK || sta == HAL_BUSY) {
		return COMP_OK;
	} else {
		COMPONENT_WARN("ADC init error %d\n", sta);
		return COMP_ERROR;
	}
	COMPONENT_TRACK("end\n");
	return COMP_OK;
}

/**
  * @brief Start the volume data repot.
  * @param send_msg_cb: Used repot the volume data.
  * @param volume_resolution: The resolution of volume knob (0~4095).
  		    The smaller the value, the higher the precision.
  * @retval Component_Status : The driver status.
  */
Component_Status DRV_Volume_Repot_Start(DRV_Volume_Value_Send_Cb send_msg_cb,
	                                           uint32_t volume_resolution)
{
	if (send_msg_cb == NULL) {
		COMPONENT_WARN("arg error\n");
		return COMP_ERROR;
	}
	if (volume_resolution > 4095) {
		COMPONENT_WARN("arg error\n");
		return COMP_ERROR;
	}
	VolumeSend = send_msg_cb;
	VolumeResolution = volume_resolution;
	HAL_ADC_ConfigChannel(VolumeAdCh, ADC_SELECT_ENABLE, ADC_IRQ_DATA, 0, 0);
	HAL_ADC_Start_Conv_IT();
	HAL_ADC_EnableIRQCallback(VolumeAdCh, Volume_Read_Cb, NULL);
	COMPONENT_TRACK("end\n");
	return COMP_OK;
}

/**
  * @brief Sopt the volume data repot.
  * @retval Component_Status : The driver status.
  */
void DRV_Volume_Repot_Stop()
{
	HAL_ADC_DisableIRQCallback(VolumeAdCh);
	HAL_ADC_ConfigChannel(VolumeAdCh, ADC_SELECT_DISABLE,
						  ADC_IRQ_NONE, 0, 0);
}
