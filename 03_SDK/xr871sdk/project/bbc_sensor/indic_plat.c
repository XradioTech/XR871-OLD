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
#include "kernel/os/os.h"
#include "driver/chip/hal_adc.h"
#include "driver/component/bme280/drv_bme280.h"
#include "stdio.h"

#define INDIC_PLAT_DEG_SET 	1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define INDIC_PLAT_DEBUG(fmt, arg...)	\
			LOG(INDIC_PLAT_DEG_SET, "[INDIC_PLAT_DEBUG] "fmt, ##arg)

#define BRGHT_AD_CH 	ADC_CHANNEL_5
#define BIRGHTNESS_RESOLUTION 	200
uint32_t ad_bright_value = 0;

//extern HAL_Status board_adc_cfg(uint32_t id, HAL_BoardReq req, void *chan);

void bbc_birght_cb(void *arg)
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

int bbc_brightness_init()
{
	ADC_InitParam AD_initParam;
	//AD_initParam.boardCfg = board_adc_cfg;
	AD_initParam.delay = 10;
	AD_initParam.freq = 500000;
	HAL_Status sta = HAL_ADC_Init(&AD_initParam);

	if (sta == HAL_OK || sta == HAL_BUSY) {
		if (sta == HAL_OK)
			HAL_ADC_Start_Conv_IT();
	} else {
		INDIC_PLAT_DEBUG("ADC init error %d\n", sta);
		return -1;
	}
	HAL_Status ret;

	ret = HAL_ADC_ConfigChannel(BRGHT_AD_CH, ADC_SELECT_ENABLE, ADC_IRQ_DATA, 0, 0);
	if (HAL_OK != ret) {
		INDIC_PLAT_DEBUG("ADC config error %d\n", ret);
		return -1;
	}

	ret = HAL_ADC_EnableIRQCallback(BRGHT_AD_CH, bbc_birght_cb, &ad_bright_value);
	if (HAL_OK != ret) {
		INDIC_PLAT_DEBUG("ADC irq en error %d\n", ret);
		return -1;
	}

	return 0;
}

void bbc_brightness_deinit()
{
	HAL_ADC_ConfigChannel(BRGHT_AD_CH, ADC_SELECT_DISABLE, ADC_IRQ_NONE,0, 0);
	HAL_ADC_DisableIRQCallback(BRGHT_AD_CH);
}

int bbc_set_bright()
{
	char bright_str[4];
	int bright = 255 - 255 * ((double)ad_bright_value / 4095.0 * 2);

	if (bright < 1)
		bright = 1;
	sprintf(bright_str, " %03d", bright);
	INDIC_PLAT_DEBUG("bright = %d\n",bright);

	return bright;
}

void bbc_bme280_init(void)
{
	DRV_BME280_Enable(I2C0_ID, 400000);
	//DRV_BME280_Sleep();
	//DRV_BME280_WakeUp();
}

int bbc_bme280_temp(void)
{
	unsigned int temper;

	temper = DRV_BME280_Read(TEMPERATURE);

	float value = (float)temper / 100.0;
	char temp[10];
	sprintf(temp, "%.2f c", value);

	INDIC_PLAT_DEBUG("temp = %s\n",temp);

	return temper;
}

void bbc_bme280_pres(void)
{
	unsigned int pressure;

	pressure = DRV_BME280_Read(PRESSURE);

	float value = (float)pressure / 100.0;
	char pres[10];
	sprintf(pres, "%.2f hpa", value);

	INDIC_PLAT_DEBUG("press = %s\n",pres);
}

int bbc_bme280_humd(void)
{
	unsigned int humidity;

	humidity = DRV_BME280_Read(HUMIDITY);

	float value = (float)humidity / 1000.0;
	char hum[5];
	sprintf(hum, "%.2f rh", value);

	INDIC_PLAT_DEBUG("hum = %s\n",hum);

	return humidity;
}

void senor_init(void)
{
	if (bbc_brightness_init() != 0) {
		INDIC_PLAT_DEBUG("brightness init error\n");
	}
	bbc_bme280_init();
}