/**
  * @file  drv_bme280.c
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
#include "bme280.h"
#include "driver/chip/hal_i2c.h"
#include "driver/component/bme280/drv_bme280.h"

#define BME280_DBG 0
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_BME280_DBG(fmt, arg...)	\
			//LOG(BME280_DBG, "[HAL BME280] "fmt, ##arg)


uint32_t BME280_I2cId;
uint32_t BME280_I2cClkFreq;

static struct bme280_t bme280_Info;

static void BME280_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
}

static s8 BME280_I2C_Write(u8 devAddr, u8 memAddr, u8 *buf, u8 size)
{
	s8 ret = HAL_I2C_Master_Transmit_Mem_IT(BME280_I2cId, devAddr, memAddr, I2C_MEMADDR_SIZE_8BIT, buf, size);
	return ret;
}

static s8 BME280_I2C_Read(u8 devAddr, u8 memAddr, u8 *buf, u8 size)
{
	s8 ret = HAL_I2C_Master_Receive_Mem_IT(BME280_I2cId, devAddr, memAddr, I2C_MEMADDR_SIZE_8BIT, buf, size);
	return ret;
}

static BME280_RETURN_FUNCTION_TYPE BME280_Init(struct bme280_t *bme280)
{
	I2C_InitParam initParam;
	initParam.addrMode = I2C_ADDR_MODE_7BIT;
	initParam.clockFreq = BME280_I2cClkFreq;
	HAL_I2C_Init(BME280_I2cId, &initParam);

	bme280->bus_write = BME280_I2C_Write;
	bme280->bus_read = BME280_I2C_Read;
	bme280->dev_addr = BME280_I2C_ADDRESS1;
	bme280->delay_msec = BME280_delay_msek;

	BME280_RETURN_FUNCTION_TYPE type = bme280_init(bme280);
	if (type == 1)
		COMPONENT_WARN("bme280 init error \n");

	COMPONENT_TRACK("end\n");
	return type;
}

/**
  * @brief Enable BME280.
  * @note This function is used to enable bme280 and config the i2c.
  * @retval success return 0, else return 1.
  */
s32 DRV_BME280_Enable(I2C_ID id, uint32_t i2c_clk_freq)
{
	BME280_I2cId = id;
	BME280_I2cClkFreq = i2c_clk_freq;
	/* The variable used to assign the standby time*/
	s32 com_rslt = BME280_ERROR;
	u8 v_stand_by_time_u8 = BME280_INIT_VALUE;
	DRV_BME280_DBG("HAL_BME280_Enable\n");
	com_rslt = BME280_Init(&bme280_Info);
	DRV_BME280_DBG("HAL_BME280_Init\n");
	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	DRV_BME280_DBG("bme280_set_power_mode\n");
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
	DRV_BME280_DBG("bme280_set_oversamp_humidity\n");
	/* set the pressure oversampling*/
	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_2X);
	DRV_BME280_DBG("bme280_set_oversamp_pressure\n");
	/* set the temperature oversampling*/
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_4X);
	DRV_BME280_DBG("bme280_set_oversamp_temperature\n");
	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	DRV_BME280_DBG("bme280_set_standby_durn\n");
	/* This API used to read back the written value of standby time*/
	com_rslt += bme280_get_standby_durn(&v_stand_by_time_u8);
	OS_MSleep(100);
	DRV_BME280_DBG("bme280_get_standby_durn\n");
	COMPONENT_TRACK("end\n");
	return com_rslt;
}

/**
  * @brief Disable BME280.
  * @note This function is used to enable bme280 and deinit the i2c.
  * @retval None.
  */
void DRV_BME280_Disable()
{
	HAL_I2C_DeInit(BME280_I2cId);
	COMPONENT_TRACK("end\n");
}

/**
  * @brief Read BME280 data.
  * @note This function is used to BME280 data.
  * @param BME280_READ_MODE: The data that you want to read.
  * @retval BME280 data.
  */
int DRV_BME280_Read(BME280_READ_MODE mode)
{
	s32 v_comp_data_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	switch(mode) {
		case TEMPERATURE:
			if (bme280_read_uncomp_temperature(&v_comp_data_s32[0]) == -1) {
				DRV_BME280_DBG("read uncomp temperature error\n");
				return -1;
			}
			v_comp_data_s32[1] = bme280_compensate_temperature_int32(v_comp_data_s32[0]);
			break;
		case PRESSURE:
			if (bme280_read_uncomp_pressure(&v_comp_data_s32[0]) == -1) {
				DRV_BME280_DBG("read uncomp pressure error\n");
				return -1;
			}
			v_comp_data_s32[1] = bme280_compensate_pressure_int32(v_comp_data_s32[0]);
			break;
		case HUMIDITY:
			if (bme280_read_uncomp_humidity(&v_comp_data_s32[0]) == -1) {
				DRV_BME280_DBG("read uncomp humidity error\n");
				return -1;
			}
			v_comp_data_s32[1] = bme280_compensate_humidity_int32(v_comp_data_s32[0]);
			break;
		default:
			return -1;
		}
		return v_comp_data_s32[1];
}

/**
  * @brief BME280 sleep.
  * @note This function is used to BME280 run in power saving mode.
  * @retval None.
  */
void DRV_BME280_Sleep()
{
	bme280_set_power_mode(BME280_SLEEP_MODE);
	COMPONENT_TRACK("end\n");
}

/**
  * @brief BME280 wake up.
  * @note This function is used to wake up BME280.
  * @retval None.
  */
void DRV_BME280_WakeUp()
{
	bme280_set_power_mode(BME280_NORMAL_MODE);
	COMPONENT_TRACK("end\n");
}

#if 0
s32 DRV_BME280_Test()
{
	/* The variable used to read uncompensated temperature*/
	s32 v_data_uncomp_temp_s32 = BME280_INIT_VALUE;
	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_pres_s32 = BME280_INIT_VALUE;

	/* The variable used to read uncompensated pressure*/
	s32 v_data_uncomp_hum_s32 = BME280_INIT_VALUE;
	/* The variable used to read compensated temperature*/
	s32 v_comp_temp_s32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	/* The variable used to read compensated pressure*/
	u32 v_comp_press_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};
	/* The variable used to read compensated humidity*/
	u32 v_comp_humidity_u32[2] = {BME280_INIT_VALUE, BME280_INIT_VALUE};

	s32 com_rslt = BME280_ERROR;
	com_rslt = DRV_BME280_Enable();
while (1) {
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*------------------------------------------------------------------*/

/************************* END INITIALIZATION *************************/

/*------------------------------------------------------------------*
************ START READ UNCOMPENSATED PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/
/* API is used to read the uncompensated pressure*/
	int ret = bme280_read_uncomp_pressure(&v_data_uncomp_pres_s32);
	if(ret == -1) {
		DRV_BME280_DBG("read pressure error\n");
		continue;
	}

	/* API is used to read the uncompensated temperature*/
	com_rslt += bme280_read_uncomp_temperature(&v_data_uncomp_temp_s32);
	/* API is used to read the uncompensated humidity*/
	com_rslt += bme280_read_uncomp_humidity(&v_data_uncomp_hum_s32);
	/* API is used to read the uncompensated temperature,pressure
	and humidity data */
	com_rslt += bme280_read_uncomp_pressure_temperature_humidity(
	&v_data_uncomp_temp_s32, &v_data_uncomp_pres_s32, &v_data_uncomp_hum_s32);
/*--------------------------------------------------------------------*
************ END READ UNCOMPENSATED PRESSURE AND TEMPERATURE********
*-------------------------------------------------------------------------*/

/*------------------------------------------------------------------*
************ START READ COMPENSATED PRESSURE, TEMPERATURE
AND HUMIDITY DATA ********
*---------------------------------------------------------------------*/
	/* API is used to compute the compensated temperature*/
	v_comp_temp_s32[0] = bme280_compensate_temperature_int32(
			v_data_uncomp_temp_s32);
	/* API is used to compute the compensated pressure*/
	v_comp_press_u32[0] = bme280_compensate_pressure_int32(
			v_data_uncomp_pres_s32);
	/* API is used to compute the compensated humidity*/
	v_comp_humidity_u32[0] = bme280_compensate_humidity_int32(
			v_data_uncomp_hum_s32);
	/* API is used to read the compensated temperature, humidity and pressure*/
	com_rslt += bme280_read_pressure_temperature_humidity(
	&v_comp_press_u32[1], &v_comp_temp_s32[1],  &v_comp_humidity_u32[1]);

	DRV_BME280_DBG("v_data_uncomp_pres_s32 = %d\n", v_comp_press_u32[0]);//v_data_uncomp_pres_s32);
}

return com_rslt;
}
#endif

