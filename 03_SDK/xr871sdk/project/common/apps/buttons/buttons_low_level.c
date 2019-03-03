/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions
 *	are met:
 *	  1. Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *	  2. Redistributions in binary form must reproduce the above copyright
 *		 notice, this list of conditions and the following disclaimer in the
 *		 documentation and/or other materials provided with the
 *		 distribution.
 *	  3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *		 its contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "kernel/os/os.h"
#include "driver/hal_board.h"
#include "driver/hal_dev.h"

#include "buttons.h"
#include "buttons_low_level.h"

#define BUTTON_DBG  0
#define BUTTON_INTERRUPT_DBG 0 /* the debug macro used in interrupt function */

#if BUTTON_DBG
#define BUTTON_DEBUG(msg, arg...)      printf("[low level button debug] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)
#define BUTTON_WARNING(msg, arg...)    printf("[low level button warning] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)
#else
#define BUTTON_DEBUG(msg, arg...)
#define BUTTON_WARNING(msg, arg...)
#endif
#define BUTTON_ERR(msg, arg...)        printf("[low level button err] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)

#if BUTTON_INTERRUPT_DBG
#define BUTTON_IT_LOG(fmt, arg...)            		 \
    do {                                             \
            __nonxip_data static char __fmt[] = fmt; \
            printf(__fmt, ##arg);                    \
    } while (0)
#else
#define BUTTON_IT_LOG(fmt, arg...)
#endif

#define AD_FLUCTUATION 150
#define AD_MAX_VALUE 3500 /* all ad buttons channel's max ad value, this not an exact value, but important */
#define AD_MIN_VALUE 0

static OS_Semaphore_t buttons_sem;
static ad_button_info ad_buttons_info;
static gpio_button_info gpio_buttons_info;

__nonxip_text
static void ad_buttons_cb(void *arg) /* can't put this function into xip */
{
	ADC_Channel channel = (ADC_Channel)arg;
	int ad_value = 0, low = 0, high = 0;

	/* get button channel ad value */
	while ((ad_value = HAL_ADC_GetValue(channel)) == 0);

	/* calculate the interval of the AD value */
	low = ad_value - AD_FLUCTUATION < AD_MIN_VALUE ? AD_MIN_VALUE : ad_value - AD_FLUCTUATION;
	high = ad_value + AD_FLUCTUATION > AD_MAX_VALUE ? AD_MAX_VALUE : ad_value + AD_FLUCTUATION;

	/* reset the AD value of the interrupt trigger */
	if (low >= AD_MAX_VALUE)
		HAL_ADC_ConfigChannel(channel, ADC_SELECT_ENABLE, ADC_IRQ_LOW, AD_MAX_VALUE, 0);
	else
		HAL_ADC_ConfigChannel(channel, ADC_SELECT_ENABLE, ADC_IRQ_LOW_HIGH, low, high);

	/* release the buttons' semaphore */
	OS_SemaphoreRelease(&buttons_sem);

	BUTTON_IT_LOG("ad buttons interrupt callback, channel:%d value:%d\n", channel, ad_value);
}

static int ad_buttons_init(ADC_IRQCallback cb)
{
	HAL_Status sta;
	ADC_InitParam adc_init_param;
	uint8_t ad_buttons_channel_flag = 0;

	adc_init_param.delay = 10;
	adc_init_param.freq = 500000;
	adc_init_param.mode = ADC_CONTI_CONV;

	sta = HAL_ADC_Init(&adc_init_param);
	if (sta != HAL_OK && sta != HAL_BUSY) {
		BUTTON_ERR("adc init error");
		return -1;
	}

	/* get ad buttons info from board_config.c */
	HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_AD_BUTTON, 0), (uint32_t)&ad_buttons_info);

	int n = ad_buttons_info.count;
	int i;

	/* get which channel has ad buttons, ad buttons maybe in different channels. */
	for (i = 0; i < n; i++)
		ad_buttons_channel_flag |= 1 << ad_buttons_info.ad_buttons_p[i].channel;

	/* init each channel that has buttons */
	for (i = 0; i < ADC_CHANNEL_NUM - 1; i++) {
		if (ad_buttons_channel_flag >> i & 0x01) {
			BUTTON_DEBUG("config channel %d ", i);
			sta = HAL_ADC_ConfigChannel(i, ADC_SELECT_ENABLE, ADC_IRQ_LOW, AD_MAX_VALUE, 0);
			if (sta != HAL_OK) {
				BUTTON_ERR("channel %d config error", i);
				return -1;
			}
			sta = HAL_ADC_EnableIRQCallback(i, cb, (void*)i);
			if (sta != HAL_OK) {
				BUTTON_ERR("channel %d set callback error", i);
				return -1;
			}
		}
	}

	HAL_ADC_Start_Conv_IT();

	return 0;
}

static int ad_buttons_deinit(void)
{
	int n = ad_buttons_info.count;
	int i;
	uint8_t ad_buttons_channel_flag = 0;
	HAL_Status sta;

	/* get which channel has ad buttons, ad buttons maybe in different channels */
	for (i = 0; i < n; i++)
		ad_buttons_channel_flag |= 1 << ad_buttons_info.ad_buttons_p[i].channel;

	/* deinit each channel that has buttons */
	for (i = 0; i < ADC_CHANNEL_NUM - 1; i++) {
		if (ad_buttons_channel_flag >> i & 0x01) {
			BUTTON_DEBUG("disable irq callback channel %d ", i);
			sta = HAL_ADC_DisableIRQCallback(i);
			if (sta != HAL_OK) {
				BUTTON_ERR("channel %d disable callback error", i);
				return -1;
			}
		}
	}

	HAL_ADC_Stop_Conv_IT();
	HAL_ADC_DeInit();

	return 0;
}

__nonxip_text
static void gpio_buttons_cb(void *arg) /* can't put this function into xip */
{
	OS_SemaphoreRelease(&buttons_sem);

	BUTTON_IT_LOG("gpio buttons interrupt callback\n");
}

static int gpio_buttons_init(GPIO_IRQCallback cb)
{
	/* get gpio buttons info from board_config.c */
	HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_GPIO_BUTTON, 0), (uint32_t)&gpio_buttons_info);

	int n = gpio_buttons_info.count;
	int i;
	GPIO_IrqParam irq_param;

	irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
	irq_param.callback = cb;
	irq_param.arg = NULL;

	/* init each pin */
	for (i = 0; i < n; i++) {
		HAL_GPIO_Init(gpio_buttons_info.gpio_buttons_p[i].gpio_param.port,
						gpio_buttons_info.gpio_buttons_p[i].gpio_param.pin,
						&gpio_buttons_info.gpio_buttons_p[i].gpio_param.config);
		HAL_GPIO_EnableIRQ(gpio_buttons_info.gpio_buttons_p[i].gpio_param.port,
						gpio_buttons_info.gpio_buttons_p[i].gpio_param.pin,
						&irq_param);
	}

	return 0;
}

static int gpio_buttons_deinit(void)
{
	int n = gpio_buttons_info.count;
	int i;

	/* deinit each pin */
	for (i = 0; i < n; i++) {
		HAL_GPIO_DisableIRQ(gpio_buttons_info.gpio_buttons_p[i].gpio_param.port,
						gpio_buttons_info.gpio_buttons_p[i].gpio_param.pin);
		HAL_GPIO_DeInit(gpio_buttons_info.gpio_buttons_p[i].gpio_param.port,
						gpio_buttons_info.gpio_buttons_p[i].gpio_param.pin);
	}
	return 0;
}

static uint32_t get_adc_data(ADC_Channel channel)
{
	uint32_t ad_value = 0;
	/* wait until get the valid ad value */
	while ((ad_value = HAL_ADC_GetValue(channel)) == 0)
		OS_MSleep(1);
	return ad_value;
}

static int  get_ad_buttons_state(void)
{
	int ad_value = 0;
	int i;
	int n = ad_buttons_info.count;
	int ad_buttons_state = 0;
	int ad_buttons_is_tiggered = 0;

	/* get ad value and identify which button is pressed */
	for (i = 0; i < n; i++) {
		ad_value = get_adc_data(ad_buttons_info.ad_buttons_p[i].channel);
		BUTTON_DEBUG("first ad value:%d", ad_value);
		if (ad_value < AD_MAX_VALUE)
			ad_buttons_is_tiggered = 1;

		/* compare button's AD values */
		if ((ad_value >= ad_buttons_info.ad_buttons_p[i].value - AD_FLUCTUATION) &&
			(ad_value <= ad_buttons_info.ad_buttons_p[i].value + AD_FLUCTUATION)) {
			OS_MSleep(ad_buttons_info.ad_buttons_p[i].debounce_time); /* debounce time */
			ad_value = get_adc_data(ad_buttons_info.ad_buttons_p[i].channel);
			BUTTON_DEBUG("second ad value:%d", ad_value);
			/* compare button's AD values again */
			if ((ad_value >= ad_buttons_info.ad_buttons_p[i].value - AD_FLUCTUATION) &&
				(ad_value <= ad_buttons_info.ad_buttons_p[i].value + AD_FLUCTUATION)) {
				/* save the ad value, this can improve recognition accuracy */
				ad_buttons_info.ad_buttons_p[i].value = ad_value;
				/* record the button has pressed */
				ad_buttons_state |= ad_buttons_info.ad_buttons_p[i].mask;
				BUTTON_DEBUG("ad value is %d, %s has press", ad_value, ad_buttons_info.ad_buttons_p[i].name);
			}
		}
	}

	/* if the ad buttons has been tiggered but do not known which button pressed,
	 * then judge the second ad vlaue.
	 */
	if (!ad_buttons_state && ad_buttons_is_tiggered) {
		/* if the ad value very large, that mean all buttons has release,
		 * else it was just a false trigger.
		 */
		if (ad_value > AD_MAX_VALUE)
			return ALL_BUTTONS_RELEASE;
		else
			return INVALID_BUTTON_STATE;
	}

	return ad_buttons_state;
}

static int get_gpio_buttons_state(void)
{
	int i;
	int n = gpio_buttons_info.count;
	GPIO_PinState sta;
	int gpio_buttons_state = 0;

	/* read each pin and identify which button is pressed */
	for (i = 0; i < n; i++) {
		sta = HAL_GPIO_ReadPin(gpio_buttons_info.gpio_buttons_p[i].gpio_param.port,
								gpio_buttons_info.gpio_buttons_p[i].gpio_param.pin);
		if ((gpio_buttons_info.gpio_buttons_p[i].active_low && sta == GPIO_PIN_LOW) ||
			(!gpio_buttons_info.gpio_buttons_p[i].active_low && sta == GPIO_PIN_HIGH)) {
			OS_MSleep(gpio_buttons_info.gpio_buttons_p[i].debounce_time); /* debounce time */
			sta = HAL_GPIO_ReadPin(gpio_buttons_info.gpio_buttons_p[i].gpio_param.port,
									gpio_buttons_info.gpio_buttons_p[i].gpio_param.pin);
			if ((gpio_buttons_info.gpio_buttons_p[i].active_low && sta == GPIO_PIN_LOW) ||
			(!gpio_buttons_info.gpio_buttons_p[i].active_low && sta == GPIO_PIN_HIGH)) {
				gpio_buttons_state |= gpio_buttons_info.gpio_buttons_p[i].mask;
				BUTTON_DEBUG("%s has press", gpio_buttons_info.gpio_buttons_p[i].name);
			}
		}
	}

	return gpio_buttons_state;
}

/**
  * @brief Get buttons state from low level buttons.
  * @note This interface will get ad and gpio buttons' state. The return value
  *       is a 32-bit integer, one bit one button, 1 mean the button has pressed,
  *       0 mean the button has release.
  * @param void
  * @retval 0: all buttons has released
  *         0xffffffff: invalid button state
  *         others: valid button state
  */
int buttons_low_level_get_state(void)
{
	int ad_buttons_state = 0;
	int gpio_buttons_state = 0;

	ad_buttons_state = get_ad_buttons_state();
	if (ad_buttons_state == INVALID_BUTTON_STATE)
		return INVALID_BUTTON_STATE;

	gpio_buttons_state = get_gpio_buttons_state();

	return ad_buttons_state | gpio_buttons_state;
}

/**
  * @brief Wait the low level buttons' semaphore.
  * @note Upper application use this interface to wait buttons' event in blocking mode.
  * @param waitms: wait timeout. OS_WAIT_FOREVER for waiting forever, zero for no waiting.
  * @retval 0 on success.
  */
int buttons_low_level_wait_semaphore(uint32_t waitms)
{
	return OS_SemaphoreWait(&buttons_sem, waitms);
}

/**
  * @brief Release the low level buttons' semaphore.
  * @note Upper application use this interface to release buttons' event.
  *       Upper application can use this interface to prevent the app from waiting for
  *       button events all the time.
  * @param void
  * @retval 0 on success.
  */
int buttons_low_level_release_semaphore(void)
{
	return OS_SemaphoreRelease(&buttons_sem);
}

/**
  * @brief Initialize the low level buttons.
  * @note This interface will initialize all the buttons, include ad and gpio buttons.
  * @param void
  * @retval 0: success, -1: fail
  */
int buttons_low_level_init(void)
{
	OS_Status sta;

	ad_buttons_init(ad_buttons_cb);
	gpio_buttons_init(gpio_buttons_cb);

	sta = OS_SemaphoreCreate(&buttons_sem, 0, 1);
	if (sta != OS_OK) {
		BUTTON_ERR("buttons semaphore create error");
		return -1;
	}
	return 0;
}

/**
  * @brief deinitialize the low level buttons.
  * @note This interface will deinitialize all the buttons, include ad and gpio buttons.
  * @param void
  * @retval 0: success
  */
int buttons_low_level_deinit(void)
{
	ad_buttons_deinit();
	gpio_buttons_deinit();
	return 0;
}

