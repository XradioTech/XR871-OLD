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

#include "sys/io.h"
#include "driver/chip/hal_wakeup.h"
#include "hal_base.h"

#define HAL_DBG_WAKEUP 1

#if (HAL_DBG_WAKEUP == 1)
#define WK_INF(fmt, arg...) HAL_LOG(HAL_DBG_ON && 0, "[WK] "fmt, ##arg)
#define WK_WAR(fmt, arg...) HAL_LOG(HAL_DBG_ON && HAL_DBG_WAKEUP, "[WK] "fmt, ##arg)
#define WK_ERR(fmt, arg...) HAL_LOG(HAL_DBG_ON && HAL_DBG_WAKEUP, "[WK] "fmt, ##arg)
#else
#define WK_INF(fmt, arg...)
#define WK_WAR(fmt, arg...)
#define WK_ERR(fmt, arg...)
#endif

static uint32_t wakeup_event;

#ifdef __CONFIG_ARCH_APP_CORE
static void Wakeup_ClrIO()
{
	HAL_PRCM_WakeupIODisableCfgHold(0x0ff);
	HAL_PRCM_WakeupIODisableGlobal();
	HAL_PRCM_WakeupIOSetFallingEvent(0x0ff);
	HAL_PRCM_WakeupIODisable(0x0ff);

	HAL_PRCM_WakeupIOClearEventDetected(0x0ff);
}
#endif

void HAL_Wakeup_ClrTimer(void)
{
	HAL_PRCM_WakeupTimerDisable();
	while (HAL_PRCM_WakeupTimerGetCurrentValue())
		;
}

static void Wakeup_Source_Handler(void)
{
	if (HAL_PRCM_GetWakeupTimerPending()) {
		HAL_PRCM_ClearWakeupTimerPending();

		while (HAL_PRCM_GetWakeupTimerPending())
			;
	}

	HAL_Wakeup_ClrTimer();
#ifdef __CONFIG_ARCH_APP_CORE
	Wakeup_ClrIO();
#endif

	NVIC_DisableIRQ(A_WAKEUP_IRQn);
	NVIC_ClearPendingIRQ(A_WAKEUP_IRQn);

	/* maybe call user wakeup callback fun */

	WK_INF("%s %x\n", __func__, wakeup_event);
}

#ifdef WAKEUP_TIMER_CHECK_TIME
static uint32_t wakeup_time_back = 0xffffffff;
#endif

/* should set wakeup timer everytime if you want wake up system from suspend.
 * count_32k: the count of 32k crystal, from
 *             WAKEUP_TIMER_MIN_TIME*32(WAKEUP_TIMER_MIN_TIME mS) to 134217727(4194.303S).
 * return: -1: err, 0: ok.
 */
int32_t HAL_Wakeup_SetTimer(uint32_t count_32k)
{
#ifdef WAKEUP_TIMER_CHECK_TIME
	uint32_t current_count;
	unsigned long flags;
#endif

	if ((count_32k < (32*WAKEUP_TIMER_MIN_TIME)) || (count_32k & PRCM_CPUx_WAKE_TIMER_EN_BIT))
		return -1;

#ifdef WAKEUP_TIMER_CHECK_TIME
	flags = xr_irq_save();
	current_count = HAL_PRCM_WakeupTimerGetCurrentValue();
	if (wakeup_time_back > current_count)
		wakeup_time_back -= current_count;
	else
		WK_WAR("WAR:%s,%d\n", __func__, __LINE__);

	if (wakeup_time_back <= count_32k) {
		xr_irq_restore(flags);
		WK_WAR("ignor time set, bk:%d cu:%d\n", wakeup_time_back, count_32k);
		return -1;
	}

	wakeup_time_back = count_32k;
	xr_irq_restore(flags);
#endif
	HAL_Wakeup_ClrTimer();
	HAL_PRCM_WakeupTimerSetCompareValue(count_32k);
	HAL_PRCM_WakeupTimerEnable();

	return 0;
}

#ifdef __CONFIG_ARCH_APP_CORE
static uint32_t wakeup_io_en;
static uint32_t wakeup_io_mode;

/* set once will work forever.
 * pn: 0~9
 * mode: 0:negative edge, 1:positive edge
 */
void HAL_Wakeup_SetIO(uint32_t pn, uint32_t mode)
{
	if (pn >= WAKEUP_IO_MAX || mode > 1) {
		WK_ERR("%s,%d err\n", __func__, __LINE__);
		return;
	}

	/* mode */
	if (mode)
		wakeup_io_mode |= BIT(pn);
	else
		wakeup_io_mode &= ~BIT(pn);

	/* enable */
	wakeup_io_en |= BIT(pn);
}

/* pn: 0~7
 * mode: 0:negative edge, 1:positive edge
 */
void HAL_Wakeup_ClrIO(uint32_t pn)
{
	wakeup_io_en &= ~BIT(pn);
}

static GPIO_Pin WakeIo_To_Gpio(uint32_t wkup_io)
{
	switch (wkup_io) {
	case 0: return WAKEUP_IO0;
	case 1: return WAKEUP_IO1;
	case 2: return WAKEUP_IO2;
	case 3: return WAKEUP_IO3;
	case 4: return WAKEUP_IO4;
	case 5: return WAKEUP_IO5;
	case 6: return WAKEUP_IO6;
	case 7: return WAKEUP_IO7;
	case 8: return WAKEUP_IO8;
	case 9: return WAKEUP_IO9;
	}

	WK_ERR("%s,%d err!\n", __func__, __LINE__);

	return 0;
}

int32_t HAL_Wakeup_SetIOHold(uint32_t hold_io)
{
	/* clear */
	HAL_PRCM_WakeupIOClearEventDetected(HAL_PRCM_WakeupIOGetEventStatus());
	/* set hold */
	HAL_PRCM_WakeupIOEnableCfgHold(hold_io);

	return 0;
}
#endif

int32_t HAL_Wakeup_SetSrc(void)
{
#ifdef __CONFIG_ARCH_APP_CORE
	uint32_t i, wkio_input;
#endif

	/* check wakeup time */
	if (HAL_PRCM_GetWakeupTimerEnable() &&
	    (HAL_PRCM_WakeupTimerGetCompareValue() <= (32*WAKEUP_TIMER_MIN_TIME))) {
		WK_ERR("wakeup timer err:%x\n", HAL_PRCM_WakeupTimerGetCurrentValue());
		return -1;
	}

#ifdef __CONFIG_ARCH_APP_CORE
	/* enable wakeup gpio if configed wakeup io */
	if (wakeup_io_en) {
		wkio_input = wakeup_io_en;
		for (i = 0; (i < WAKEUP_IO_MAX) && wkio_input; wkio_input >>= 1, i++) {
			if (wkio_input & 0x01) {
				GPIO_InitParam param;

				param.mode = GPIOx_Pn_F0_INPUT;
				param.driving = GPIO_DRIVING_LEVEL_1;
				param.pull = GPIO_PULL_UP;
				WK_INF("init io:%d\n", WakeIo_To_Gpio(i));
				HAL_GPIO_Init(GPIO_PORT_A, WakeIo_To_Gpio(i), &param); /* set input */
			}
		}
		/* clear */
		HAL_PRCM_WakeupIOClearEventDetected(HAL_PRCM_WakeupIOGetEventStatus());
		/* mode */
		HAL_PRCM_WakeupIOSetRisingEvent(wakeup_io_mode);
		/* enable */
		HAL_PRCM_WakeupIOEnable(wakeup_io_en);
		/* enable global wakeup io */
		HAL_PRCM_WakeupIOEnableGlobal();
		/* set hold if this io enable */
		HAL_PRCM_WakeupIOEnableCfgHold(wakeup_io_en);
	}
#endif

	NVIC_EnableIRQ(A_WAKEUP_IRQn); /* enable when sleep */

	return 0;
}

void HAL_Wakeup_ClrSrc(void)
{
#ifdef __CONFIG_ARCH_APP_CORE
	uint32_t i, wkio_input;
#endif
#ifdef WAKEUP_TIMER_CHECK_TIME
	unsigned long flags;
#endif

#ifdef __CONFIG_ARCH_APP_CORE
	/* wakeup io event */
	wakeup_event = HAL_PRCM_WakeupIOGetEventStatus();
#endif

	/* wakeup timer event */
	if (HAL_PRCM_GetWakeupTimerPending()) {
		HAL_PRCM_ClearWakeupTimerPending();
		while (HAL_PRCM_GetWakeupTimerPending())
			;
		wakeup_event |= PM_WAKEUP_SRC_WKTIMER;
#ifdef WAKEUP_TIMER_CHECK_TIME
		flags = xr_irq_save();
		wakeup_time_back = 0xffffffff;
		xr_irq_restore(flags);
#endif
	}

	/* no events is net cpu sev */
	if (!wakeup_event)
		wakeup_event = PM_WAKEUP_SRC_WKSEV;

#ifdef __CONFIG_ARCH_APP_CORE
	if (wakeup_io_en) {
		wkio_input = wakeup_io_en;
		for (i = 0; (i < WAKEUP_IO_MAX) && wkio_input; wkio_input >>= 1, i++) {
			if (wkio_input & 0x01) {
				HAL_GPIO_DeInit(GPIO_PORT_A, WakeIo_To_Gpio(i));
				WK_INF("deinit io:%d\n", i);
			}
		}
	}
#endif

	NVIC_EnableIRQ(A_WAKEUP_IRQn);
}

/**
 * query super-standby wakeup source.
 * @para:  point of buffer to store wakeup event informations.
 *
 * return: wakeup event;
 */
uint32_t HAL_Wakeup_GetEevent(void)
{
	return wakeup_event;
}

void HAL_Wakeup_Init(void)
{
#ifdef __CONFIG_ARCH_APP_CORE
	Wakeup_ClrIO();
#endif
	HAL_Wakeup_ClrTimer();

	HAL_NVIC_SetIRQHandler(A_WAKEUP_IRQn, Wakeup_Source_Handler);
}

void HAL_Wakeup_DeInit(void)
{
	NVIC_DisableIRQ(A_WAKEUP_IRQn);

#ifdef __CONFIG_ARCH_APP_CORE
	Wakeup_ClrIO();
#endif
	HAL_Wakeup_ClrTimer();
}
