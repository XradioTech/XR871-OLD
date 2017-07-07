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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys/io.h"
#include "errno.h"
#include "sys/list.h"

#include "driver/chip/device.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_nvic.h"

#include "pm/pm.h"
#include "pm_i.h"
#include "port.h"

#ifdef CONFIG_PM

/* It is better that all interrupts are managed by itself, the interrupt should
 * be disable by it's driver if it is not used during suspend. We use this mask
 * for a sample concentrated cometrue, 1: use this irq as wakeup source */
unsigned int nvic_int_mask[] = {
	NVIC_PERIPH_IRQ_MASK0,
#if (MAX_IRQn > 32)
	NVIC_PERIPH_IRQ_MASK1,
#endif
};

ct_assert((sizeof(nvic_int_mask) + 3) / 4 >= (NVIC_PERIPH_IRQ_NUM + 31)/32);

/* delay us time, 32~100000 us */
void udelay(unsigned int us)
{
#if defined(__CONFIG_CHIP_XR871)
	unsigned long long expire = 0;

	if (us < 5 || us > 100000)
		return;

	expire = (us / 32) + HAL_RTC_Get32kConter();
	while (expire > HAL_RTC_Get32kConter())
		;
#else
	unsigned int cpu_clk;

	if (us < 32 || us > 100000)
		return;

	cpu_clk = HAL_PRCM_GetCPUAClk() / 2000;

	for (int i = 0; i < cpu_clk; i++)
		i = i;
#endif
}

void loop_delay(unsigned int ms)
{
#if defined(__CONFIG_CHIP_XR871)
	udelay(ms * 1000);
#else
	for (volatile int i = 0; i < ms*16000; i++)
		i = i;
#endif
}

static unsigned int console_suspend_enabled = 0;

void pm_console_set_enable(unsigned int enable)
{
	console_suspend_enabled = enable;
}

#ifdef CONFIG_PM_DEBUG
#define PM_UART_PRINT_BUF_LEN 512
static volatile uint32_t pm_print_index;
static char pm_print_buf[PM_UART_PRINT_BUF_LEN];
#endif

int pm_console_write(char *buf, int count)
{
#ifdef CONFIG_PM_DEBUG
	if (pm_print_index + count < (PM_UART_PRINT_BUF_LEN - 1)) {
		memcpy(pm_print_buf + pm_print_index, buf, count);
		pm_print_index += count;

		return count;
	}
#endif

	return 0;
}

int pm_console_print(void)
{
	int len = 0;

#ifdef CONFIG_PM_DEBUG
	if (pm_print_index) {
		PM_LOGD("%s", pm_print_buf);
		len = pm_print_index;
		pm_print_index = 0;
	}
#endif
	return len;
}

/**
 * suspend_console - suspend the console subsystem
 *
 * This disables printf() while we go into suspend states
 */
void suspend_console(void)
{
	if (!console_suspend_enabled)
		return;
	PM_LOGD("Suspending console(s) (use no_console_suspend to debug)\n");
}

void resume_console(void)
{
	if (!console_suspend_enabled)
		return;
}

void debug_jtag_init(void)
{
#ifdef CONFIG_PM_DEBUG
	/* at this time gpio is reset state */
/*
	GPIO_InitParam param;

	param.mode = GPIOB_P2_F2_SWD_TMS;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.pull = GPIO_PULL_UP;
	HAL_GPIO_Init(GPIO_PORT_B, GPIO_PIN_2, &param);

	param.mode = GPIOB_P3_F2_SWD_TCK;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.pull = GPIO_PULL_UP;
	HAL_GPIO_Init(GPIO_PORT_B, GPIO_PIN_3, &param);
*/
#endif
}

void debug_jtag_deinit(void)
{
#ifdef CONFIG_PM_DEBUG
	//HAL_GPIO_DeInit(GPIO_PORT_B, GPIO_PIN_2);
	//HAL_GPIO_DeInit(GPIO_PORT_B, GPIO_PIN_3);
#endif
}

void platform_wake(enum suspend_state_t state)
{
	HAL_NVIC_EnableIRQ(N_UART_IRQn);
}

#endif
