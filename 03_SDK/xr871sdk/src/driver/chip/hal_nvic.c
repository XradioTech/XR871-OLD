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

#include "hal_base.h"
#include "pm/pm.h"
#include "sys/param.h"
#include "sys/xr_debug.h"

void HAL_NVIC_SetIRQHandler(IRQn_Type IRQn, NVIC_IRQHandler handler)
{
    uint32_t *vectors = (uint32_t *)SCB->VTOR;

    vectors[IRQn + NVIC_PERIPH_IRQ_OFFSET] = (uint32_t)handler;
}

NVIC_IRQHandler HAL_NVIC_GetIRQHandler(IRQn_Type IRQn)
{
    uint32_t *vectors = (uint32_t*)SCB->VTOR;

    return (NVIC_IRQHandler)(vectors[IRQn + NVIC_PERIPH_IRQ_OFFSET]);
}

void HAL_NVIC_SetPriorityGrouping(uint32_t priorityGroup)
{
	NVIC_SetPriorityGrouping(priorityGroup);
}

uint32_t HAL_NVIC_GetPriorityGrouping(void)
{
	return NVIC_GetPriorityGrouping();
}

void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
	NVIC_SetPriority(IRQn, priority);
}

uint32_t HAL_NVIC_GetPriority(IRQn_Type IRQn)
{
	return NVIC_GetPriority(IRQn);
}

void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
	NVIC_EnableIRQ(IRQn);
}

void HAL_NVIC_DisableIRQ(IRQn_Type IRQn)
{
	NVIC_DisableIRQ(IRQn);
}

void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
	NVIC_SetPendingIRQ(IRQn);
}

int HAL_NVIC_IsPendingIRQ(IRQn_Type IRQn)
{
	return (int)(NVIC_GetPendingIRQ(IRQn));
}

void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
	NVIC_ClearPendingIRQ(IRQn);
}

#ifdef CONFIG_PM
struct nvic_regs {
	uint32_t vector_table;          /* Vector Table Offset */
	uint32_t int_ctrl_state;        /* Interrupt Control and State */
	uint32_t app_int;               /* Application Interrupt Reset control */
	uint32_t sys_ctrl;              /* System control */
	uint32_t config_ctrl;           /* Configuration control */
	uint32_t sys_pri_1;             /* System Handler Priority 1 */
	uint32_t sys_pri_2;             /* System Handler Priority 2 */
	uint32_t sys_pri_3;             /* System Handler Priority 3 */
	uint32_t sys_hcrs;              /* System Handler control and state register */
	uint32_t systick_ctrl;          /* SysTick Control Status */
	uint32_t systick_reload;        /* SysTick Reload */
	//uint32_t systick_calib;         /* SysTick Calibration */
	uint32_t int_en[DIV_ROUND_UP(NVIC_VECTOR_TABLE_SIZE, 32)];             /* Interrupt set enable */
	uint8_t int_priority[NVIC_PERIPH_IRQ_NUM*4];     /* Interrupt priority */
};

static struct nvic_regs nvic_reg_store;

static int nvic_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	uint32_t i = 0;
	volatile uint32_t *reg_en_addr;
	volatile uint8_t *reg_ip_addr;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		/* Save the NVIC control registers */
		nvic_reg_store.vector_table = SCB->VTOR;
		nvic_reg_store.int_ctrl_state = SCB->ICSR;
		nvic_reg_store.app_int = SCB->AIRCR;
		nvic_reg_store.sys_ctrl = SCB->SCR;
		nvic_reg_store.config_ctrl = SCB->CCR;
		nvic_reg_store.sys_pri_1 = SCB->SHP[0];
		nvic_reg_store.sys_pri_2 = SCB->SHP[1];
		nvic_reg_store.sys_pri_3 = SCB->SHP[2];
		nvic_reg_store.sys_hcrs = SCB->SHCSR;

		/* Systick registers */
		nvic_reg_store.systick_ctrl = SysTick->CTRL;
		nvic_reg_store.systick_reload = SysTick->LOAD;
		//nvic_reg_store.systick_calib = SysTick->CALIB;

		/* Save the interrupt enable registers */
		reg_en_addr = NVIC->ISER;
		for (i = 0; i < DIV_ROUND_UP(NVIC_VECTOR_TABLE_SIZE, 32); i++) {
			nvic_reg_store.int_en[i] = reg_en_addr[i];
			//NVIC->ICER[i] = 0xffffffff; /* disable all ints */
		}

		/* Save the interrupt priority registers */
		reg_ip_addr = NVIC->IP;
		for (i = 0; i < sizeof(nvic_reg_store.int_priority); i++) {
			nvic_reg_store.int_priority[i] = reg_ip_addr[i];
		}
		HAL_DBG("%s okay\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}

static int nvic_resume(struct soc_device *dev, enum suspend_state_t state)
{
	uint32_t i = 0;
	volatile uint32_t *reg_en_addr;
	volatile uint8_t *reg_ip_addr;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		/* Restore the NVIC control registers */
		SCB->VTOR = nvic_reg_store.vector_table;
		SCB->AIRCR = (nvic_reg_store.app_int & 0x0ffff) | (0x5FA << SCB_AIRCR_VECTKEY_Pos);
		SCB->SCR = nvic_reg_store.sys_ctrl;
		SCB->CCR = nvic_reg_store.config_ctrl;
		SCB->SHP[0] = nvic_reg_store.sys_pri_1;
		SCB->SHP[1] = nvic_reg_store.sys_pri_2;
		SCB->SHP[2] = nvic_reg_store.sys_pri_3;
		SCB->SHCSR = nvic_reg_store.sys_hcrs;

		/* Systick registers */
		SysTick->CTRL = nvic_reg_store.systick_ctrl;
		SysTick->LOAD = nvic_reg_store.systick_reload;
		//SysTick->CALIB = nvic_reg_store.systick_calib;

		/* Restore the interrupt priority registers */
		reg_ip_addr = NVIC->IP;
		for (i = 0; i < sizeof(nvic_reg_store.int_priority); i++) {
			reg_ip_addr[i] = nvic_reg_store.int_priority[i];
		}

		/* Restore the interrupt enable registers */
		reg_en_addr = NVIC->ISER;
		for (i = 0; i < DIV_ROUND_UP(NVIC_VECTOR_TABLE_SIZE, 32); i++) {
			reg_en_addr[i] = nvic_reg_store.int_en[i];
		}

		__asm(" dsb \n");
		__asm(" isb \n");
		HAL_DBG("%s okay\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}

void nvic_print_regs(void)
{
	hex_dump_bytes(&nvic_reg_store, sizeof(nvic_reg_store));
}

static struct soc_device_driver nvic_drv = {
	.name = "nvic",
	.suspend_noirq = nvic_suspend,
	.resume_noirq = nvic_resume,
};

static struct soc_device nvic_dev = {
	.name = "nvic",
	.driver = &nvic_drv,
};

#define NVIC_DEV (&nvic_dev)
#else
#define NVIC_DEV NULL
#endif

void HAL_NVIC_Init(void)
{
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk |
	             SCB_SHCSR_MEMFAULTENA_Msk;

#ifdef CONFIG_PM
	pm_register_ops(NVIC_DEV);
#endif
}
