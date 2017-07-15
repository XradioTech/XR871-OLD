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

#include <string.h>
#include "driver/chip/hal_irrx.h"
#include "driver/chip/ir_nec.h"
#include "driver/chip/hal_nvic.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_clock.h"
#include "pm/pm.h"
#include "hal_debug.h"

#define HAL_DBG_IRRX  0

#if (HAL_DBG_IRRX == 1)
#define HAL_IRRX_DBG(fmt, arg...) HAL_LOG(HAL_DEBUG_ON && HAL_DBG_IRRX, "[IRRX] "fmt, ##arg)
#define irrx_hex_dump_bytes(...) print_hex_dump_bytes(__VA_ARGS__)
#define IRRX_INF HAL_IRRX_DBG
#define IRRX_WRN HAL_WARN
#define IRRX_ERR HAL_ERR
#else
#define HAL_IRRX_DBG(fmt, arg...)
#define irrx_hex_dump_bytes(...)
#define IRRX_INF(...)
#define IRRX_WRN(...)
#define IRRX_ERR(...)
#endif

/* reg 0x00: Bit definition for cir receiver control register */
#define IRRX_GLOBAL_EN          (0x00000001) /* bit0 */
#define IRRX_RX_EN              (0x00000002) /* bit1 */
#define IRRX_CIR_EN             (0x00000030) /* bit4~bit5 */

/* reg 0x2c: Bit definition for cir receiver interrupt control */
#define IRRX_RXFIFO_MASK        (0x000000FF) /* bit0~bit7 */
#define IR_FIFO_SIZE            (64)
#define IR_FIFO_TRIGER          (32)

#define IRRX_IT_ROI             (0x00000001) /* bit0 receiver fifo overrun interrupt enable */
#define IRRX_IT_RPEI            (0x00000002) /* bit1 receiver packet end interrupt enable */
#define IRRX_IT_RAI             (0x00000010) /* bit4 RX FIFO Available interrupt enable */
#define IRRX_IT_DRQ             (0x00000020) /* bit5 RX FIFO DMA enable */
#define IRRX_IT_MASK            (IRRX_IT_ROI | IRRX_IT_RPEI | IRRX_IT_RAI | IRRX_IT_DRQ)

#define IRRX_SET_RAL_MASK       (0x00003F00) //bit8~bit13 RX FIFO available received byte level
#define IRRX_RAL_INDEX          (8)

#define IRRX_STATUS_STAT        (0x00000080) /* bit7 */
#define IRRX_STATUS_RAC_MASK    (0x00003F00) /* bit8~bit14 */

/* reg 0x30: Bit definition for cir receiver status */
#define IRRX_FLAG_ROI           (0x00000001) /* Rx FIFO Overflow */
#define IRRX_FLAG_RPE           (0x00000002) /* Rx Packet End */
#define IRRX_FLAG_RA            (0x00000010) /* Rx FIFO Data Available */
#define IRRX_FLAG_MASK          (IRRX_FLAG_ROI | IRRX_FLAG_RPE | IRRX_FLAG_RA)

/* reg 0x34: Bit definition for cir receiver sample clock select prescaler */
#define IRRX_SCS_DIV_64         (0x00000000)
#define IRRX_SCS_DIV_128        (0x00000001)
#define IRRX_SCS_DIV_256        (0x00000002)
#define IRRX_SCS_DIV_512        (0x00000003)
#define IRRX_SCS_DIV_NONE       (0x01000000)
#define IRRX_SCS_DIV_MASK       (0x00010003) /* clear mask */

#define IRRX_NTHR_MASK          (0x000000FC) /* bit2~bit7 */
#define IRRX_NTHR_INDEX         (2)

#define IRRX_ITHR_MASK          (0x0000FF00) /* bit8~bit15 */
#define IRRX_ITHR_INDEX         (8)

#define IRRX_ATHR_MASK          (0x007F0000) /* bit16~bit22 */
#define IRRX_ATHR_INDEX         (16)

#define IRRX_ATHC_CLOCKUNIT_1   (0)
#define IRRX_ATHC_CLOCKUNIT_128 (1)
#define IRRX_ATHC_INDEX         (23)

#if defined (IR_CLK_32K_USED)
#define IRRX_32K_SAMPLE_CLK             IRRX_SCS_DIV_NONE       /* sample_freq = 32768Hz/1=32768Hz (30.5us) */
#define IRRX_32K_RXFILT_VAL             (11)                    /* NosieTreshold, Filter Threshold = 11*30.5 = ~335.5us < 500us */
#define IRRX_32K_RXIDLE_VAL             (3)                     /* IdleTreshold, Idle Threshold = (3+1)*128*30.5 = ~16.4ms > 9ms */
#define IRRX_32K_APB_PERIPH_CLK_SRC     CCM_APB_PERIPH_CLK_SRC_LFCLK
#define IRRX_32K_CCM_PERIPH_CLK_DIV_N   CCM_PERIPH_CLK_DIV_N_1
#define IRRX_32K_CCM_PERIPH_CLK_DIV_M   CCM_PERIPH_CLK_DIV_M_1
#else
#define IRRX_26M_SAMPLE_CLK             IRRX_SCS_DIV_128        /* 26MHz/8/128=25390.6Hz (39.4us) */
#define IRRX_26M_RXFILT_VAL             (8)                     /* NosieTreshold, Filter Threshold = 8*39.4 = ~315us < 500us */
#define IRRX_26M_RXIDLE_VAL             (3)                     /* IdleTreshold, Idle Threshold = (3+1)*128*39.4 = ~20.2ms > 9ms */
#define IRRX_26M_APB_PERIPH_CLK_SRC     CCM_APB_PERIPH_CLK_SRC_HFCLK
#define IRRX_26M_CCM_PERIPH_CLK_DIV_N   CCM_PERIPH_CLK_DIV_N_1
#define IRRX_26M_CCM_PERIPH_CLK_DIV_M   CCM_PERIPH_CLK_DIV_M_8

#define IRRX_24M_SAMPLE_CLK             IRRX_SCS_DIV_128        /* 24MHz/8/128=23437.5Hz (42.7us) */
#define IRRX_24M_RXFILT_VAL             (8)                     /* NosieTreshold, Filter Threshold = 8*42.7 = ~341us < 500us */
#define IRRX_24M_RXIDLE_VAL             (2)                     /* IdleTreshold, Idle Threshold = (2+1)*128*42.7 = ~16.4ms > 9ms */
#define IRRX_24M_APB_PERIPH_CLK_SRC     CCM_APB_PERIPH_CLK_SRC_HFCLK
#define IRRX_24M_CCM_PERIPH_CLK_DIV_N   CCM_PERIPH_CLK_DIV_N_1
#define IRRX_24M_CCM_PERIPH_CLK_DIV_M   CCM_PERIPH_CLK_DIV_M_8
#endif

typedef struct
{
	__IO uint32_t   CR;             /* 0x00, IRRX Control */
	     uint32_t   RESERVED0[3];
	__IO uint32_t   CFR;            /* 0x10, IRRX Configure */
	     uint32_t   RESERVED1[3];
	__IO uint32_t   DR;             /* 0x20, IRRX FIFO */
	     uint32_t   RESERVED2[2];
	__IO uint32_t   ICR;            /* 0x2C, IRRX Interrupt Control */
	__IO uint32_t   SR;             /* 0x30, IRRX Status */
	__IO uint32_t   CCR;            /* 0x34, IRRX Control Configure */
} IRRX_TypeDef;

/* IRRX State structures definition */
typedef enum
{
	IRRX_STATE_RESET        = 0x00, /* Peripheral is not yet Initialized */
	IRRX_STATE_READY        = 0x02, /* Peripheral Initialized and ready for use */
	IRRX_STATE_BUSY         = 0x04, /* An internal process is ongoing */
	IRRX_STATE_ERROR        = 0x08  /* Error */
} IRRX_StateTypeDef;

#define IRRX_RAW_BUF_SIZE       128     /* 256 */

/* IRRX handle Structure definition */
typedef struct
{
	IRRX_TypeDef            *Instance;                  /* IRRX registers base address */
	uint32_t                rxCnt;                      /* IRRX Rx Transfer layer Counter */
	uint8_t                 rxBuff[IRRX_RAW_BUF_SIZE];  /* Pointer to IRRX Rx transfer Buffer */
	IRRX_StateTypeDef       State;                      /* IRRX communication state */
	HAL_BoardCfg            boardCfg;
	IRRX_RxCpltCallback     rxCpltCallback;
} IRRX_HandleTypeDef;

/**
 * read an amount of data in fifo.
 * @hirrx: Pointer to a IRRX_HandleTypeDef structure that contains
 *          the configuration information for the specified IRRX module.
 * @ralcnt: Count of data to read.
 * @retval HAL status
 */
static HAL_Status IRRX_ReadFIFO_RawData(IRRX_HandleTypeDef *hirrx, uint32_t ralcnt)
{
	uint32_t i, tmp;

	if (ralcnt == 0)
		return HAL_INVALID;

	for (i = 0; i < ralcnt; i++) {
		if (hirrx->rxCnt >= IRRX_RAW_BUF_SIZE)
			tmp = hirrx->Instance->DR;
		else
			hirrx->rxBuff[hirrx->rxCnt++] = (uint8_t)(hirrx->Instance->DR);
	}

	(void)tmp;

	return HAL_OK;
}

static IRRX_HandleTypeDef hal_irrx;

/**
 * handles IRRX interrupt request.
 */
static void IRRX_IRQHandler(void)
{
	uint32_t intsta, dcnt;
	IRRX_HandleTypeDef *hirrx = &hal_irrx;

	intsta = hirrx->Instance->SR;
	hirrx->Instance->SR = intsta & IRRX_FLAG_MASK;
	IRRX_INF("%s,%d %x\n", __func__, __LINE__, intsta);

	dcnt = (intsta & IRRX_SET_RAL_MASK) >> IRRX_RAL_INDEX;
	IRRX_ReadFIFO_RawData(hirrx, dcnt);

	hirrx->State = IRRX_STATE_READY;

	if (intsta & IRRX_FLAG_RPE) { /* Packet End */
		uint32_t code;
		int32_t code_valid;

		if (hirrx->rxCnt >= IRRX_RAW_BUF_SIZE) {
			IRRX_INF("Raw Buffer Full!!\n");
			hirrx->rxCnt = 0;
			return ;
		}

		code = IRRX_NECPacket_DeCode(hirrx->rxBuff, hirrx->rxCnt);
		hirrx->rxCnt = 0;
		code_valid = IRRX_NECCode_Valid(code);

		if (code_valid) { /* report keycode */
			hirrx->rxCpltCallback(code & 0xff, (code >> 16) & 0xff);
		} /* else retry other protocal */
	}

	if (intsta & IRRX_FLAG_ROI) { /* FIFO Overflow */
		IRRX_INF("Rx FIFO Overflow!!\n");
		hirrx->rxCnt = 0;
	}
}

static void IRRX_SetConfig(IRRX_TypeDef *Instance, IRRX_InitTypeDef *param)
{
#if !defined (IR_CLK_32K_USED)
	uint32_t clk = HAL_GetHFClock();
#endif
	Instance->CR |= IRRX_CIR_EN; /* Enable IR Mode */

#if defined (IR_CLK_32K_USED)
	Instance->CCR = (IRRX_32K_RXFILT_VAL << IRRX_NTHR_INDEX) | \
	                (IRRX_32K_RXIDLE_VAL << IRRX_ITHR_INDEX) | \
	                (IRRX_32K_ACTIVE_T << IRRX_ATHR_INDEX) | \
	                (IRRX_32K_ACTIVE_T_C << IRRX_ATHC_INDEX) | \
	                IRRX_32K_SAMPLE_CLK | param->PulsePolariyInvert;
#else
	if (clk == HOSC_CLOCK_26M) {
		Instance->CCR = (IRRX_26M_RXFILT_VAL << IRRX_NTHR_INDEX) | \
		                (IRRX_26M_RXIDLE_VAL << IRRX_ITHR_INDEX) | \
		                (IRRX_26M_ACTIVE_T << IRRX_ATHR_INDEX) | \
		                (IRRX_26M_ACTIVE_T_C << IRRX_ATHC_INDEX) | \
		                IRRX_26M_SAMPLE_CLK | param->PulsePolariyInvert;
	} else if (clk == HOSC_CLOCK_24M) {
		Instance->CCR = (IRRX_24M_RXFILT_VAL << IRRX_NTHR_INDEX) | \
		                (IRRX_24M_RXIDLE_VAL << IRRX_ITHR_INDEX) | \
		                (IRRX_24M_ACTIVE_T << IRRX_ATHR_INDEX) | \
		                (IRRX_24M_ACTIVE_T_C << IRRX_ATHC_INDEX) | \
		                IRRX_24M_SAMPLE_CLK | param->PulsePolariyInvert;
	} else {
		IRRX_ERR("%s unknow clk type(%d)!\n", __func__, clk);
	}
#endif
	Instance->ICR &= ~IRRX_SET_RAL_MASK;
	Instance->ICR |= (IR_FIFO_TRIGER - 1) << IRRX_RAL_INDEX;

	Instance->CFR = param->PulsePolariyInvert;
	Instance->SR = IRRX_FLAG_MASK;

	Instance->ICR |= IRRX_IT_ROI | IRRX_IT_RPEI | IRRX_IT_RAI;

	Instance->CR |= IRRX_GLOBAL_EN | IRRX_RX_EN; /* enable ir module */
}

#ifdef CONFIG_PM
static IRRX_InitTypeDef hal_irrx_param;
static IRRX_StateTypeDef hal_irrx_suspending = 0;

static int irrx_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	hal_irrx_suspending = 1;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		HAL_IRRX_DeInit();
		IRRX_INF("%s okay\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}

static int irrx_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_IRRX_Init(&hal_irrx_param);
		IRRX_INF("%s okay\n", __func__);
		break;
	default:
		break;
	}

	hal_irrx_suspending = 0;

	return 0;
}

static struct soc_device_driver irrx_drv = {
	.name = "irrx",
	.suspend_noirq = irrx_suspend,
	.resume_noirq = irrx_resume,
};

static struct soc_device irrx_dev = {
	.name = "irrx",
	.driver = &irrx_drv,
};

#define IRRX_DEV (&irrx_dev)
#else
#define IRRX_DEV NULL
#endif

/**
 * Initializes the IRRX mode according to the specified parameters in the param.
 *
 * @param: Pointer to the configuration information for the specified IRRX module.
 */
void HAL_IRRX_Init(IRRX_InitTypeDef *param)
{
#if !defined (IR_CLK_32K_USED)
	uint32_t clk = HAL_GetHFClock();
#endif
	IRRX_HandleTypeDef *hirrx = &hal_irrx;

	HAL_ASSERT_PARAM(param->boardCfg);
	HAL_ASSERT_PARAM(param->rxCpltCallback);

	hirrx->Instance = (IRRX_TypeDef *)IRRX_BASE;
	hirrx->rxCpltCallback = param->rxCpltCallback;

	if (hirrx->State == IRRX_STATE_RESET) {
		hirrx->boardCfg = param->boardCfg;
		hirrx->boardCfg(0, HAL_BR_PINMUX_INIT, NULL);
	}

#if defined (IR_CLK_32K_USED)
	HAL_CCM_IRRX_SetMClock(IRRX_32K_APB_PERIPH_CLK_SRC,
	                       IRRX_32K_CCM_PERIPH_CLK_DIV_N, \
			       IRRX_32K_CCM_PERIPH_CLK_DIV_M);
#else
	if (clk == HOSC_CLOCK_26M) {
		HAL_CCM_IRRX_SetMClock(IRRX_26M_APB_PERIPH_CLK_SRC,
		                       IRRX_26M_CCM_PERIPH_CLK_DIV_N, \
		                       IRRX_26M_CCM_PERIPH_CLK_DIV_M);
	} else if (clk == HOSC_CLOCK_24M) {
		HAL_CCM_IRRX_SetMClock(IRRX_24M_APB_PERIPH_CLK_SRC,
		                       IRRX_24M_CCM_PERIPH_CLK_DIV_N, \
		                       IRRX_24M_CCM_PERIPH_CLK_DIV_M);
	} else {
		IRRX_ERR("%s unknow clk type(%d)!\n", __func__, clk);
	}
#endif
	HAL_CCM_IRRX_EnableMClock();
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_IRRX);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_IRRX);
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_IRRX);

	IRRX_SetConfig(hirrx->Instance, param);
	hirrx->Instance->SR = IRRX_FLAG_MASK; /* Clear All Rx Interrupt Status */

	hirrx->State = IRRX_STATE_READY;
	hirrx->rxCnt = 0;
#ifdef CONFIG_PM
	if (!hal_irrx_suspending) {
		memcpy(&hal_irrx_param, param, sizeof(IRRX_InitTypeDef));
		pm_register_ops(IRRX_DEV);
	}
#endif
	HAL_NVIC_SetIRQHandler(IRRX_IRQn, IRRX_IRQHandler);
	NVIC_EnableIRQ(IRRX_IRQn);
}

/**
 * DeInitializes the IRRX peripheral.
 */
void HAL_IRRX_DeInit(void)
{
	IRRX_HandleTypeDef *hirrx = &hal_irrx;

	if (hirrx->State != IRRX_STATE_READY)
		return ;

	NVIC_DisableIRQ(IRRX_IRQn);

#ifdef CONFIG_PM
	if (!hal_irrx_suspending) {
		pm_unregister_ops(IRRX_DEV);
	}
#endif

	/* Disable the Peripheral */
	hirrx->Instance->CR &= ~IRRX_GLOBAL_EN;

	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_IRRX);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_IRRX);
	HAL_CCM_IRRX_DisableMClock();

	hirrx->boardCfg(0, HAL_BR_PINMUX_DEINIT, NULL);

	hirrx->State = IRRX_STATE_RESET;
}

#undef IRRX_INF
#undef IRRX_WRN
#undef IRRX_ERR
