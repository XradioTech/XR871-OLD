/**
  * @file  hal_uart.c
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

#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_uart.h"
#include "pm/pm.h"

#include "hal_base.h"

#define UART_TRANSMIT_BY_IRQ_HANDLER	1

/* UART DMA mode configuration, use UART_DMA_MODE_0 by default */
#define UART_CFG_DMA_MODE           UART_DMA_MODE_0
#define UART_CFG_DMA_PTE_RX         0 // UART_DMA_PTE_RX_BIT
#define UART_CFG_DMA_PTE_TX         UART_DMA_PTE_TX_BIT
#define UART_CFG_DMA_RX_WAIT_CYCLE  DMA_WAIT_CYCLE_2
#define UART_CFG_DMA_TX_WAIT_CYCLE  (UART_CFG_DMA_MODE == UART_DMA_MODE_0 ? \
                                     DMA_WAIT_CYCLE_2 : DMA_WAIT_CYCLE_32)

/* default FIFO trigger level (the FIFO size is 64-byte) */
#define UART_TX_FIFO_TRIG_LEVEL_IT  UART_TX_FIFO_TRIG_LEVEL_TWO_CHAR
#define UART_TX_FIFO_TRIG_LEVEL_DMA UART_TX_FIFO_TRIG_LEVEL_TWO_CHAR
#define UART_RX_FIFO_TRIG_LEVEL_IT  UART_RX_FIFO_TRIG_LEVEL_HALF_FULL
#define UART_RX_FIFO_TRIG_LEVEL_DMA UART_RX_FIFO_TRIG_LEVEL_ONE_CHAR

#if HAL_UART_OPT_DMA
typedef struct {
	DMA_ChannelInitParam	param;
	DMA_Channel             chan;
	HAL_Semaphore           sem;
} UART_DMAPrivate;
#endif

#if HAL_UART_OPT_IT
typedef struct {
	uint8_t                *buf;
	int32_t                 bufSize;
	HAL_Semaphore           sem;
} UART_ITPrivate;
#endif

typedef struct {
	UART_T                 *uart;
#if HAL_UART_OPT_DMA
	UART_DMAPrivate        *txDMA;
	UART_DMAPrivate        *rxDMA;
#endif
#if HAL_UART_OPT_IT
	UART_ITPrivate         *txIT;
	UART_ITPrivate         *rxIT;

	UART_RxReadyCallback    rxReadyCallback;
	void                   *arg;
#endif
	/* UARTx->IIR_FCR.FIFO_CTRL is write only, shadow its value */
	uint8_t                IIR_FCR_FIFO_CTRL;
#ifdef CONFIG_PM
	uint8_t                 bypassPmMode;
	uint8_t                 txDelay;
	UART_InitParam          param;
	struct soc_device       dev;
#endif
} UART_Private;

static UART_Private *gUartPrivate[UART_NUM];

static UART_T * const gUartInstance[UART_NUM] = {
	UART0,
	UART1,
};

__STATIC_INLINE CCM_BusPeriphBit UART_GetCCMPeriphBit(UART_ID uartID)
{
	switch (uartID) {
	case UART0_ID:
		return CCM_BUS_PERIPH_BIT_UART0;
	case UART1_ID:
	default:
		return CCM_BUS_PERIPH_BIT_UART1;
	}
}

__STATIC_INLINE DMA_Periph UART_GetDMAPeriph(UART_ID uartID)
{
	switch (uartID) {
	case UART0_ID:
		return DMA_PERIPH_UART0;
	case UART1_ID:
	default:
		return DMA_PERIPH_UART1;
	}
}

__STATIC_INLINE UART_Private *UART_GetUartPriv(UART_ID uartID)
{
	if (uartID < UART_NUM) {
		return gUartPrivate[uartID];
	} else {
		return NULL;
	}
}

__STATIC_INLINE void UART_SetUartPriv(UART_ID uartID, UART_Private *priv)
{
	gUartPrivate[uartID] = priv;
}

__STATIC_INLINE void UART_EnableIRQ(UART_T *uart, uint32_t mask)
{
	HAL_SET_BIT(uart->DLH_IER.IRQ_EN, mask);
}

__STATIC_INLINE void UART_DisableIRQ(UART_T *uart, uint32_t mask)
{
	HAL_CLR_BIT(uart->DLH_IER.IRQ_EN, mask);
}

__STATIC_INLINE void UART_EnableTxReadyIRQ(UART_T *uart)
{
	UART_EnableIRQ(uart, UART_TX_READY_IRQ_EN_BIT);
}

__STATIC_INLINE void UART_DisableTxReadyIRQ(UART_T *uart)
{
	UART_DisableIRQ(uart, UART_TX_READY_IRQ_EN_BIT);
}

__STATIC_INLINE void UART_EnableRxReadyIRQ(UART_T *uart)
{
	UART_EnableIRQ(uart, UART_RX_READY_IRQ_EN_BIT);
}

__STATIC_INLINE void UART_DisableRxReadyIRQ(UART_T *uart)
{
	UART_DisableIRQ(uart, UART_RX_READY_IRQ_EN_BIT);
}

__STATIC_INLINE void UART_DisableAllIRQ(UART_T *uart)
{
	uart->DLH_IER.IRQ_EN = 0U;
}

__STATIC_INLINE void UART_EnableFIFO(UART_T *uart, UART_Private *priv, int enable)
{
	if (enable) {
		HAL_SET_BIT(priv->IIR_FCR_FIFO_CTRL, UART_FIFO_EN_BIT);
	} else  {
		HAL_CLR_BIT(priv->IIR_FCR_FIFO_CTRL, UART_FIFO_EN_BIT);
	}
	uart->IIR_FCR.FIFO_CTRL = priv->IIR_FCR_FIFO_CTRL;
}

__STATIC_INLINE void UART_ResetFIFO(UART_T *uart, UART_Private *priv, uint32_t mask)
{
	uint32_t reg = priv->IIR_FCR_FIFO_CTRL;
	HAL_SET_BIT(reg, mask);
	uart->IIR_FCR.FIFO_CTRL = reg;
}

__STATIC_INLINE void UART_SetRxFifoTrigLevel(UART_T *uart, UART_Private *priv,
                                             UART_RxFifoTrigLevel level)
{
	HAL_MODIFY_REG(priv->IIR_FCR_FIFO_CTRL, UART_RX_FIFO_TRIG_LEVEL_MASK, level);
	uart->IIR_FCR.FIFO_CTRL = priv->IIR_FCR_FIFO_CTRL;
}

__STATIC_INLINE void UART_SetTxFifoTrigLevel(UART_T *uart, UART_Private *priv,
                                             UART_TxFifoTrigLevel level)
{
	HAL_MODIFY_REG(priv->IIR_FCR_FIFO_CTRL, UART_TX_FIFO_TRIG_LEVEL_MASK, level);
	uart->IIR_FCR.FIFO_CTRL = priv->IIR_FCR_FIFO_CTRL;
}

/* baudRate = apb_freq / (16 * div) */
__STATIC_INLINE uint32_t UART_CalcClkDiv(uint32_t baudRate)
{
	uint32_t div;

	div = baudRate << 4;
	div = (HAL_GetAPBClock() + (div >> 1)) / div;
	if (div == 0)
		div = 1;
	return div;
}

__STATIC_INLINE int UART_IsBusy(UART_T *uart)
{
	return HAL_GET_BIT(uart->STATUS, UART_BUSY_BIT);
}

__STATIC_INLINE void UART_SetClkDiv(UART_T *uart, uint16_t div)
{
	HAL_SET_BIT(uart->LINE_CTRL, UART_DIV_ACCESS_BIT);

	uart->RBR_THR_DLL.DIV_LOW = div & UART_DIV_LOW_MASK;
	uart->DLH_IER.DIV_HIGH = (div >> 8) & UART_DIV_HIGH_MASK;

	HAL_CLR_BIT(uart->LINE_CTRL, UART_DIV_ACCESS_BIT);
}

__STATIC_INLINE void UART_EnableTx(UART_T *uart)
{
	HAL_CLR_BIT(uart->HALT, UART_HALT_TX_EN_BIT);
}

__STATIC_INLINE void UART_DisableTx(UART_T *uart)
{
	HAL_SET_BIT(uart->HALT, UART_HALT_TX_EN_BIT);
}

__STATIC_INLINE void UART_ConfigStart(UART_T *uart)
{
	HAL_SET_BIT(uart->HALT, UART_CHANGE_AT_BUSY_BIT);
}

__STATIC_INLINE void UART_ConfigFinish(UART_T *uart)
{
	HAL_SET_BIT(uart->HALT, UART_CHANGE_UPDATE_BIT);
	while (HAL_GET_BIT(uart->HALT, UART_CHANGE_UPDATE_BIT)) {
		;
	}
	HAL_CLR_BIT(uart->HALT, UART_CHANGE_AT_BUSY_BIT);
}

__STATIC_INLINE void UART_SetTxDelay(UART_T *uart, uint8_t txDelay)
{
	uart->TX_DELAY = txDelay;
}

__STATIC_INLINE UART_T *UART_GetInstance(UART_Private *priv)
{
	return priv->uart;
}

/**
 * @brief Get UART hardware instance by the specified UART ID
 * @param[in] uartID ID of the specified UART
 * @return UART hardware instance
 */
UART_T *HAL_UART_GetInstance(UART_ID uartID)
{
	return gUartInstance[uartID];
}

/**
 * @brief Check whether the specified UART can transmit data or not
 * @param[in] uart UART hardware instance
 * @return 1 The UART can transmit data
 * @return 0 The UART cannot transmit data
 */
int HAL_UART_IsTxReady(UART_T *uart)
{
//	return HAL_GET_BIT(uart->LINE_STATUS, UART_TX_HOLD_EMPTY_BIT);
	return HAL_GET_BIT(uart->STATUS, UART_TX_FIFO_NOT_FULL_BIT);
}

/**
 * @brief Check whether the transmit FIFO of the specified UART is empty or not
 * @param[in] uart UART hardware instance
 * @return 1 The UART transmit FIFO is empty
 * @return 0 The UART transmit FIFO is not empty
 */
int HAL_UART_IsTxEmpty(UART_T *uart)
{
	return HAL_GET_BIT(uart->STATUS, UART_TX_FIFO_EMPTY_BIT);
}

/**
 * @brief Check whether the specified UART has receive data or not
 * @param[in] uart UART hardware instance
 * @return 1 The UART has receive data
 * @return 0 The UART has no receive data
 */
int HAL_UART_IsRxReady(UART_T *uart)
{
	return HAL_GET_BIT(uart->LINE_STATUS, UART_RX_READY_BIT);
//	return HAL_GET_BIT(uart->STATUS, UART_RX_FIFO_NOT_EMPTY_BIT);
}

/**
 * @brief Get one byte received data of the specified UART
 * @param[in] uart UART hardware instance
 * @return One byte received data
 *
 * @note Before calling this function, make sure the specified UART has
 *       receive data by calling HAL_UART_IsRxReady()
 */
uint8_t HAL_UART_GetRxData(UART_T *uart)
{
	return (uint8_t)(uart->RBR_THR_DLL.RX_BUF);
}

/**
 * @brief Transmit one byte data for the specified UART
 * @param[in] uart UART hardware instance
 * @param[in] data one byte data
 * @return None
 *
 * @note Before calling this function, make sure the specified UART can
 *       transmit data by calling HAL_UART_IsTxReady()
 */
void HAL_UART_PutTxData(UART_T *uart, uint8_t data)
{
	uart->RBR_THR_DLL.TX_HOLD = data;
}

__STATIC_INLINE uint32_t UART_GetInterruptID(UART_T *uart)
{
	return HAL_GET_BIT(uart->IIR_FCR.IRQ_ID, UART_IID_MASK);
}

__STATIC_INLINE uint32_t UART_GetLineStatus(UART_T *uart)
{
	return HAL_GET_BIT(uart->LINE_STATUS, UART_LINE_STATUS_MASK);
}

__STATIC_INLINE uint32_t UART_GetModemStatus(UART_T *uart)
{
	return HAL_GET_BIT(uart->MODEM_STATUS, UART_MODEM_STATUS_MASK);
}

__STATIC_INLINE uint32_t UART_GetUartStatus(UART_T *uart)
{
	return HAL_GET_BIT(uart->STATUS, UART_STATUS_MASK);
}

#if HAL_UART_OPT_IT

static void UART_IRQHandler(UART_T *uart, UART_Private *priv)
{
	uint32_t iid = UART_GetInterruptID(uart);

	switch (iid) {
	case UART_IID_MODEM_STATUS:
		UART_GetModemStatus(uart);
		break;
	case UART_IID_TX_READY:
#if UART_TRANSMIT_BY_IRQ_HANDLER
		if (priv && priv->txIT && priv->txIT->buf) {
			while (priv->txIT->bufSize > 0) {
				if (HAL_UART_IsTxReady(uart)) {
					HAL_UART_PutTxData(uart, *priv->txIT->buf);
					++priv->txIT->buf;
					--priv->txIT->bufSize;
				} else {
					break;
				}
			}
			if (priv->txIT->bufSize == 0) {
				UART_DisableTxReadyIRQ(uart);
				HAL_SemaphoreRelease(&priv->txIT->sem); /* end transmitting */
			}
		} else {
			UART_DisableTxReadyIRQ(uart);
		}
#else /* UART_TRANSMIT_BY_IRQ_HANDLER */
		UART_DisableTxReadyIRQ(uart);
		HAL_SemaphoreRelease(&priv->txIT->sem);
#endif /* UART_TRANSMIT_BY_IRQ_HANDLER */
		break;
	case UART_IID_RX_READY:
	case UART_IID_CHAR_TIMEOUT:
		if (priv && priv->rxReadyCallback) {
			priv->rxReadyCallback(priv->arg);
		} else if (priv && priv->rxIT && priv->rxIT->buf) {
			while (priv->rxIT->bufSize > 0) {
				if (HAL_UART_IsRxReady(uart)) {
					*priv->rxIT->buf = HAL_UART_GetRxData(uart);
					++priv->rxIT->buf;
					--priv->rxIT->bufSize;
				} else {
					break;
				}
			}
			if (priv->rxIT->bufSize == 0 || iid == UART_IID_CHAR_TIMEOUT) {
				UART_DisableRxReadyIRQ(uart);
				HAL_SemaphoreRelease(&priv->rxIT->sem); /* end receiving */
			}
		} else {
			HAL_WRN("no one recv data, but uart irq is enable\n");
			/* discard received data */
			while (HAL_UART_IsRxReady(uart)) {
				HAL_UART_GetRxData(uart);
			}
		}
		break;
	case UART_IID_LINE_STATUS:
		UART_GetLineStatus(uart);
		break;
	case UART_IID_BUSY_DETECT:
		UART_GetUartStatus(uart);
		break;
	case UART_IID_RS485:
	default:
		break;
	}
}

void UART0_IRQHandler(void)
{
	UART_IRQHandler(UART0, UART_GetUartPriv(UART0_ID));
}

void UART1_IRQHandler(void)
{
	UART_IRQHandler(UART1, UART_GetUartPriv(UART1_ID));
}

void N_UART_IRQHandler(void)
{
	UART_IRQHandler(N_UART, NULL);
}

static UART_ITPrivate *UART_CreateITPrivate(void)
{
	UART_ITPrivate *itPriv;

	itPriv = HAL_Malloc(sizeof(UART_ITPrivate));
	if (itPriv == NULL) {
		return itPriv;
	}
	HAL_Memset(itPriv, 0, sizeof(UART_ITPrivate));
	if (HAL_SemaphoreInitBinary(&itPriv->sem) != HAL_OK) {
		HAL_Free(itPriv);
		itPriv = NULL;
	}
	return itPriv;
}

static void UART_DestroyITPrivate(UART_ITPrivate *itPriv)
{
	HAL_SemaphoreDeinit(&itPriv->sem);
	HAL_Free(itPriv);
}

#endif /* HAL_UART_OPT_IT */

#if HAL_UART_OPT_DMA

static DMA_Channel UART_HwInitDMA(DMA_Channel chan, const DMA_ChannelInitParam *param)
{
#ifdef CONFIG_PM
	if (chan != DMA_CHANNEL_INVALID) {
		chan = HAL_DMA_RequestSpecified(chan);
	} else
#endif
	{
		chan = HAL_DMA_Request();
	}

	if (chan == DMA_CHANNEL_INVALID) {
		return chan;
	}

	HAL_DMA_Init(chan, param);

	return chan;
}

static void UART_HwDeInitDMA(DMA_Channel chan)
{
	if (chan != DMA_CHANNEL_INVALID) {
		HAL_DMA_Stop(chan);
		HAL_DMA_DeInit(chan);
		HAL_DMA_Release(chan);
	}
}

#endif /* HAL_UART_OPT_DMA */

static UART_T *UART_HwInit(UART_ID uartID, const UART_InitParam *param, UART_Private *priv)
{
	UART_T *uart;
	CCM_BusPeriphBit ccmPeriphBit;
	uint32_t tmp;
#if HAL_UART_OPT_IT
	IRQn_Type IRQn;
	NVIC_IRQHandler IRQHandler;
#endif

	switch (uartID) {
	case UART0_ID:
		uart = UART0;
#if HAL_UART_OPT_IT
		IRQn = UART0_IRQn;
		IRQHandler = UART0_IRQHandler;
#endif
		break;
	case UART1_ID:
	default:
		uart = UART1;
#if HAL_UART_OPT_IT
		IRQn = UART1_IRQn;
		IRQHandler = UART1_IRQHandler;
#endif
		break;
	}

	/* config pinmux */
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_UART, uartID), 0);

	/* enable uart clock and release reset */
	ccmPeriphBit = UART_GetCCMPeriphBit(uartID);
	HAL_CCM_BusEnablePeriphClock(ccmPeriphBit);
	HAL_CCM_BusReleasePeriphReset(ccmPeriphBit);

	UART_DisableAllIRQ(uart); /* disable all IRQ */

	UART_DisableTx(uart);

	UART_ConfigStart(uart);

	/* set baud rate, parity, stop bits, data bits */
	tmp = UART_CalcClkDiv(param->baudRate);
	UART_SetClkDiv(uart, (uint16_t)tmp);

	HAL_MODIFY_REG(uart->LINE_CTRL,
		           UART_PARITY_MASK | UART_STOP_BITS_MASK |
		           UART_DATA_BITS_MASK | UART_BREAK_CTRL_BIT,
		           param->parity | param->stopBits | param->dataBits);

	UART_ConfigFinish(uart);

	/* set uart->MODEM_CTRL */
	if (uartID != UART0_ID && param->isAutoHwFlowCtrl) {
		tmp = UART_WORK_MODE_UART | UART_AUTO_FLOW_CTRL_EN_BIT | UART_RTS_ASSERT_BIT;
	} else {
		tmp = UART_WORK_MODE_UART;
	}
	HAL_MODIFY_REG(uart->MODEM_CTRL,
		           UART_WORK_MODE_MASK | UART_LOOP_BACK_EN_BIT |
		           UART_AUTO_FLOW_CTRL_EN_BIT | UART_RTS_ASSERT_BIT,
		           tmp);

#if HAL_UART_OPT_IT
	/* set TX interrupt tirgger mode to TX FIFO trigger mode */
	UART_EnableIRQ(uart, UART_TX_FIFO_TRIG_MODE_EN_BIT);
#endif

	/* set FIFO level, DMA mode, reset FIFO, enable FIFO */
	priv->IIR_FCR_FIFO_CTRL = UART_RX_FIFO_TRIG_LEVEL_IT |
	                          UART_TX_FIFO_TRIG_LEVEL_IT |
	                          UART_CFG_DMA_MODE |
	                          UART_FIFO_EN_BIT;
	uart->IIR_FCR.FIFO_CTRL = priv->IIR_FCR_FIFO_CTRL |
	                          UART_TX_FIFO_RESET_BIT |
	                          UART_RX_FIFO_RESET_BIT;

	/* set DMA PTE TX/RX mode, enable TX */
	uart->HALT = UART_CFG_DMA_PTE_RX | UART_CFG_DMA_PTE_TX;

#if HAL_UART_OPT_IT
	HAL_NVIC_ConfigExtIRQ(IRQn, IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);
#endif

	return uart;
}

static void UART_HwDeInit(UART_ID uartID, UART_Private *priv)
{
	UART_T *uart;
	CCM_BusPeriphBit ccmPeriphBit;
#if HAL_UART_OPT_IT
	IRQn_Type IRQn;
#endif

#if HAL_UART_OPT_IT
	switch (uartID) {
	case UART0_ID:
		IRQn = UART0_IRQn;
		break;
	case UART1_ID:
	default:
		IRQn = UART1_IRQn;
		break;
	}

	HAL_NVIC_DisableIRQ(IRQn);
#endif
	uart = UART_GetInstance(priv);
	UART_DisableAllIRQ(uart);
	UART_DisableTx(uart);
	uart->IIR_FCR.FIFO_CTRL = 0;

	/* disable uart clock and force reset */
	ccmPeriphBit = UART_GetCCMPeriphBit(uartID);
	HAL_CCM_BusForcePeriphReset(ccmPeriphBit);
	HAL_CCM_BusDisablePeriphClock(ccmPeriphBit);

	/* De-config pinmux */
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_UART, uartID), 0);
}

#ifdef CONFIG_PM

static int uart_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	UART_ID uartID = (UART_ID)dev->platform_data;
	UART_Private *priv = UART_GetUartPriv(uartID);

	switch (state) {
	case PM_MODE_SLEEP:
		if (priv->bypassPmMode & PM_SUPPORT_SLEEP)
			break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
#if HAL_UART_OPT_DMA
		if (priv->txDMA) {
			UART_HwDeInitDMA(priv->txDMA->chan);
		}
		if (priv->rxDMA) {
			UART_HwDeInitDMA(priv->rxDMA->chan);
		}
#endif
#if 0 /* wait tx done is not driver's job */
		while (!HAL_UART_IsTxEmpty(UART_GetInstance(priv))) { }
		HAL_UDelay(100); /* wait tx done */
#endif
		UART_HwDeInit(uartID, priv);
		HAL_DBG("%s ok, id %d\n", __func__, uartID);
		break;
	default:
		break;
	}
	return 0;
}

static int uart_resume(struct soc_device *dev, enum suspend_state_t state)
{
	UART_ID uartID = (UART_ID)dev->platform_data;
	UART_Private *priv = UART_GetUartPriv(uartID);
	UART_T *uart;

	switch (state) {
	case PM_MODE_SLEEP:
		if (priv->bypassPmMode & PM_SUPPORT_SLEEP)
			break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		UART_HwInit(uartID, &priv->param, priv);
		uart = UART_GetInstance(priv);
		UART_SetTxDelay(uart, priv->txDelay);
#if HAL_UART_OPT_DMA
		if (priv->txDMA) {
			priv->txDMA->chan = UART_HwInitDMA(priv->txDMA->chan, &priv->txDMA->param);
			/* set UART TX FIFO trigger level for DMA mode */
			if (UART_TX_FIFO_TRIG_LEVEL_DMA != UART_TX_FIFO_TRIG_LEVEL_IT) {
				UART_SetTxFifoTrigLevel(uart, priv, UART_TX_FIFO_TRIG_LEVEL_DMA);
			}
		}
		if (priv->rxDMA) {
			priv->rxDMA->chan = UART_HwInitDMA(priv->rxDMA->chan, &priv->rxDMA->param);
			/* set UART RX FIFO trigger level for DMA mode */
			if (UART_RX_FIFO_TRIG_LEVEL_DMA != UART_RX_FIFO_TRIG_LEVEL_IT) {
				UART_SetRxFifoTrigLevel(uart, priv, UART_RX_FIFO_TRIG_LEVEL_DMA);
			}
		}
#endif
#if HAL_UART_OPT_IT
		if (priv->rxReadyCallback) {
			UART_EnableRxReadyIRQ(uart);
		}
#endif
		HAL_DBG("%s ok, id %d\n", __func__, uartID);
		break;
	default:
		break;
	}

	return 0;
}

static const struct soc_device_driver uart_drv = {
	.name = "uart",
	.suspend_noirq = uart_suspend,
	.resume_noirq = uart_resume,
};

static const char gUartName[UART_NUM][6] = {
	{ "uart0" },
	{ "uart1" },
};

#endif /* CONFIG_PM */

/**
 * @brief Initialize the UART according to the specified parameters
 * @param[in] uartID ID of the specified UART
 * @param[in] param Pointer to UART_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_Init(UART_ID uartID, const UART_InitParam *param)
{
	UART_Private *priv;

	if (uartID > UART_NUM) {
		HAL_DBG("invalid uart id %d\n", uartID);
		return HAL_ERROR;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv != NULL) {
		HAL_DBG("uart %d is inited\n", uartID);
		return HAL_BUSY;
	}

	priv = HAL_Malloc(sizeof(UART_Private));
	if (priv == NULL) {
		HAL_ERR("no mem\n");
		return HAL_ERROR;
	}
	HAL_Memset(priv, 0, sizeof(UART_Private));
	UART_SetUartPriv(uartID, priv);

	priv->uart = UART_HwInit(uartID, param, priv);

#ifdef CONFIG_PM
	HAL_Memcpy(&priv->param, param, sizeof(UART_InitParam));
	priv->dev.name = gUartName[uartID];
	priv->dev.driver = &uart_drv;
	priv->dev.platform_data = (void *)uartID;
	pm_register_ops(&priv->dev);
#endif

	return HAL_OK;
}

/**
 * @brief DeInitialize the specified UART
 * @param[in] uartID ID of the specified UART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_DeInit(UART_ID uartID)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_OK;
	}

#if HAL_UART_OPT_IT
	if (priv->rxReadyCallback != NULL) {
		HAL_WRN("rx cb enabled\n");
	}
#endif

#ifdef CONFIG_PM
	pm_unregister_ops(&priv->dev);
#endif

#if HAL_UART_OPT_DMA
	HAL_UART_DeInitTxDMA(uartID);
	HAL_UART_DeInitRxDMA(uartID);
#endif

	UART_HwDeInit(uartID, priv);

#if HAL_UART_OPT_IT
	if (priv->txIT) {
		UART_DestroyITPrivate(priv->txIT);
	}
	if (priv->rxIT) {
		UART_DestroyITPrivate(priv->rxIT);
	}
#endif

	UART_SetUartPriv(uartID, NULL);
	HAL_Free(priv);

	return HAL_OK;
}

#if HAL_UART_OPT_IT

/**
 * @brief Transmit an amount of data in interrupt mode
 * @param[in] uartID ID of the specified UART
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be transmitted
 * @return Number of bytes transmitted, -1 on error
 *
 * @note This function is not thread safe. If using the UART transmit series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
int32_t HAL_UART_Transmit_IT(UART_ID uartID, const uint8_t *buf, int32_t size)
{
	UART_T *uart;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}

	if (priv->txIT == NULL) {
		priv->txIT = UART_CreateITPrivate();
		if (priv->txIT == NULL) {
			return -1;
		}
	}

	uart = UART_GetInstance(priv);

#if UART_TRANSMIT_BY_IRQ_HANDLER
	priv->txIT->buf = (uint8_t *)buf;
	priv->txIT->bufSize = size;

	UART_EnableTxReadyIRQ(uart);
	HAL_SemaphoreWait(&priv->txIT->sem, HAL_WAIT_FOREVER);
	UART_DisableTxReadyIRQ(uart);
 	priv->txIT->buf = NULL;
	size -= priv->txIT->bufSize;
	priv->txIT->bufSize = 0;
#else /* UART_TRANSMIT_BY_IRQ_HANDLER */
	uint8_t *ptr = buf;
	int32_t left = size;
	while (left > 0) {
		if (HAL_UART_IsTxReady(uart)) {
			HAL_UART_PutTxData(uart, *ptr);
			++ptr;
			--left;
		} else {
			UART_EnableTxReadyIRQ(uart);
			HAL_SemaphoreWait(&priv->txIT->sem, HAL_WAIT_FOREVER);
		}
	}
	UART_DisableTxReadyIRQ(uart); /* just in case */
#endif /* UART_TRANSMIT_BY_IRQ_HANDLER */

	return size;
}

/**
 * @brief Receive an amount of data in interrupt mode
 * @param[in] uartID ID of the specified UART
 * @param[out] buf Pointer to the data buffer
 * @param[in] size The maximum number of bytes to be received.
 *                 The actual received bytes can be less than this.
 * @param[in] msec Timeout value in millisecond to receive data.
 *                 HAL_WAIT_FOREVER for no timeout.
 * @return Number of bytes received, -1 on error
 *
 * @note This function is not thread safe. If using the UART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
int32_t HAL_UART_Receive_IT(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec)
{
	UART_T *uart;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}

	if (priv->rxReadyCallback != NULL) {
		HAL_WRN("rx cb is enabled\n");
		return -1;
	}

	if (priv->rxIT == NULL) {
		priv->rxIT = UART_CreateITPrivate();
		if (priv->rxIT == NULL) {
			return -1;
		}
	}

	priv->rxIT->buf = buf;
	priv->rxIT->bufSize = size;

	uart = UART_GetInstance(priv);
	UART_EnableRxReadyIRQ(uart);
	HAL_SemaphoreWait(&priv->rxIT->sem, msec);
	UART_DisableRxReadyIRQ(uart);

	priv->rxIT->buf = NULL;
	size -= priv->rxIT->bufSize;
	priv->rxIT->bufSize = 0;

	return size;
}

/**
 * @brief Enable receive ready callback function for the specified UART
 * @param[in] uartID ID of the specified UART
 * @param[in] cb The UART receive ready callback function
 * @param[in] arg Argument of the UART receive ready callback function
 * @retval HAL_Status, HAL_OK on success
 *
 * @note To handle receive data externally, use this function to enable the
 *       receive ready callback function, then receive and process the data in
 *       the callback function.
 * @note If the receive ready callback function is enabled, all other receive
 *       series functions cannot be used to receive data.
 * @note This function is not thread safe. If using the UART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
HAL_Status HAL_UART_EnableRxCallback(UART_ID uartID, UART_RxReadyCallback cb, void *arg)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	priv->rxReadyCallback = cb;
	priv->arg = arg;
	UART_EnableRxReadyIRQ(UART_GetInstance(priv));

	return HAL_OK;
}

/**
 * @brief Disable receive ready callback function for the specified UART
 * @param[in] uartID ID of the specified UART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_DisableRxCallback(UART_ID uartID)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	UART_DisableRxReadyIRQ(UART_GetInstance(priv));
	priv->rxReadyCallback = NULL;
	priv->arg = NULL;

	return HAL_OK;
}

#endif /* HAL_UART_OPT_IT */

#if HAL_UART_OPT_DMA

HAL_Status HAL_UART_InitTxDMA(UART_ID uartID, const DMA_ChannelInitParam *param)
{
	UART_Private *priv;
	UART_DMAPrivate *txDMA;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	if (priv->txDMA != NULL) {
		HAL_DBG("uart %d tx dma is inited\n", uartID);
		return HAL_ERROR;
	}

	txDMA = HAL_Malloc(sizeof(UART_DMAPrivate));
	if (txDMA == NULL) {
		HAL_ERR("no mem\n");
		return HAL_ERROR;
	}

	HAL_Memset(txDMA, 0, sizeof(UART_DMAPrivate));
	HAL_SemaphoreSetInvalid(&txDMA->sem);

	HAL_Memcpy(&txDMA->param, param, sizeof(DMA_ChannelInitParam));
	if (txDMA->param.cfg == 0) {
		txDMA->param.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		                                              UART_CFG_DMA_TX_WAIT_CYCLE,
		                                              DMA_BYTE_CNT_MODE_REMAIN,
		                                              DMA_DATA_WIDTH_8BIT,
		                                              DMA_BURST_LEN_1,
		                                              DMA_ADDR_MODE_FIXED,
		                                              UART_GetDMAPeriph(uartID),
		                                              DMA_DATA_WIDTH_8BIT,
		                                              DMA_BURST_LEN_1,
		                                              DMA_ADDR_MODE_INC,
		                                              DMA_PERIPH_SRAM);
	}

	txDMA->chan = UART_HwInitDMA(DMA_CHANNEL_INVALID, &txDMA->param);
	if (txDMA->chan == DMA_CHANNEL_INVALID) {
		HAL_Free(txDMA);
		return HAL_ERROR;
	}
	priv->txDMA = txDMA;

	/* set UART TX FIFO trigger level for DMA mode */
	if (UART_TX_FIFO_TRIG_LEVEL_DMA != UART_TX_FIFO_TRIG_LEVEL_IT) {
		UART_SetTxFifoTrigLevel(UART_GetInstance(priv), priv,
		                        UART_TX_FIFO_TRIG_LEVEL_DMA);
	}

	return HAL_OK;
}

HAL_Status HAL_UART_InitRxDMA(UART_ID uartID, const DMA_ChannelInitParam *param)
{
	UART_Private *priv;
	UART_DMAPrivate *rxDMA;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	if (priv->rxDMA != NULL) {
		HAL_DBG("uart %d rx dma is inited\n", uartID);
		return HAL_ERROR;
	}

	rxDMA = HAL_Malloc(sizeof(UART_DMAPrivate));
	if (rxDMA == NULL) {
		HAL_ERR("no mem\n");
		return HAL_ERROR;
	}

	HAL_Memset(rxDMA, 0, sizeof(UART_DMAPrivate));
	HAL_SemaphoreSetInvalid(&rxDMA->sem);

	HAL_Memcpy(&rxDMA->param, param, sizeof(DMA_ChannelInitParam));
	if (rxDMA->param.cfg == 0) {
		rxDMA->param.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		                                              UART_CFG_DMA_RX_WAIT_CYCLE,
		                                              DMA_BYTE_CNT_MODE_REMAIN,
		                                              DMA_DATA_WIDTH_8BIT,
		                                              DMA_BURST_LEN_1,
		                                              DMA_ADDR_MODE_INC,
		                                              DMA_PERIPH_SRAM,
		                                              DMA_DATA_WIDTH_8BIT,
		                                              DMA_BURST_LEN_1,
		                                              DMA_ADDR_MODE_FIXED,
		                                              UART_GetDMAPeriph(uartID));
	}

	rxDMA->chan = UART_HwInitDMA(DMA_CHANNEL_INVALID, &rxDMA->param);
	if (rxDMA->chan == DMA_CHANNEL_INVALID) {
		HAL_Free(rxDMA);
		return HAL_ERROR;
	}
	priv->rxDMA = rxDMA;

	/* set UART RX FIFO trigger level for DMA mode */
	if (UART_RX_FIFO_TRIG_LEVEL_DMA != UART_RX_FIFO_TRIG_LEVEL_IT) {
		UART_SetRxFifoTrigLevel(UART_GetInstance(priv), priv,
		                        UART_RX_FIFO_TRIG_LEVEL_DMA);
	}

	return HAL_OK;
}

HAL_Status HAL_UART_DeInitTxDMA(UART_ID uartID)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	if (priv->txDMA) {
		UART_HwDeInitDMA(priv->txDMA->chan);

		/* reset UART TX FIFO trigger level */
		if (UART_TX_FIFO_TRIG_LEVEL_DMA != UART_TX_FIFO_TRIG_LEVEL_IT) {
			UART_SetTxFifoTrigLevel(UART_GetInstance(priv), priv,
			                        UART_TX_FIFO_TRIG_LEVEL_IT);
		}

		if (HAL_SemaphoreIsValid(&priv->txDMA->sem)) {
			HAL_SemaphoreDeinit(&priv->txDMA->sem);
		}

		HAL_Free(priv->txDMA);
		priv->txDMA = NULL;
	}

	return HAL_OK;
}

HAL_Status HAL_UART_DeInitRxDMA(UART_ID uartID)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	if (priv->rxDMA) {
		UART_HwDeInitDMA(priv->rxDMA->chan);

		/* set UART RX FIFO trigger level for DMA mode */
		if (UART_RX_FIFO_TRIG_LEVEL_DMA != UART_RX_FIFO_TRIG_LEVEL_IT) {
			UART_SetRxFifoTrigLevel(UART_GetInstance(priv), priv,
			                        UART_RX_FIFO_TRIG_LEVEL_IT);
		}

		if (HAL_SemaphoreIsValid(&priv->rxDMA->sem)) {
			HAL_SemaphoreDeinit(&priv->rxDMA->sem);
		}

		HAL_Free(priv->rxDMA);
		priv->rxDMA = NULL;
	}

	return HAL_OK;
}

HAL_Status HAL_UART_StartTransmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size)
{
	UART_T *uart;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return HAL_ERROR;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	if (priv->txDMA == NULL) {
		HAL_WRN("tx dma not enable\n");
		return HAL_ERROR;
	}

	uart = UART_GetInstance(priv);
	HAL_DMA_Start(priv->txDMA->chan,
	              (uint32_t)buf,
		          (uint32_t)&uart->RBR_THR_DLL.TX_HOLD,
		          size);
	return HAL_OK;
}

int32_t HAL_UART_StopTransmit_DMA(UART_ID uartID)
{
	UART_Private *priv;
	DMA_Channel chan;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}

	if (priv->txDMA == NULL) {
		HAL_WRN("tx dma not enable\n");
		return -1;
	}

	chan = priv->txDMA->chan;
	HAL_DMA_Stop(chan);

	return HAL_DMA_GetByteCount(chan);
}

HAL_Status HAL_UART_StartReceive_DMA(UART_ID uartID, uint8_t *buf, int32_t size)
{
	UART_T *uart;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return HAL_ERROR;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

#if HAL_UART_OPT_IT
	if (priv->rxReadyCallback != NULL) {
		HAL_WRN("rx cb is enabled\n");
		return HAL_ERROR;
	}
#endif

	if (priv->rxDMA == NULL) {
		HAL_WRN("rx dma not enable\n");
		return HAL_ERROR;
	}

	uart = UART_GetInstance(priv);
	HAL_DMA_Start(priv->rxDMA->chan,
	              (uint32_t)&uart->RBR_THR_DLL.RX_BUF,
	              (uint32_t)buf,
	              size);
	return HAL_OK;
}

int32_t HAL_UART_StopReceive_DMA(UART_ID uartID)
{
	UART_Private *priv;
	DMA_Channel chan;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}

	if (priv->rxDMA == NULL) {
		HAL_WRN("rx dma not enable\n");
		return -1;
	}

	chan = priv->rxDMA->chan;
	HAL_DMA_Stop(chan);

	return HAL_DMA_GetByteCount(chan);
}

/**
 * @internal
 * @brief UART DMA transfer complete callback fucntion to release waiting
 *        semaphore
 * @param[in] arg Pointer to waiting semaphore for DMA transfer
 * @return None
 */
static void UART_DMAEndCallback(void *arg)
{
	UART_DMAPrivate *dmaPriv = *((UART_DMAPrivate **)arg);
	HAL_SemaphoreRelease(&dmaPriv->sem);
}

/**
 * @brief Enable UART transmitting data in DMA mode
 * @param[in] uartID ID of the specified UART
 * @retval HAL_Status, HAL_OK on success, HAL_ERROR on no valid DMA channel
 *
 * @note To transmit data in DMA mode, a DMA channel for the specified
 *       UART to transmit data MUST be configured first by this function.
 */
HAL_Status HAL_UART_EnableTxDMA(UART_ID uartID)
{
	HAL_Status ret;
	UART_Private *priv;
	DMA_ChannelInitParam dmaParam;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	HAL_Memset(&dmaParam, 0, sizeof(dmaParam));
	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = UART_DMAEndCallback;
	dmaParam.endArg = &priv->txDMA;

	ret = HAL_UART_InitTxDMA(uartID, &dmaParam);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_SemaphoreInitBinary(&priv->txDMA->sem);
	if (ret != HAL_OK) {
		HAL_UART_DeInitTxDMA(uartID);
	}

	return ret;
}

/**
 * @brief Enable UART receiving data in DMA mode
 * @param[in] uartID ID of the specified UART
 * @retval HAL_Status, HAL_OK on success, HAL_ERROR on no valid DMA channel
 *
 * @note To reveive data in DMA mode, a DMA channel for the specified
 *       UART to receive data MUST be configured first by this function.
 */
HAL_Status HAL_UART_EnableRxDMA(UART_ID uartID)
{
	HAL_Status ret;
	UART_Private *priv;
	DMA_ChannelInitParam dmaParam;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	HAL_Memset(&dmaParam, 0, sizeof(dmaParam));
	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = UART_DMAEndCallback;
	dmaParam.endArg = &priv->rxDMA;

	ret = HAL_UART_InitRxDMA(uartID, &dmaParam);
	if (ret != HAL_OK) {
		return ret;
	}

	ret = HAL_SemaphoreInitBinary(&priv->rxDMA->sem);
	if (ret != HAL_OK) {
		HAL_UART_DeInitRxDMA(uartID);
	}

	return ret;
}

/**
 * @brief Disable UART transmitting data in DMA mode
 * @param[in] uartID ID of the specified UART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_DisableTxDMA(UART_ID uartID)
{
	return HAL_UART_DeInitTxDMA(uartID);
}

/**
 * @brief Disable UART receiving data in DMA mode
 * @param[in] uartID ID of the specified UART
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_DisableRxDMA(UART_ID uartID)
{
	return HAL_UART_DeInitRxDMA(uartID);
}

/**
 * @brief Transmit an amount of data in DMA mode
 *
 * Steps to transmit data in DMA mode:
 *     - use HAL_UART_EnableTxDMA() to enable UART transmit DMA mode
 *     - use HAL_UART_Transmit_DMA() to transmit data, it can be called
 *       repeatedly after HAL_UART_EnableTxDMA()
 *     - use HAL_UART_DisableTxDMA() to disable UART transmit DMA mode if needed
 *
 * @param[in] uartID ID of the specified UART
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be transmitted
 * @return Number of bytes transmitted, -1 on error
 *
 * @note This function is not thread safe. If using the UART transmit series
 *       functions in multi-thread, make sure they are executed exclusively.
 * @note To transmit data in DMA mode, HAL_UART_EnableTxDMA() MUST be executed
 *       before calling this function.
 */
int32_t HAL_UART_Transmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size)
{
	UART_Private *priv;
	int32_t left;

	if (HAL_UART_StartTransmit_DMA(uartID, buf, size) != HAL_OK) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	HAL_SemaphoreWait(&priv->txDMA->sem, HAL_WAIT_FOREVER);

	left = HAL_UART_StopTransmit_DMA(uartID);
	if (left < 0) {
		return -1;
	}

	return (size - left);
}

/**
 * @brief Receive an amount of data in DMA mode
 *
 * Steps to receive data in DMA mode:
 *     - use HAL_UART_EnableRxDMA() to enable UART receive DMA mode
 *     - use HAL_UART_Receive_DMA() to receive data, it can be called
 *       repeatedly after HAL_UART_EnableRxDMA()
 *     - use HAL_UART_DisableRxDMA() to disable UART receive DMA mode if needed
 *
 * @param[in] uartID ID of the specified UART
 * @param[out] buf Pointer to the data buffer
 * @param[in] size The maximum number of bytes to be received.
 *                 The actual received bytes can be less than this.
 * @param[in] msec Timeout value in millisecond to receive data.
 *                 HAL_WAIT_FOREVER for no timeout.
 * @return Number of bytes received, -1 on error
 *
 * @note This function is not thread safe. If using the UART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 * @note To receive data in DMA mode, HAL_UART_EnableRxDMA() MUST be executed
 *       before calling this function.
 */
int32_t HAL_UART_Receive_DMA(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec)
{
	UART_Private *priv;
	int32_t left;

	if (HAL_UART_StartReceive_DMA(uartID, buf, size) != HAL_OK) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	HAL_SemaphoreWait(&priv->rxDMA->sem, msec);

	left = HAL_UART_StopReceive_DMA(uartID);
	if (left < 0) {
		return -1;
	}

	return (size - left);
}

#endif /* HAL_UART_OPT_DMA */

/**
 * @brief Transmit an amount of data in polling mode
 * @param[in] uartID ID of the specified UART
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be transmitted
 * @return Number of bytes transmitted, -1 on error
 *
 * @note This function is not thread safe. If using the UART transmit series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
int32_t HAL_UART_Transmit_Poll(UART_ID uartID, const uint8_t *buf, int32_t size)
{
	UART_T *uart;
	UART_Private *priv;
	const uint8_t *ptr;
	int32_t left;

	if (buf == NULL || size <= 0) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}
	uart = UART_GetInstance(priv);
	ptr = buf;
	left = size;
	while (left > 0) {
		while (!HAL_UART_IsTxReady(uart)) {
			; /* wait FIFO become not full */
		}

		HAL_UART_PutTxData(uart, *ptr);
		++ptr;
		--left;
	}

	return size;
}

/**
 * @brief Receive an amount of data in polling mode
 * @param[in] uartID ID of the specified UART
 * @param[out] buf Pointer to the data buffer
 * @param[in] size The maximum number of bytes to be received.
 *                 The actual received bytes can be less than this.
 * @param[in] msec Timeout value in millisecond to receive data.
 *                 HAL_WAIT_FOREVER for no timeout.
 * @return Number of bytes received, -1 on error
 *
 * @note This function is not thread safe. If using the UART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 */
int32_t HAL_UART_Receive_Poll(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec)
{
	UART_T *uart;
	uint8_t *ptr;
	int32_t left;
	uint32_t endTick;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}
#if HAL_UART_OPT_IT
	if (priv->rxReadyCallback != NULL) {
		HAL_WRN("rx cb is enabled\n");
		return -1;
	}
#endif

	uart = UART_GetInstance(priv);
	ptr = buf;
	left = size;

	if (msec != HAL_WAIT_FOREVER) {
		endTick = HAL_Ticks() + HAL_MSecsToTicks(msec);
	}
	while (left > 0) {
		if (HAL_UART_IsRxReady(uart)) {
			*ptr = HAL_UART_GetRxData(uart);
			++ptr;
			--left;
		} else if (msec != HAL_WAIT_FOREVER) {
			if (HAL_TimeAfter(HAL_Ticks(), endTick)) {
				break;
			}
		}
	}

	return (size - left);
}

/**
 * @brief Start or stop to transmit break characters
 * @param[in] uartID ID of the specified UART
 * @param[in] isSet
 *     @arg !0 Start to transmit break characters
 *     @arg  0 Stop to transmit break characters
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_SetBreakCmd(UART_ID uartID, int8_t isSet)
{
	UART_T *uart;
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}
	uart = UART_GetInstance(priv);

	if (isSet) {
		HAL_SET_BIT(uart->LINE_CTRL, UART_BREAK_CTRL_BIT);
	} else {
		HAL_CLR_BIT(uart->LINE_CTRL, UART_BREAK_CTRL_BIT);
	}
	return HAL_OK;
}

#ifdef CONFIG_PM
/**
 * @brief Set PM mode to be bypassed
 * @param[in] uartID ID of the specified UART
 * @param[in] mode Bit mask of PM mode to be bypassed
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_SetBypassPmMode(UART_ID uartID, uint8_t mode)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}
	priv->bypassPmMode = mode;
	return HAL_OK;
}
#endif

/**
 * @brief Set the UART's configuration according to the specified parameters
 * @param[in] uartID ID of the specified UART
 * @param[in] param Pointer to UART_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_SetConfig(UART_ID uartID, const UART_InitParam *param)
{
	UART_Private *priv;
	UART_T *uart;
	uint32_t tmp;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}
	uart = UART_GetInstance(priv);

	UART_ConfigStart(uart);

	/* set baud rate, parity, stop bits, data bits */
	tmp = UART_CalcClkDiv(param->baudRate);
	UART_SetClkDiv(uart, (uint16_t)tmp);

	HAL_MODIFY_REG(uart->LINE_CTRL,
		           UART_PARITY_MASK | UART_STOP_BITS_MASK | UART_DATA_BITS_MASK,
		           param->parity | param->stopBits | param->dataBits);

	UART_ConfigFinish(uart);

	/* set auto hardware flow control */
	if (uartID != UART0_ID && param->isAutoHwFlowCtrl) {
		tmp = UART_AUTO_FLOW_CTRL_EN_BIT | UART_RTS_ASSERT_BIT;
	} else {
		tmp = 0;
	}
	HAL_MODIFY_REG(uart->MODEM_CTRL,
		           UART_AUTO_FLOW_CTRL_EN_BIT | UART_RTS_ASSERT_BIT,
		           tmp);

#ifdef CONFIG_PM
	HAL_Memcpy(&priv->param, param, sizeof(UART_InitParam));
#endif
	return HAL_OK;
}

/**
 * @brief Set the delay parameter between two transmitting bytes
 * @param[in] uartID ID of the specified UART
 * @param[in] txDelay The delay parameter between two transmitting bytes
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_UART_SetTxDelay(UART_ID uartID, uint8_t txDelay)
{
	UART_Private *priv;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	UART_SetTxDelay(UART_GetInstance(priv), txDelay);
#ifdef CONFIG_PM
	priv->txDelay = txDelay;
#endif
	return HAL_OK;
}
