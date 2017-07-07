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

#ifndef _DRIVER_CHIP_HAL_UART_H_
#define _DRIVER_CHIP_HAL_UART_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	UART0_ID = 0,
	UART1_ID = 1,
	UART_NUM = 2,
} UART_ID;

typedef struct
{
	union {
		__I  uint32_t RX_BUF;	/* 8-bit valid */
		__O  uint32_t TX_HOLD;	/* 8-bit valid */
		__IO uint32_t DIV_LOW;	/* 8-bit valid */
	} RBR_THR_DLL;					/* UART Receive buffer/Transmit holding/Divisor latch low Register, Address offset: 0x00 */
	union {
		__IO uint32_t DIV_HIGH;
		__IO uint32_t IRQ_EN;
	} DLH_IER;						/* UART Divisor latch high/interrupt enable Register,               Address offset: 0x04 */
	union {
		__I  uint32_t IRQ_ID;
		__O  uint32_t FIFO_CTRL;
	} IIR_FCR;						/* UART Interrupt Identity/fifo control Register,                   Address offset: 0x08 */
	__IO uint32_t LINE_CTRL;        /* UART Line Control Register,                                      Address offset: 0x0C */
	__IO uint32_t MODEM_CTRL;        /* UART Modem Control Register,                                     Address offset: 0x10 */
	__I  uint32_t LINE_STATUS;        /* UART Line Status Register,                                       Address offset: 0x14 */
	__I  uint32_t MODEM_STATUS;        /* UART Modem Stauts Register,                                      Address offset: 0x18 */
	__IO uint32_t SCRATCH;        /* UART Scratch Register,                                           Address offset: 0x1C */
	 	 uint32_t RESERVED1[23];   /* Reserved, 0x30-0x78                                                                   */
	__I  uint32_t STATUS;        /* UART Status Register,                                            Address offset: 0x7C */
	__I  uint32_t TX_FIFO_LEVEL;        /* UART Transmit fifo level Register,                               Address offset: 0x80 */
	__I  uint32_t RX_FIFO_LEVEL;        /* UART Receive fifo level Register,                                Address offset: 0x84 */
	     uint32_t RESERVED2[7];    /* Reserved, 0x88-0xA0                                                                   */
	__IO uint32_t HALT;       /* UART Regular sequence Register,                                  Address offset: 0xA4 */
	     uint32_t RESERVED3[9];    /* Reserved, 0xA8-0xC8                                                                   */
	__IO uint32_t TX_DELAY;        /* UART TX Delay Register,                                          Address offset: 0xCC */
	     uint32_t RESERVED4[1];    /* Reserved, 0xD0                                                                        */
	__IO uint32_t BAUD_DECT_CTRL;       /* UART Baudrate Detection Control Register,                        Address offset: 0xD4 */
	__IO uint32_t BAUD_DECT_VAL_LOW;      /* UART Baudrate Detection Counter Low Register,                    Address offset: 0xD8 */
	__IO uint32_t BAUD_DECT_VAL_HIGH;      /* UART Baudrate Detection Counter High Register,                   Address offset: 0xDC */
} UART_T;

#define UART0	((UART_T *)UART0_BASE)
#define UART1	((UART_T *)UART1_BASE)

/* UARTx->RBR_THR_DLL.RX_BUF, R */
#define UART_RX_DATA_MASK	0xFFU

/* UARTx->RBR_THR_DLL.TX_HOLD, W */
#define UART_TX_DATA_MASK	0xFFU

/* UARTx->RBR_THR_DLL.DIV_LOW, R/W */
#define UART_DIV_LOW_MASK	0xFFU

/* UARTx->DLH_IER.DIV_HIGH, R/W */
#define UART_DIV_HIGH_MASK	0xFFU

/* UARTx->DLH_IER.IRQ_EN, R/W */
#define UART_TX_FIFO_TRIG_MODE_EN_BIT	HAL_BIT(7)
#define UART_RS485_IRQ_EN_BIT			HAL_BIT(4)
#define UART_MODEM_STATUS_IRQ_EN_BIT	HAL_BIT(3)
#define UART_LINE_STATUS_IRQ_EN_BIT		HAL_BIT(2)
#define UART_TX_READY_IRQ_EN_BIT		HAL_BIT(1)
#define UART_RX_READY_IRQ_EN_BIT		HAL_BIT(0)

/* UARTx->IIR_FCR.IRQ_ID, R */
#define UART_IID_SHIFT					0
#define UART_IID_MASK					(0xFU << UART_IID_SHIFT)
typedef enum {
	UART_IID_MODEM_STATUS = (0x0U << UART_IID_SHIFT),
	UART_IID_NONE         = (0x1U << UART_IID_SHIFT),
	UART_IID_TX_READY     = (0x2U << UART_IID_SHIFT),
	UART_IID_RS485        = (0x3U << UART_IID_SHIFT),
	UART_IID_RX_READY     = (0x4U << UART_IID_SHIFT),
	UART_IID_LINE_STATUS  = (0x6U << UART_IID_SHIFT),
	UART_IID_BUSY_DETECT  = (0x7U << UART_IID_SHIFT),
	UART_IID_CHAR_TIMEOUT = (0xCU << UART_IID_SHIFT),
} UART_IIDType;

/* UARTx->IIR_FCR.FIFO_CTRL, W */
#define UART_RX_FIFO_TRIG_LEVEL_SHIFT	6
#define UART_RX_FIFO_TRIG_LEVEL_MASK	(0x3 << UART_RX_FIFO_TRIG_LEVEL_SHIFT)
typedef enum {
	UART_RX_FIFO_TRIG_LEVEL_ONE_CHAR	 = (0x0 << UART_RX_FIFO_TRIG_LEVEL_SHIFT),
	UART_RX_FIFO_TRIG_LEVEL_QUARTER_FULL = (0x1 << UART_RX_FIFO_TRIG_LEVEL_SHIFT),
	UART_RX_FIFO_TRIG_LEVEL_HALF_FULL	 = (0x2 << UART_RX_FIFO_TRIG_LEVEL_SHIFT),
	UART_RX_FIFO_TRIG_LEVEL_NEARLY_FULL  = (0x3 << UART_RX_FIFO_TRIG_LEVEL_SHIFT),
} UART_RxFifoTrigLevel;

#define UART_TX_FIFO_TRIG_LEVEL_SHIFT	4
#define UART_TX_FIFO_TRIG_LEVEL_MASK	(0x3 << UART_TX_FIFO_TRIG_LEVEL_SHIFT)
typedef enum {
	UART_TX_FIFO_TRIG_LEVEL_EMPTY	     = (0x0 << UART_TX_FIFO_TRIG_LEVEL_SHIFT),
	UART_TX_FIFO_TRIG_LEVEL_TWO_CHAR     = (0x1 << UART_TX_FIFO_TRIG_LEVEL_SHIFT),
	UART_TX_FIFO_TRIG_LEVEL_QUARTER_FULL = (0x2 << UART_TX_FIFO_TRIG_LEVEL_SHIFT),
	UART_TX_FIFO_TRIG_LEVEL_HALF_FULL    = (0x3 << UART_TX_FIFO_TRIG_LEVEL_SHIFT),
} UART_TxFifoTrigLevel;

#define UART_DMA_MODE_SHIFT		3
#define UART_DMA_MODE_MASK		(1U << UART_DMA_MODE_SHIFT)
typedef enum {
	UART_DMA_MODE_0 = (0U << UART_DMA_MODE_SHIFT),
	UART_DMA_MODE_1 = (1U << UART_DMA_MODE_SHIFT)
} UART_DMAMode;

#define UART_TX_FIFO_RESET_BIT		HAL_BIT(2)
#define UART_RX_FIFO_RESET_BIT		HAL_BIT(1)
#define UART_FIFO_EN_BIT			HAL_BIT(0)

/* UARTx->LINE_CTRL, R/W */
#define UART_DIV_ACCESS_BIT		HAL_BIT(7)
#define UART_BREAK_CTRL_BIT		HAL_BIT(6)

#define UART_PARITY_EN_BIT		HAL_BIT(3)
#define UART_PARITY_SEL_SHIFT	4
#define UART_PARITY_SEL_MASK	(0x3U << UART_PARITY_SEL_SHIFT)
#define UART_PARITY_MASK		(UART_PARITY_EN_BIT | UART_PARITY_SEL_MASK)
typedef enum {
	UART_PARITY_NONE = 0U,
	UART_PARITY_ODD  = UART_PARITY_EN_BIT | (0x0U << UART_PARITY_SEL_SHIFT),
	UART_PARITY_EVEN = UART_PARITY_EN_BIT | (0x1U << UART_PARITY_SEL_SHIFT)
} UART_Parity;

#define UART_STOP_BITS_SHIFT	2
#define UART_STOP_BITS_MASK		(0x1U << UART_STOP_BITS_SHIFT)
typedef enum {
	UART_STOP_BITS_1 = (0x0U << UART_STOP_BITS_SHIFT),
	UART_STOP_BITS_2 = (0x1U << UART_STOP_BITS_SHIFT)
} UART_StopBits; /* UART_STOP_BITS_2 is 1.5 stop bits for UART_DATA_BITS_5 */

#define UART_DATA_BITS_SHIFT	0
#define UART_DATA_BITS_MASK		(0x3U << UART_DATA_BITS_SHIFT)
typedef enum {
	UART_DATA_BITS_5 = (0x0U << UART_DATA_BITS_SHIFT),
	UART_DATA_BITS_6 = (0x1U << UART_DATA_BITS_SHIFT),
	UART_DATA_BITS_7 = (0x2U << UART_DATA_BITS_SHIFT),
	UART_DATA_BITS_8 = (0x3U << UART_DATA_BITS_SHIFT)
} UART_DataBits;

/* UARTx->MODEM_CTRL, R/W */
#define UART_WORK_MODE_SHIFT	6
#define UART_WORK_MODE_MASK		(0x3U << UART_WORK_MODE_SHIFT)
typedef enum {
	UART_WORK_MODE_UART  = (0x0U << UART_WORK_MODE_SHIFT),
	UART_WORK_MODE_IRDA  = (0x1U << UART_WORK_MODE_SHIFT),
	UART_WORK_MODE_RS485 = (0x2U << UART_WORK_MODE_SHIFT),
} UART_WorkMode;

#define UART_AUTO_FLOW_CTRL_EN_BIT	HAL_BIT(5)
#define UART_LOOP_BACK_EN_BIT		HAL_BIT(4)
#define UART_RTS_ASSERT_BIT			HAL_BIT(1)
#define UART_DTR_ASSERT_BIT			HAL_BIT(0)

/* UARTx->LINE_STATUS, R */
#define UART_FIFO_ERROR_BIT			HAL_BIT(7)
#define UART_TX_EMPTY_BIT			HAL_BIT(6)
#define UART_TX_HOLD_EMPTY_BIT		HAL_BIT(5)
#define UART_BREAK_IRQ_BIT			HAL_BIT(4)
#define UART_FRAME_ERROR_BIT		HAL_BIT(3)
#define UART_PARITY_ERROR_BIT		HAL_BIT(2)
#define UART_OVERRUN_ERROR_BIT		HAL_BIT(1)
#define UART_RX_READY_BIT			HAL_BIT(0)
#define UART_LINE_STATUS_MASK		0xFFU

/* UARTx->MODEM_STATUS, R */
#define UART_CTS_ASSERTED_BIT		HAL_BIT(4)
#define UART_DELTA_CTS_BIT			HAL_BIT(0)
#define UART_MODEM_STATUS_MASK		0xF3U

/* UARTx->STATUS, R */
#define UART_RX_FIFO_FULL_BIT		HAL_BIT(4)
#define UART_RX_FIFO_NOT_EMPTY_BIT	HAL_BIT(3)
#define UART_TX_FIFO_EMPTY_BIT		HAL_BIT(2)
#define UART_TX_FIFO_NOT_FULL_BIT	HAL_BIT(1)
#define UART_BUSY_BIT				HAL_BIT(0)
#define UART_STATUS_MASK			0x1FU

/* UARTx->TX_FIFO_LEVEL, R/O */
#define UART_TX_FIFO_LEVEL_SHIFT	0
#define UART_TX_FIFO_LEVEL_VMASK	0x7FU

/* UARTx->RX_FIFO_LEVEL, R/O */
#define UART_RX_FIFO_LEVEL_SHIFT	0
#define UART_RX_FIFO_LEVEL_VMASK	0x7FU

/* UARTx->HALT, R/W */
#define UART_DMA_PTE_TX_BIT			HAL_BIT(7)
#define UART_DMA_PTE_RX_BIT			HAL_BIT(6)

#define UART_CHANGE_UPDATE_BIT		HAL_BIT(2)
#define UART_CHANGE_AT_BUSY_BIT		HAL_BIT(1)

#define UART_HALT_TX_EN_BIT			HAL_BIT(0)

/******************************************************************************/

typedef struct {
	HAL_BoardCfg		boardCfg;
	uint32_t			baudRate;	/* in bps */
	UART_Parity			parity;
	UART_StopBits 		stopBits;
	UART_DataBits		dataBits;
	int8_t				isAutoHwFlowCtrl;
} UART_InitParam;

typedef void (*UART_RxReadyCallback) (void *arg);

UART_T *HAL_UART_GetInstance(UART_ID uartID);
int HAL_UART_IsTxReady(UART_T *uart);
int HAL_UART_IsTxEmpty(UART_T *uart);
int HAL_UART_IsRxReady(UART_T *uart);
uint8_t HAL_UART_GetRxData(UART_T *uart);
void HAL_UART_PutTxData(UART_T *uart, uint8_t data);

HAL_Status HAL_UART_Init(UART_ID uartID, UART_InitParam *param);
HAL_Status HAL_UART_DeInit(UART_ID uartID);

int32_t HAL_UART_Transmit_IT(UART_ID uartID, uint8_t *buf, int32_t size);
int32_t HAL_UART_Receive_IT(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec);

HAL_Status HAL_UART_EnableRxCallback(UART_ID uartID, UART_RxReadyCallback cb, void *arg);
HAL_Status HAL_UART_DisableRxCallback(UART_ID uartID);

HAL_Status HAL_UART_EnableTxDMA(UART_ID uartID);
HAL_Status HAL_UART_EnableRxDMA(UART_ID uartID);
HAL_Status HAL_UART_DisableTxDMA(UART_ID uartID);
HAL_Status HAL_UART_DisableRxDMA(UART_ID uartID);
int32_t HAL_UART_Transmit_DMA(UART_ID uartID, uint8_t *buf, int32_t size);
int32_t HAL_UART_Receive_DMA(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec);

int32_t HAL_UART_Transmit_Poll(UART_ID uartID, uint8_t *buf, int32_t size);
int32_t HAL_UART_Receive_Poll(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec);

void HAL_UART_SetBreakCmd(UART_ID uartID, int8_t isSet);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_UART_H_ */
