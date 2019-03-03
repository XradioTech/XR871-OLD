/**
  * @file  hal_gpio.c
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

#include "driver/chip/hal_gpio.h"
#include "pm/pm.h"

#include "hal_base.h"

/* useful macros */
#define GPIO_PINS_MASK(pinNum)	((1U << pinNum) - 1)

#define GPIO_REG_BITS			32
#define GPIO_GET_REG_IDX_SHIFT(idx, shift, pin, bits)   \
    do {                                                \
        (shift) = (pin) * (bits);                       \
        (idx) = (shift) / GPIO_REG_BITS;                \
        (shift) = (shift) % GPIO_REG_BITS;              \
    } while (0)

typedef struct {
	GPIO_IRQCallback   callback;
	void              *arg;
} GPIO_PinCallback;

#ifdef CONFIG_PM
typedef struct {
	GPIO_CTRL_T gpioCtrl;
	GPIO_IRQ_T  gpioIRQ;
} GPIO_PmPrivate;
#endif

typedef struct {
	GPIO_PinCallback   *pinCb[GPIO_PORT_NUM];
#ifdef CONFIG_PM
	GPIO_PmPrivate     *pmPriv[GPIO_PORT_NUM];
#endif
} GPIO_Private;

static GPIO_Private gGpioPrivate;

#define GPIOA_IRQ_PIN_NUM	GPIOA_PIN_NUM
#define GPIOB_IRQ_PIN_NUM	(GPIOB_PIN_NUM - 2)	/* except for [PB14, PB15]*/

static const uint8_t gGpioIrqPinNum[GPIO_PORT_NUM] = {
	GPIOA_IRQ_PIN_NUM,
	GPIOB_IRQ_PIN_NUM,
};

#define GPIOA_CTRL	((GPIO_CTRL_T *)GPIOA_CTRL_BASE)
#define GPIOB_CTRL	((GPIO_CTRL_T *)GPIOB_CTRL_BASE)

#define GPIOA_IRQ	((GPIO_IRQ_T *)GPIOA_IRQ_BASE)
#define GPIOB_IRQ	((GPIO_IRQ_T *)GPIOB_IRQ_BASE)

static GPIO_CTRL_T * const gGpioPortCtrl[GPIO_PORT_NUM] = {
	GPIOA_CTRL,
	GPIOB_CTRL,
};

static GPIO_IRQ_T * const gGpioPortIrq[GPIO_PORT_NUM] = {
	GPIOA_IRQ,
	GPIOB_IRQ,
};

__STATIC_INLINE GPIO_CTRL_T *GPIO_GetCtrlInstance(GPIO_Port port)
{
	return gGpioPortCtrl[port];
}

__STATIC_INLINE GPIO_IRQ_T *GPIO_GetIRQInstance(GPIO_Port port)
{
	return gGpioPortIrq[port];
}

/*
 * Status for all GPIO pins
 *   - 0: GPIOx_Pn_F7_DISABLE
 *   - 1: not GPIOx_Pn_F7_DISABLE
 */
static uint32_t gGpioPinStatus[GPIO_PORT_NUM];

__STATIC_INLINE int GPIO_IsAllPinDisabled(void)
{
	return (gGpioPinStatus[GPIO_PORT_A] == 0 &&
	        gGpioPinStatus[GPIO_PORT_B] == 0);
}

__STATIC_INLINE void GPIO_SetPinStatus(GPIO_Port port, GPIO_Pin pin)
{
	if (GPIO_IsAllPinDisabled()) {
		HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_GPIO);
	}

	HAL_SET_BIT(gGpioPinStatus[port], HAL_BIT(pin));
}

__STATIC_INLINE void GPIO_ClearPinStatus(GPIO_Port port, GPIO_Pin pin)
{
	HAL_CLR_BIT(gGpioPinStatus[port], HAL_BIT(pin));

	if (GPIO_IsAllPinDisabled()) {
		HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_GPIO);
	}
}

/*
 * IRQ handling
 */
static void GPIO_IRQHandler(GPIO_Port port, uint32_t pinNum)
{
	uint32_t i;
	uint32_t pinMask;
	uint32_t irqStatus;
	uint32_t isPending;
	GPIO_IRQ_T *gpiox;
	GPIO_PinCallback *pinCb;

	gpiox = GPIO_GetIRQInstance(port);
	pinMask = GPIO_PINS_MASK(pinNum);
	irqStatus = gpiox->IRQ_STATUS & gpiox->IRQ_EN & pinMask; /* get pending bits */
	gpiox->IRQ_STATUS = irqStatus; /* clear pending bits */

	pinCb = gGpioPrivate.pinCb[port];
	if (pinCb == NULL) {
		return;
	}

	for (i = GPIO_PIN_0; i < pinNum && irqStatus != 0; ++i) {
		isPending = irqStatus & HAL_BIT(0);
		if (isPending && pinCb[i].callback) {
			pinCb[i].callback(pinCb[i].arg);
		}
		irqStatus >>= 1;
	}
}

void GPIOA_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PORT_A, GPIOA_IRQ_PIN_NUM);
}

void GPIOB_IRQHandler(void)
{
	GPIO_IRQHandler(GPIO_PORT_B, GPIOB_IRQ_PIN_NUM);
}

static void GPIO_EnableIRQ(GPIO_IRQ_T *gpiox, GPIO_Pin pin)
{
	HAL_SET_BIT(gpiox->IRQ_EN, HAL_BIT(pin));
}

static void GPIO_DisableIRQ(GPIO_IRQ_T *gpiox, GPIO_Pin pin)
{
	HAL_CLR_BIT(gpiox->IRQ_EN, HAL_BIT(pin));
}

static int GPIO_IsPendingIRQ(GPIO_IRQ_T *gpiox, GPIO_Pin pin)
{
	return HAL_GET_BIT_VAL(gpiox->IRQ_STATUS, pin, 1);
}

static void GPIO_ClearPendingIRQ(GPIO_IRQ_T *gpiox, GPIO_Pin pin)
{
	HAL_SET_BIT(gpiox->IRQ_STATUS, HAL_BIT(pin));
}

#ifdef CONFIG_PM

static int gpio_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	GPIO_PmPrivate **pPmPriv;
	GPIO_Port port;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		pPmPriv = gGpioPrivate.pmPriv;
		for (port = GPIO_PORT_A; port < GPIO_PORT_NUM; ++port) {
			if (pPmPriv[port] != NULL) {
				HAL_Memcpy(&pPmPriv[port]->gpioCtrl,
				           GPIO_GetCtrlInstance(port),
				           sizeof(GPIO_CTRL_T));
				HAL_Memcpy(&pPmPriv[port]->gpioIRQ,
				           GPIO_GetIRQInstance(port),
				           sizeof(GPIO_IRQ_T));
			}
		}
		break;
	default:
		break;
	}

	HAL_DBG("%s ok, pin status 0x%08x, 0x%08x\n",
			__func__, gGpioPinStatus[GPIO_PORT_A], gGpioPinStatus[GPIO_PORT_B]);

	return 0;
}

static int gpio_resume(struct soc_device *dev, enum suspend_state_t state)
{
	GPIO_PmPrivate **pPmPriv;
	GPIO_Port port;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		pPmPriv = gGpioPrivate.pmPriv;
		for (port = GPIO_PORT_A; port < GPIO_PORT_NUM; ++port) {
			if (pPmPriv[port] != NULL) {
				HAL_Memcpy(GPIO_GetCtrlInstance(port),
				           &pPmPriv[port]->gpioCtrl,
				           sizeof(GPIO_CTRL_T));
				HAL_Memcpy(GPIO_GetIRQInstance(port),
				           &pPmPriv[port]->gpioIRQ,
				           sizeof(GPIO_IRQ_T));
			}
		}
		break;
	default:
		break;
	}

	HAL_DBG("%s ok, pin status 0x%08x, 0x%08x\n",
			__func__, gGpioPinStatus[GPIO_PORT_A], gGpioPinStatus[GPIO_PORT_B]);

	return 0;
}

static const struct soc_device_driver gpio_drv = {
	.name = "gpio",
	.suspend_noirq = gpio_suspend,
	.resume_noirq = gpio_resume,
};

static struct soc_device gpio_dev = {
	.name = "gpio",
	.driver = &gpio_drv,
};

#define GPIO_DEV (&gpio_dev)

#endif /* CONFIG_PM */

void HAL_GPIO_GlobalInit(const GPIO_GlobalInitParam *param)
{
	void *tmp;
	GPIO_Port port;
	GPIO_Private *priv = &gGpioPrivate;

	for (port = GPIO_PORT_A; port < GPIO_PORT_NUM; ++port) {
		if (param->portIRQUsed & HAL_BIT(port)) {
			tmp = HAL_Malloc(sizeof(GPIO_PinCallback) * gGpioIrqPinNum[port]);
			HAL_Memset(tmp, 0, sizeof(GPIO_PinCallback) * gGpioIrqPinNum[port]);
			priv->pinCb[port] = tmp;
		} else {
			priv->pinCb[port] = NULL;
		}
	}


#ifdef CONFIG_PM
	for (port = GPIO_PORT_A; port < GPIO_PORT_NUM; ++port) {
		if (param->portPmBackup & HAL_BIT(port)) {
			tmp = HAL_Malloc(sizeof(GPIO_PmPrivate));
			HAL_Memset(tmp, 0, sizeof(GPIO_PmPrivate));
			priv->pmPriv[port] = tmp;
		} else {
			priv->pmPriv[port] = NULL;
		}
	}

	pm_register_ops(GPIO_DEV);
#endif
}

/**
 * @brief Initialize the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] param Pointer to GPIO_InitParam structure
 * @return None
 */
void HAL_GPIO_Init(GPIO_Port port, GPIO_Pin pin, const GPIO_InitParam *param)
{
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	GPIO_SetPinStatus(port, pin);

	/* set working mode (function) */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_MODE_BITS);
	HAL_MODIFY_REG(gpiox->MODE[regIdx],
				   GPIO_CTRL_MODE_VMASK << bitShift,
				   (param->mode & GPIO_CTRL_MODE_VMASK) << bitShift);

	/* set driving */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_DRIVING_BITS);
	HAL_MODIFY_REG(gpiox->DRIVING[regIdx],
				   GPIO_CTRL_DRIVING_VMASK << bitShift,
				   (param->driving & GPIO_CTRL_DRIVING_VMASK) << bitShift);

	/* set pull */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_PULL_BITS);
	HAL_MODIFY_REG(gpiox->PULL[regIdx],
				   GPIO_CTRL_PULL_VMASK << bitShift,
				   (param->pull & GPIO_CTRL_PULL_VMASK) << bitShift);

	if (param->mode == GPIOx_Pn_F7_DISABLE) {
		GPIO_ClearPinStatus(port, pin);
	}

	HAL_ExitCriticalSection(flags);
}

/**
 * @brief Deinitialize the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @return None
 * @note After deinitialization, the GPIO is in its reset state:
 *       (GPIOx_Pn_F7_DISABLE, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE).
 */
void HAL_GPIO_DeInit(GPIO_Port port, GPIO_Pin pin)
{
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	/* set working mode (function) to disable */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_MODE_BITS);
	HAL_MODIFY_REG(gpiox->MODE[regIdx],
				   GPIO_CTRL_MODE_VMASK << bitShift,
				   (GPIOx_Pn_F7_DISABLE & GPIO_CTRL_MODE_VMASK) << bitShift);

	/* set driving to default value (GPIO_DRIVING_LEVEL_1) */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_DRIVING_BITS);
	HAL_MODIFY_REG(gpiox->DRIVING[regIdx],
				   GPIO_CTRL_DRIVING_VMASK << bitShift,
				   (GPIO_DRIVING_LEVEL_1 & GPIO_CTRL_DRIVING_VMASK) << bitShift);

	/* set pull to default value (GPIO_PULL_NONE) */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_PULL_BITS);
	HAL_MODIFY_REG(gpiox->PULL[regIdx],
				   GPIO_CTRL_PULL_VMASK << bitShift,
				   (GPIO_PULL_NONE & GPIO_CTRL_PULL_VMASK) << bitShift);

	GPIO_ClearPinStatus(port, pin);

	HAL_ExitCriticalSection(flags);
}

/**
 * @brief Get the configuration of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[out] param Pointer to GPIO_InitParam structure
 * @return None
 */
void HAL_GPIO_GetConfig(GPIO_Port port, GPIO_Pin pin, GPIO_InitParam *param)
{
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	/* get working mode (function) */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_MODE_BITS);
	param->mode = HAL_GET_BIT_VAL(gpiox->MODE[regIdx], bitShift,
	                              GPIO_CTRL_MODE_VMASK);

	/* get driving */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_DRIVING_BITS);
	param->driving = HAL_GET_BIT_VAL(gpiox->DRIVING[regIdx], bitShift,
	                                 GPIO_CTRL_DRIVING_VMASK);

	/* get pull */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_PULL_BITS);
	param->pull = HAL_GET_BIT_VAL(gpiox->PULL[regIdx], bitShift,
	                              GPIO_CTRL_PULL_VMASK);

	HAL_ExitCriticalSection(flags);
}

void HAL_GPIO_SetMode(GPIO_Port port, GPIO_Pin pin, GPIO_WorkMode mode)
{
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	GPIO_SetPinStatus(port, pin);

	/* set working mode (function) */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_MODE_BITS);
	HAL_MODIFY_REG(gpiox->MODE[regIdx],
				   GPIO_CTRL_MODE_VMASK << bitShift,
				   (mode & GPIO_CTRL_MODE_VMASK) << bitShift);

	if (mode == GPIOx_Pn_F7_DISABLE) {
		GPIO_ClearPinStatus(port, pin);
	}

	HAL_ExitCriticalSection(flags);
}

void HAL_GPIO_SetDriving(GPIO_Port port, GPIO_Pin pin, GPIO_DrivingLevel driving)
{
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	/* set driving */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_DRIVING_BITS);
	HAL_MODIFY_REG(gpiox->DRIVING[regIdx],
				   GPIO_CTRL_DRIVING_VMASK << bitShift,
				   (driving & GPIO_CTRL_DRIVING_VMASK) << bitShift);

	HAL_ExitCriticalSection(flags);
}

void HAL_GPIO_SetPull(GPIO_Port port, GPIO_Pin pin, GPIO_PullType pull)
{
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	/* set pull */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_CTRL_PULL_BITS);
	HAL_MODIFY_REG(gpiox->PULL[regIdx],
				   GPIO_CTRL_PULL_VMASK << bitShift,
				   (pull & GPIO_CTRL_PULL_VMASK) << bitShift);

	HAL_ExitCriticalSection(flags);
}

/**
 * @brief Set the state of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] state GPIO pin state
 * @return None
 */
void HAL_GPIO_WritePin(GPIO_Port port, GPIO_Pin pin, GPIO_PinState state)
{
	GPIO_CTRL_T *gpiox;
	unsigned long flags;

	gpiox = GPIO_GetCtrlInstance(port);
	flags = HAL_EnterCriticalSection();

	if (state == GPIO_PIN_LOW)
		HAL_CLR_BIT(gpiox->DATA, HAL_BIT(pin));
	else
		HAL_SET_BIT(gpiox->DATA, HAL_BIT(pin));

	HAL_ExitCriticalSection(flags);
}

/**
 * @brief Get the state of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @return GPIO pin state
 */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_Port port, GPIO_Pin pin)
{
	GPIO_CTRL_T *gpiox;

	gpiox = GPIO_GetCtrlInstance(port);
	return (GPIO_PinState)HAL_GET_BIT_VAL(gpiox->DATA, pin, 1);
}

/**
 * @brief Set the state of the specified GPIO port
 * @param[in] port GPIO port
 * @param[in] portMask GPIO port state, bit mask of all pins
 * @return None
 */
void HAL_GPIO_WritePort(GPIO_Port port, uint32_t portMask)
{
	GPIO_CTRL_T *gpiox;

	gpiox = GPIO_GetCtrlInstance(port);
	gpiox->DATA = portMask;
}

/**
 * @brief Get the state of the specified GPIO port
 * @param[in] port GPIO port
 * @return GPIO port state, bit mask of all pins
 */
uint32_t HAL_GPIO_ReadPort(GPIO_Port port)
{
	GPIO_CTRL_T *gpiox;

	gpiox = GPIO_GetCtrlInstance(port);
	return gpiox->DATA;
}

/**
 * @brief Enable the IRQ of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @param[in] param Pointer to GPIO_IrqParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_GPIO_EnableIRQ(GPIO_Port port, GPIO_Pin pin, const GPIO_IrqParam *param)
{
	GPIO_Private *priv;
	uint32_t regIdx;
	uint32_t bitShift;
	GPIO_IRQ_T *gpiox;
	GPIO_PinCallback *pinCb;
	IRQn_Type IRQn;
	NVIC_IRQHandler IRQHandler;
	uint8_t irqPinNum;
	unsigned long flags;

	priv = &gGpioPrivate;

	switch (port) {
	case GPIO_PORT_A:
		pinCb = priv->pinCb[port];
		IRQn = GPIOA_IRQn;
		IRQHandler = GPIOA_IRQHandler;
		irqPinNum = GPIOA_IRQ_PIN_NUM;
		break;
	case GPIO_PORT_B:
		pinCb = priv->pinCb[port];
		IRQn = GPIOB_IRQn;
		IRQHandler = GPIOB_IRQHandler;
		irqPinNum = GPIOB_IRQ_PIN_NUM;
		break;
	default:
		pinCb = NULL;
		break;
	}

	if (pinCb == NULL || pin >= irqPinNum) {
		HAL_ERR("Invalid pin (%d, %d)\n", port, pin);
		return HAL_ERROR;
	}

	/* set callback */
	pinCb[pin].callback = param->callback;
	pinCb[pin].arg = param->arg;

	gpiox = GPIO_GetIRQInstance(port);
	flags = HAL_EnterCriticalSection();

	/* set IRQ trigger mode */
	GPIO_GET_REG_IDX_SHIFT(regIdx, bitShift, pin, GPIO_IRQ_EVT_BITS);
	HAL_MODIFY_REG(gpiox->IRQ_MODE[regIdx],
				   GPIO_IRQ_EVT_VMASK << bitShift,
				   (param->event & GPIO_IRQ_EVT_VMASK) << bitShift);

	if (GPIO_IsPendingIRQ(gpiox, pin)) {
		GPIO_ClearPendingIRQ(gpiox, pin);
	}
	if (gpiox->IRQ_EN == 0) {
		HAL_NVIC_ConfigExtIRQ(IRQn, IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);
	}
	GPIO_EnableIRQ(gpiox, pin);
	HAL_ExitCriticalSection(flags);

	return HAL_OK;
}

/**
 * @brief Disable the IRQ of the specified GPIO
 * @param[in] port GPIO port
 * @param[in] pin GPIO pin number
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_GPIO_DisableIRQ(GPIO_Port port, GPIO_Pin pin)
{
	GPIO_Private *priv;
	GPIO_IRQ_T *gpiox;
	GPIO_PinCallback *pinCb;
	IRQn_Type IRQn;
	uint8_t irqPinNum;
	unsigned long flags;

	priv = &gGpioPrivate;

	switch (port) {
	case GPIO_PORT_A:
		pinCb = priv->pinCb[port];
		IRQn = GPIOA_IRQn;
		irqPinNum = GPIOA_IRQ_PIN_NUM;
		break;
	case GPIO_PORT_B:
		pinCb = priv->pinCb[port];
		IRQn = GPIOB_IRQn;
		irqPinNum = GPIOB_IRQ_PIN_NUM;
		break;
	default:
		pinCb = NULL;
		break;
	}

	if (pinCb == NULL || pin >= irqPinNum) {
		HAL_ERR("Invalid pin (%d, %d)\n", port, pin);
		return HAL_ERROR;
	}

	gpiox = GPIO_GetIRQInstance(port);
	flags = HAL_EnterCriticalSection();
	GPIO_DisableIRQ(gpiox, pin);
	if (gpiox->IRQ_EN == 0) {
		HAL_NVIC_DisableIRQ(IRQn);
	}
	if (GPIO_IsPendingIRQ(gpiox, pin)) {
		GPIO_ClearPendingIRQ(gpiox, pin);
	}
	HAL_ExitCriticalSection(flags);
	pinCb[pin].callback = NULL;
	pinCb[pin].arg = NULL;

	return HAL_OK;
}

/**
 * @brief Set debounce parameters of the specified GPIO port
 * @param[in] port GPIO port
 * @param[in] param Pointer to GPIO_IrqDebParam structure
 * @return None
 *
 * @note The debounce parameters are for all pins of the specified GPIO port
 */
void HAL_GPIO_SetIRQDebounce(GPIO_Port port, const GPIO_IrqDebParam *param)
{
	GPIO_IRQ_T *gpiox;

	gpiox = GPIO_GetIRQInstance(port);
	gpiox->IRQ_DEBOUNCE = (param->clkSrc << GPIO_IRQ_DEB_CLK_SRC_SHIFT) |
	                      (param->clkPrescaler << GPIO_IRQ_DEB_CLK_PRESCALER_SHIFT);
}

/**
 * @brief Configure the GPIOs pinmux by the specified parameters
 * @param[in] param Pointer to the array of GPIO_PinMuxParam structure, one
 *                  array element for one GPIO pinmux
 * @param[in] count Elements number of the GPIO pinmux parameters array
 * @return None
 */
void HAL_GPIO_PinMuxConfig(const GPIO_PinMuxParam *param, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i) {
		HAL_GPIO_Init(param[i].port, param[i].pin, &param[i].config);
	}
}

/**
 * @brief Deconfigure the GPIOs pinmux by the specified parameters
 * @param[in] param Pointer to the array of GPIO_PinMuxParam structure, one
 *                  array element for one GPIO pinmux, param->config is ignored.
 * @param[in] count Elements number of the GPIO pinmux parameters array
 * @return None
 */
void HAL_GPIO_PinMuxDeConfig(const GPIO_PinMuxParam *param, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i) {
		HAL_GPIO_DeInit(param[i].port, param[i].pin);
	}
}
