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

#include "driver/chip/hal_pwm.h"
#include "driver/chip/hal_ccm.h"
#include "hal_debug.h"
#include "hal_inc.h"
#include <string.h>

#define HAL_DBG_PWM		0
#define HAL_PWM_DBG(fmt, arg...)	\
	HAL_LOG(HAL_DEBUG_ON && HAL_DBG_PWM, "[HAL PWM] "fmt, ##arg)


#define MAXCNTRVAL 65535
#define RISECH(ch)	(HAL_BIT(ch) * HAL_BIT(ch))
#define FALLCH(ch)	(HAL_BIT(ch) * HAL_BIT(ch) * 2)
#define PWM_IRQ_ALL_BITS			((1 << (PWM_CH_NUM << 1)) - 1)

uint8_t PWM_IO_InitFalg = 0;

static __IO uint32_t *PWM_GetPPRRegAdress(PWM_CHID ch);

static PWM_ChGroup PWM_ChToGroup(PWM_CHID ch)
{
	uint32_t temp = ch / 2;
	HAL_PWM_DBG("ChToGroup ch = %d, group = %d\n", ch, temp);
	if (temp >= PWM_GROUP_NUM)
		return PWM_GROUP_NULL;
	return (PWM_ChGroup)temp;
}

/******************************************
 *PWM IRQ Enable Register
 *Register Name :	PIER 0~7CH
********************************************/

void HAL_PWM_OUT_IRQEnable(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->PIER, HAL_BIT(ch));
}

void HAL_PWM_OUT_IRQDisable(PWM_CHID ch)
{
	HAL_CLR_BIT(PWM->PIER, HAL_BIT(ch));
}


/******************************************
 *PWM IRQ Status Register
 *Register Name :	PISR 0~7CH
********************************************/

void PWM_OutIRQ_Clear(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->PISR, HAL_BIT(ch));
}


/******************************************
 *PWM Capture IRQ Enable Register
 *Register Name :	CIER 0~15CH
********************************************/

__STATIC_INLINE void PWM_InuptRiseEdgeIRQEnable(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->CIER,  RISECH(ch));
}

__STATIC_INLINE void PWM_InuptFallEdgeIRQEnable(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->CIER, FALLCH(ch));
}

__STATIC_INLINE void PWM_InputBothEdgeIRQEnable(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->CIER, (RISECH(ch) | FALLCH(ch)));
}

void HAL_PWM_InputIRQEnable(PWM_IrqMode irq,PWM_CHID ch)
{
	if (irq == PWM_IRQ_RISEEDGE)
		PWM_InuptRiseEdgeIRQEnable(ch);
	else if (irq == PWM_IRQ_FALLEDGE)
		PWM_InuptFallEdgeIRQEnable(ch);
	else if (irq == PWM_IRQ_BOTHEDGE)
		PWM_InputBothEdgeIRQEnable(ch);
}


__STATIC_INLINE void PWM_InputRiseEdgeIRQDisable(PWM_CHID ch)
{
	HAL_CLR_BIT(PWM->CIER, RISECH(ch));
}

__STATIC_INLINE void PWM_InputFallEdgeIRQDisable(PWM_CHID ch)
{
	HAL_CLR_BIT(PWM->CIER, FALLCH(ch));
}

__STATIC_INLINE void PWM_InputBothEdgeIRQDisable(PWM_CHID ch)
{
	HAL_CLR_BIT(PWM->CIER, FALLCH(ch) | RISECH(ch));
}

void HAL_PWM_InputIRQDisable(PWM_IrqMode irq,PWM_CHID ch)
{
	if (irq == PWM_IRQ_RISEEDGE)
		PWM_InputRiseEdgeIRQDisable(ch);
	else if (irq == PWM_IRQ_FALLEDGE)
		PWM_InputFallEdgeIRQDisable(ch);
	else if (irq == PWM_IRQ_BOTHEDGE)
		PWM_InputBothEdgeIRQDisable(ch);
}

/******************************************
 *PWM Capture IRQ Status Register
 *Register Name :	CISR 0~15 CH
********************************************/

__STATIC_INLINE __IO uint32_t PWM_InputIRQStatus()
{
	return PWM->CISR;
}

__STATIC_INLINE void PWM_InputRiseEdgeIRQClear(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->CISR, RISECH(ch));
}

__STATIC_INLINE void PWM_InputFallEdgeIRQClear(PWM_CHID ch)
{
	HAL_SET_BIT(PWM->CISR, FALLCH(ch));
}

/******************************************
 *PWMXX clock configuration Register
 *Register Name :	PCCRXX
********************************************/

/*Frequency division PWM pair clock :  1、2、4、8、16、32、64、128、256 */
static void PWM_Src_Clk_Div(__IO uint32_t *Register, PWM_SRC_CLK_DIV div)
{
	if (Register == NULL)
		return;
	uint32_t p = *Register;
	p &= ~PWM_SRC_CLK_DIV_V;
	p |= div;
	*Register = p;
}

/*Gating clock to PWM pair*/
typedef enum {
	PWM_CLK_NO_GAT,
	PWM_CLK_GAT,
}PWM_CLK_CTRL;

__STATIC_INLINE void PWM_Src_Clk_Gating(__IO uint32_t *Register, PWM_CLK_CTRL clkGating)
{
	if (Register == NULL)
		return;
	if (clkGating == PWM_CLK_GAT)
		HAL_SET_BIT(*Register, PWM_CH_CLK_GATING);
	else if (clkGating == PWM_CLK_NO_GAT)
		HAL_CLR_BIT(*Register, PWM_CH_CLK_GATING);
}

/*Bypass colck source to PWM Pair output*/

__STATIC_INLINE void PWM_Src_Clk_LowChBypass(__IO uint32_t *Register)
{
	if (Register == NULL)
		return;
	HAL_SET_BIT(*Register, PWM_LOW_CH_CLKBYPASS);
}

__STATIC_INLINE void PWM_Src_Clk_HighChBypass(__IO uint32_t *Register)
{
	if (Register == NULL)
		return;
	HAL_SET_BIT(*Register, PWM_HIGH_CH_CLKBYPASS);
}


__STATIC_INLINE void PWM_Src_Clk_LowChNoBypass(__IO uint32_t *Register)
{
	if (Register == NULL)
		return;
	HAL_CLR_BIT(*Register, PWM_LOW_CH_CLKBYPASS);
}


__STATIC_INLINE void PWM_Src_Clk_HighChNoBypass(__IO uint32_t *Register)
{
	if (Register == NULL)
		return;
	HAL_CLR_BIT(*Register, PWM_HIGH_CH_CLKBYPASS);
}

/*Choose source clock for PWM group: 24MHZ/100MHZ*/
static void PWM_SrcClockSelet(__IO uint32_t *Register, PWM_SrcClock clk)
{
	if (Register == NULL)
		return;
	if (clk == PWM_CLK_HOSC)
		*Register &= ~PWM_SRC_CLK_SELECT;
	else if (clk == PWM_CLK_APB1){
		HAL_SET_BIT(*Register, PWM_SRC_CLK_SELECT);
		HAL_CLR_BIT(*Register, HAL_BIT(7));
	}
}

/*Get Register Adress*/
static __IO uint32_t *PWM_GetPCCRAdress(PWM_ChGroup chGroup)
{
	if (chGroup >= PWM_GROUP_NUM)
		return NULL;

	return &PWM->PCCR[chGroup];
}

void PWM_ClockCtrl(PWM_CHID ch, PWM_FunCtrl en)
{
	PWM_ChGroup chGroup = PWM_ChToGroup(ch);
	__IO uint32_t *Register = NULL;
	Register = PWM_GetPCCRAdress(chGroup);
	if (en == PWM_FUNENABLE)
		PWM_Src_Clk_Gating(Register, PWM_CLK_GAT);
	else if (en == PWM_FUNDISABLE)
		PWM_Src_Clk_Gating(Register, PWM_CLK_NO_GAT);
}

void PWM_GroupClockCtrl(PWM_ChGroup chGroup, PWM_FunCtrl en)
{
	__IO uint32_t *Register = NULL;
	Register = PWM_GetPCCRAdress(chGroup);
	if (en == PWM_FUNENABLE)
		PWM_Src_Clk_Gating(Register, PWM_CLK_GAT);
	else if (en == PWM_FUNDISABLE)
		PWM_Src_Clk_Gating(Register, PWM_CLK_NO_GAT);
}

/*Config PWM pair clock*/
static void PWM_SrcClockInit(PWM_SrcClk *set)
{
	__IO uint32_t *Register = NULL;
	Register = PWM_GetPCCRAdress(set->chGroup);
	if (Register != NULL) {
		PWM_SrcClockSelet(Register, set->srcClk);
		PWM_Src_Clk_Div(Register, set->srcClkDiv);
	}
}

/******************************************
 *PDZCRXX Dead Zone Control Register
 *Register Name :	PDZCRXX
********************************************/

/*Get Register Adress*/
static __IO uint32_t *PWM_GetPDZCRAdress(PWM_ChGroup chGroup)
{
	if (chGroup >= PWM_GROUP_NUM)
		return NULL;

	return &PWM->PDZCR[chGroup];
}

HAL_Status HAL_PWM_DeadZoneEnable(PWM_ChGroup chGroup)
{
	__IO uint32_t *Register = NULL;
	Register = PWM_GetPDZCRAdress(chGroup);
	if (Register == NULL)
		return HAL_ERROR;
	HAL_SET_BIT(*Register, PWM_CH_DZ_EN);
	return HAL_OK;
}

HAL_Status HAL_PWM_DeadZoneDisable(PWM_ChGroup chGroup)
{
	__IO uint32_t *Register = NULL;
	Register = PWM_GetPDZCRAdress(chGroup);
	if (Register == NULL)
		return HAL_ERROR;
	HAL_CLR_BIT(*Register, PWM_CH_DZ_EN);
	return HAL_OK;
}

HAL_Status HAL_PWM_SetDeadZoneTime(PWM_ChGroup chGroup, uint8_t deadZoneTime)
{
	__IO uint32_t *Register = NULL;
	Register = PWM_GetPDZCRAdress(chGroup);
	if (Register == NULL)
		return HAL_ERROR;
	uint32_t p = *Register;
	p &= ~PWM_CH_DZ_INV;
	p |= (deadZoneTime << 8);
	*Register = p;
	return HAL_OK;
}

/******************************************
 *PWM control Register:	CH 0~7
 *Register Name :	PCR
********************************************/
static __IO uint32_t* PWM_GetPCRRegAdress(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return NULL;

	return &PWM->CH_REG[ch].PCR;
}

static void PWM_Prescal(__IO uint32_t *Register, uint32_t preScal)
{
	if (Register == NULL)
		return;
	if (preScal > 255)
		return;
	uint32_t p = *Register;
	p &= ~PWM_PCR_PRESCAL;
	p |= preScal;
	*Register = p;						//Source clock Frequency Division
}

static void PWM_Out_Polarity(__IO uint32_t *Register, PWM_Out_polarity polarity)
{
	if (Register == NULL)
		return;
	if (polarity == PWM_LOWLEVE)
		HAL_CLR_BIT(*Register, PWM_PCR_ACT_STA);
	else if (polarity == PWM_HIGHLEVE)
		HAL_SET_BIT(*Register, PWM_PCR_ACT_STA);
}

static void PWM_OutModeSet(__IO uint32_t *Register, uint32_t mode)
{
	if (Register == NULL)
		return;
	if (mode == 0)
		HAL_CLR_BIT(*Register, PWM_PCR_MODE);
	else if (mode ==1)
		HAL_SET_BIT(*Register, PWM_PCR_MODE);
}

HAL_Status HAL_PWM_PluseStart(PWM_CHID ch)
{
	__IO uint32_t* Register = NULL;
	Register = PWM_GetPCRRegAdress(ch);
	if ((*Register & PWM_PCR_PLUSE_START) > 0)
		return HAL_BUSY;
	if (Register != NULL)
		HAL_SET_BIT(*Register, PWM_PCR_PLUSE_START);
	return HAL_OK;
}

int  PWM_PeriodReady(PWM_CHID ch)
{
	__IO uint32_t* Register = NULL;
	Register = PWM_GetPCRRegAdress(ch);			//Get Register Adress
	if (Register != NULL)
		return HAL_GET_BIT(*Register, PWM_PCR_PERIODRDY);
	else
		return -1;
}

typedef struct  {
	PWM_CHID ch;					//PWM Channel 0~7
	uint32_t preScal;				//value 0~255 ; Source clock division value 1~256,
									//the actual value = value + 1
	PWM_Out_polarity polarity;		//0 : low level 1: high level
}PWM_PCRX;


static void PWM_Control_CycleMode(PWM_PCRX *set)
{
	__IO uint32_t* Register = NULL;
		Register = PWM_GetPCRRegAdress(set->ch);	//Get Register Adress
		PWM_Prescal(Register, set->preScal);		//Source clock Frequency Division
		PWM_Out_Polarity(Register, set->polarity);	//Set Low level or High level
		PWM_OutModeSet(Register, 0);				//Set cycle mode or pulse mode

}

static void PWM_Control_PluseMode(PWM_PCRX *set)
{
	__IO uint32_t* Register = NULL;
		Register = PWM_GetPCRRegAdress(set->ch);	//Get Register Adress
		PWM_Prescal(Register, set->preScal);		//Source clock Frequency Division
		PWM_Out_Polarity(Register, set->polarity);	//Set Low level or High level
		PWM_OutModeSet(Register, 1);				//Set cycle mode or pulse mode

}

uint32_t HAL_PWM_GetChClkFreq(PWM_CHID ch)
{
	 __IO uint32_t* Register = NULL;
	 uint32_t srcClkDiv ,chClkDiv, srcClkFreq;

	 Register = PWM_GetPCRRegAdress(ch);	//Get Register Adress
	 chClkDiv = (*Register & PWM_PCR_PRESCAL) + 1;
	 HAL_PWM_DBG("chClkDiv %d\n", chClkDiv);
	 Register = PWM_GetPCCRAdress(PWM_ChToGroup(ch));
	 srcClkDiv = *Register & PWM_SRC_CLK_DIV_V;
	 srcClkDiv = (1 << srcClkDiv);
	 HAL_PWM_DBG("srcClkDiv %d\n", srcClkDiv);
	 if ((*Register & PWM_SRC_CLK_SELECT) == 0) {
		srcClkFreq = HAL_GetHFClock();
		HAL_PWM_DBG("srcClkFreq-HF:  %d\n", srcClkFreq);
	 } else {
	 	srcClkFreq = HAL_GetAPBClock();
		HAL_PWM_DBG("srcClkFreq-APB:  %d\n", srcClkFreq);
	 }
	 return (srcClkFreq / srcClkDiv / chClkDiv);
}

/******************************************
 *PWM Period Register:	CH 0~7
 *Register Name :	PPR
********************************************/

static __IO uint32_t *PWM_GetPPRRegAdress(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return NULL;

	return &PWM->CH_REG[ch].PPR;
}

static void PWM_SetEntierCycle(PWM_CHID ch , uint16_t entierCycle)
{
	__IO uint32_t *Register = PWM_GetPPRRegAdress(ch);
	if (Register == NULL)
		return;
	uint32_t p = *Register;
	p &= ~PWM_PPR_ENTIER_CYCLE;
	p |= (entierCycle << 16);
	*Register = p;
}

uint32_t HAL_PWM_GetEnterCycleValue(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetPPRRegAdress(ch);
	if (Register == NULL)
		return 0;
	return (*Register & PWM_PPR_ENTIER_CYCLE) >> 16;
}

static void PWM_SetActCycle(PWM_CHID ch, uint16_t actCycle)
{
	__IO uint32_t *Register = PWM_GetPPRRegAdress(ch);
	if (Register == NULL)
		return;
	uint32_t p = *Register;
	p &= ~PWM_PPR_ACT_CYCLE;
	p |= actCycle;
	*Register = p;
}

/******************************************
 *PWM Counter Register:	CH 0~7
 *Register Name :	PCNTR
********************************************/

__I uint32_t * PWM_GetPCNTRRegAdress(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return NULL;

	return &PWM->CH_REG[ch].PCNTR;
}


/******************************************
 *PWM Capture  Register:	CH 0~7
 *Register Name :	CCR
********************************************/

static __IO uint32_t *PWM_GetCCRRegAdress(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return NULL;

	return &PWM->CH_REG[ch].CCR;

}

/*return -1 error, 0 not get rising edge, 1 get rising edge*/

HAL_Status HAL_PWM_InputSignalInverse(PWM_CHID ch, PWM_FunCtrl en)
{
	__IO uint32_t *Register = PWM_GetCCRRegAdress(ch);
	if (Register == NULL)
		return HAL_ERROR;
	if (en == PWM_FUNENABLE)
	*Register |= PWM_CCR_CAPINV;
	else if (en == PWM_FUNDISABLE)
	*Register &= ~PWM_CCR_CAPINV;
	return HAL_OK;
}

int HAL_PWM_FallEdgeConterLockFlag(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetCCRRegAdress(ch);
	if (Register == NULL)
		return -1;
	return *Register & PWM_CCR_CFLF;
}

void HAL_PWM_ClearFallEdgeConterLockFlag(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetCCRRegAdress(ch);
	if (Register == NULL)
		return;
	*Register |= PWM_CCR_CFLF;
}

/*return -1 error, 0 not get rising edge, 1 get rising edge*/

int HAL_PWM_RiseEdgeConterLockFlag(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetCCRRegAdress(ch);
	if (Register == NULL)
		return -1;
	return *Register & PWM_CCR_CRLF;
}

void HAL_PWM_ClearRiseEdgeConterLockFlag(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetCCRRegAdress(ch);
	if(Register == NULL)
		return;
	*Register |= PWM_CCR_CRLF;
}

/******************************************
 *PWM Capture Rise Lock Register:	CH 0~7
 *Register Name :	CRLR
********************************************/

static __IO uint32_t *PWM_GetCRLRRegAdress(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return NULL;

	return &PWM->CH_REG[ch].CRLR;
}

uint32_t PWM_CRLRValue(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetCRLRRegAdress(ch);
	if (Register == NULL)
		return 0;
	return ((uint16_t)(*Register & PWM_CRLR));
}

/******************************************
 *PWM Capture Fall Lock Register:	CH 0~7
 *Register Name :	CRLR
********************************************/

static __IO uint32_t *PWM_GetCFLRRegAdress(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return NULL;

	return &PWM->CH_REG[ch].CFLR;
}

uint16_t PWM_CFLRValue(PWM_CHID ch)
{
	__IO uint32_t *Register = PWM_GetCFLRRegAdress(ch);
	if (Register == NULL)
		return 0;

	return ((uint16_t)(*Register & PWM_CFLR));
}

/******************************************
 *PWM Enable Register:	CH 0~7
 *Register Name :	PER
********************************************/

HAL_Status HAL_PWM_OutModeEnableCh(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return HAL_ERROR;
	PWM_ClockCtrl(ch, PWM_FUNENABLE);
	HAL_CLR_BIT(PWM->CER, HAL_BIT(ch));
	HAL_SET_BIT(PWM->PER, HAL_BIT(ch));
	return HAL_OK;
}

HAL_Status HAL_PWM_OutModeDisableCh(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return HAL_ERROR;
	PWM_ClockCtrl(ch, PWM_FUNDISABLE);
	HAL_CLR_BIT(PWM->PER, HAL_BIT(ch));
	return HAL_OK;
}

void PWM_GroupOutModeEnable(PWM_CHID ch1, PWM_CHID ch2){
	PWM_ClockCtrl(ch1, PWM_FUNENABLE);
	PWM_ClockCtrl(ch2, PWM_FUNENABLE);
	HAL_CLR_BIT(PWM->CER, HAL_BIT(ch1) | HAL_BIT(ch2));
	HAL_SET_BIT(PWM->PER, HAL_BIT(ch1) | HAL_BIT(ch2));
}

void PWM_GroupOutModeDisable(PWM_CHID ch1, PWM_CHID ch2)
{
	HAL_PWM_DBG("ch1%d ch2%d\n",ch1, ch2);
	HAL_PWM_DBG("PWM_GroupOutModeDisable\n");
	PWM_ClockCtrl(ch1, PWM_FUNDISABLE);
	PWM_ClockCtrl(ch2, PWM_FUNDISABLE);
	HAL_CLR_BIT(PWM->PER, HAL_BIT(ch1) | HAL_BIT(ch2));
}


/******************************************
 *PWM Capture Enable Register:	CH 0~7
 *Register Name :	CER
********************************************/

HAL_Status HAL_PWM_InputEnableCh(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
		return HAL_ERROR;
	PWM_ClockCtrl(ch, PWM_FUNENABLE);
	HAL_CLR_BIT(PWM->PER, HAL_BIT(ch));
	HAL_SET_BIT(PWM->CER, HAL_BIT(ch));
	return HAL_OK;
}

HAL_Status HAL_PWM_InputDisableCh(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM)
			return HAL_ERROR;
	PWM_ClockCtrl(ch, PWM_FUNENABLE);
	HAL_CLR_BIT(PWM->CER, HAL_BIT(ch));
	return HAL_OK;
}

/******************************************
 *PWM Output function:
********************************************/
uint32_t HAL_PWM_SrcClkInit(PWM_SrcClk *set)
{
	uint32_t srcClkActualFreq = 0;
	if (set->srcClk == PWM_CLK_HOSC)
		srcClkActualFreq = HAL_GetHFClock() / (1 << set->srcClkDiv);
	else if (set->srcClk == PWM_CLK_APB1)
		srcClkActualFreq =  HAL_GetAPBClock() / (1 << set->srcClkDiv);
	PWM_SrcClockInit(set);
	return srcClkActualFreq;
}

static uint32_t PWM_Out_ChClkDiv(uint32_t hz, uint32_t srcClockActualFreq, PWM_PCRX *set, uint8_t mode)
{
		uint32_t chClkFreq = 1;
		uint32_t minFreq = 0;

		uint32_t temp1 = 0, temp2 = 0;

		if ((srcClockActualFreq % MAXCNTRVAL) > 0)
			temp1 = 1;
		if (((srcClockActualFreq + temp1) % 256) > 0)
			temp2 = 1;
		minFreq = (srcClockActualFreq / MAXCNTRVAL + temp1)/ 256 + temp2;
		HAL_PWM_DBG("PWM MinFreq = %d\n", minFreq);
		if (hz > srcClockActualFreq || hz < minFreq)
			chClkFreq = 0;

		if (hz > (srcClockActualFreq / MAXCNTRVAL + temp1))
			set->preScal = 0 ;
		else {
			set->preScal =  (srcClockActualFreq / MAXCNTRVAL + temp1) % hz;
			if (set->preScal)
				set->preScal =  (srcClockActualFreq / MAXCNTRVAL + temp1) / hz;
			else
				set->preScal =  (srcClockActualFreq / MAXCNTRVAL + temp1) / hz - 1;
		}
		HAL_PWM_DBG("CH CLK DIV = %d\n", set->preScal + 1);
		if (mode == 1)
			PWM_Control_CycleMode(set);
		else if (mode == 2)
			PWM_Control_PluseMode(set);
		if (chClkFreq != 0)
			chClkFreq = srcClockActualFreq / (set->preScal + 1);
		HAL_PWM_DBG("ch clk  freq = %d \n", chClkFreq);
		return chClkFreq;
}

HAL_Status HAL_PWM_CycleModeInit(PWM_Output_Init *set)
{
	PWM_PCRX pcr_Set;
	uint32_t chEntierCycle = 0;
	pcr_Set.ch= set->ch;
	pcr_Set.polarity= set->polarity;
	chEntierCycle = PWM_Out_ChClkDiv(set->hz, set->srcClkActualFreq, &pcr_Set, 1) / set->hz;
	HAL_PWM_DBG("chEntierCycle = %d\n", chEntierCycle);
	while (PWM_PeriodReady(set->ch)== 1);
	PWM_SetEntierCycle(set->ch, chEntierCycle);
	if (chEntierCycle == 0)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_Status HAL_PWM_PluseModeInit(PWM_Output_Init *set)
{
	PWM_PCRX pcr_Set;
	uint32_t chEntierCycle = 0;
	pcr_Set.ch = set->ch;
	pcr_Set.polarity= set->polarity;
	chEntierCycle = PWM_Out_ChClkDiv(set->hz, set->srcClkActualFreq, &pcr_Set, 2) / set->hz;
	while (PWM_PeriodReady(set->ch)== 1);
	PWM_SetEntierCycle(set->ch, chEntierCycle);
	if (chEntierCycle == 0)
		return HAL_ERROR;
	return HAL_OK;
}

/*value : 0~EntierCycle; EntierCycle = PWM_GetEnterCycleValue(Ch), DutyRatio = Value/EntierCycle */
HAL_Status HAL_PWM_SetDutyRatio(PWM_CHID ch, uint16_t value)
{
	uint32_t chEntierCycle = HAL_PWM_GetEnterCycleValue(ch);
	if (value > chEntierCycle)
		return HAL_ERROR;
	while (PWM_PeriodReady(ch)== 1);
	PWM_SetActCycle(ch, value);
	return HAL_OK;
}


/*******************************************************
Complementary mode

********************************************************/
typedef struct {
	PWM_CHID ch0;
	PWM_CHID ch1;
}PWM_GroupToCh;

static PWM_GroupToCh _PWM_GroupToCh(PWM_ChGroup chGroup)
{
	PWM_GroupToCh group;
	group.ch0 = chGroup * 2;
	group.ch1 = chGroup * 2 + 1;
	return group;
}

HAL_Status HAL_PWM_ComplementaryEnable(PWM_ChGroup chGroup)
{
	if (chGroup >= PWM_GROUP_NUM)
		return HAL_ERROR;
	PWM_GroupToCh group = _PWM_GroupToCh(chGroup);
	PWM_GroupOutModeEnable(group.ch0 ,group.ch1);
	return HAL_OK;
}

HAL_Status HAL_PWM_ComplementaryDisable(PWM_ChGroup chGroup)
{
	if (chGroup >= PWM_GROUP_NUM)
		return HAL_ERROR;
	PWM_GroupToCh group = _PWM_GroupToCh(chGroup);
	PWM_GroupOutModeDisable(group.ch0 ,group.ch1);
	return HAL_OK;
}

HAL_Status HAL_PWM_ComplementaryModeInit(PWM_Complementary_Mode *set)
{
	PWM_Output_Init ch1;
	PWM_Output_Init ch2;
	PWM_GroupToCh group = _PWM_GroupToCh(set->chGroup);

	ch1.ch = group.ch0;
	ch2.ch = group.ch1;
	ch1.hz = set->hz;
	ch2.hz = set->hz;
	ch1.polarity= set->lowChPolarity;
	ch2.polarity= set->highChPolarity;
	ch1.srcClkActualFreq = set->srcClkActualFreq;
	ch2.srcClkActualFreq = set->srcClkActualFreq;
	HAL_PWM_DBG("Complementary init l_ch ch%d\n", ch1.ch);
	HAL_PWM_DBG("Complementary init h_ch ch%d\n", ch2.ch);
	if (HAL_PWM_CycleModeInit(&ch1) == HAL_ERROR) {
		HAL_PWM_DBG("Complementary init h_ch ch%d error\n", ch1.ch);
		return HAL_ERROR;
	}
	if (HAL_PWM_CycleModeInit(&ch2) == HAL_ERROR) {
		HAL_PWM_DBG("Complementary init h_ch ch%d error\n", ch2.ch);
		return HAL_ERROR;
	}
	return HAL_OK;
}

/*value : 0~EntierCycle; EntierCycle = PWM_GetEnterCycleValue(Ch), DutyRatio = Value/EntierCycle */
HAL_Status HAL_PWM_ComplementarySetDutyRatio(PWM_ChGroup chgroup, uint16_t Value)
{
	PWM_GroupToCh group = _PWM_GroupToCh(chgroup);
	if (HAL_PWM_SetDutyRatio(group.ch0, Value) == -1)
		return HAL_ERROR;
	if (HAL_PWM_SetDutyRatio(group.ch1, Value) == -1)
		return HAL_ERROR;
	return HAL_OK;
}

/************************************************
*PWM capture
*
*************************************************/

HAL_Status HAL_PWM_InputInit(PWM_Input_Init *set){
	PWM_PCRX pcr_Set;
	pcr_Set.ch = set->ch;
	if (set->chClkDiv < 1 || set->chClkDiv > 256)
		return HAL_ERROR;
	pcr_Set.preScal = set->chClkDiv - 1;
	PWM_Control_CycleMode(&pcr_Set);
	return HAL_OK;
}

PWM_squareWaveInfo HAL_PWM_CaptureResult(PWM_CHID ch)
{
	uint32_t riseTime ,fallTime;

	PWM_squareWaveInfo info = {0, 0, 0};

	if (HAL_PWM_FallEdgeConterLockFlag(ch) && HAL_PWM_RiseEdgeConterLockFlag(ch)) {
		HAL_PWM_ClearFallEdgeConterLockFlag(ch);
		HAL_PWM_ClearRiseEdgeConterLockFlag(ch);
		fallTime = PWM_CRLRValue(ch);
		riseTime = PWM_CFLRValue(ch);

		info.highLevelTime = riseTime ;
		info.lowLevelTime = fallTime ;
		info.periodTime = fallTime + riseTime;
	}
	//HAL_PWM_DBG("RiseTime %d ch =%d\n", riseTime, ch);
	//HAL_PWM_DBG("FallTime %d ch =%d\n   ", fallTime, ch);
	return info;
}

void PWM_ModuleEnable()
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_PWM);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_PWM);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_PWM);
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_PWM);
}

void PWM_ModuleDisable()
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_PWM);
}

void HAL_PWM_IO_Init(PWM_Init_Param *pwm_Param)
{
	HAL_PWM_DBG("HAL_PWM_IO_Init ch = %d\n", pwm_Param->ch);
	int isInit = PWM_IO_InitFalg & (1 << pwm_Param->ch);

	if (PWM_IO_InitFalg == 0) {
		HAL_PWM_DBG("pwm module enable\n");
		PWM_ModuleEnable();
	}

	if (!isInit) {
		pwm_Param->boardCfg(0, HAL_BR_PINMUX_INIT, (void *)(pwm_Param->ch));
		PWM_IO_InitFalg |= 1 << pwm_Param->ch;
	}
	HAL_PWM_DBG("Init PWM_IO_InitFalg%d\n", PWM_IO_InitFalg);
}
void HAL_PWM_DeInit(PWM_Init_Param *pwm_Param)
{
	int isInit = PWM_IO_InitFalg & (1 << pwm_Param->ch);
	if(isInit) {
		HAL_PWM_DBG("deinit\n");
		pwm_Param->boardCfg(0, HAL_BR_PINMUX_DEINIT, (void *)pwm_Param->ch);
		PWM_IO_InitFalg &= ~(1 << pwm_Param->ch);

		if (PWM_IO_InitFalg == 0) {
			HAL_PWM_DBG("pwm module disable\n");
			PWM_ModuleDisable();
		}
	}
	HAL_PWM_DBG("deinit PWM_IO_InitFalg%d\n", PWM_IO_InitFalg);
}


/**********************************************
*interrupt
*
***********************************************/
void HAL_PWM_ModuleIRQEnable(){
		HAL_NVIC_SetPriority(PWM_ECT_IRQn, NVIC_PERIPHERAL_PRIORITY_DEFAULT);
		HAL_NVIC_EnableIRQ(PWM_ECT_IRQn);
}

void HAL_PWM_ModuleIRQDisable(){
		HAL_NVIC_DisableIRQ(PWM_ECT_IRQn);
}

/********************Output************************/

PWM_IrqList PWM_IRQ[8] = {
					   {NULL,NULL},{NULL,NULL},{NULL,NULL},
					   {NULL,NULL},{NULL,NULL},{NULL,NULL},
					   {NULL,NULL},{NULL,NULL}
					};

void HAL_PWM_OutIRQInit(PWM_OutIRQ *set)
{
	HAL_PWM_DBG("PWMOUT IRQ num = %d\n", set->ch);
	PWM_IRQ[set->ch].arg = set->arg;
	PWM_IRQ[set->ch].callBack = set->callBack;
}

PWM_IrqList *HAL_PWM_GetIRQList(PWM_CHID ch)
{
	if (ch >= PWM_CH_NUM) {
		HAL_WARN("PWM: Can't get IRQ List, channel error\n");
		return NULL;
	}
	return &PWM_IRQ[ch];
}


void PWM_OutIRQ_CallBack()
{
	uint32_t i;
	uint32_t irqStatus;
	uint32_t isIrq;
	PWM_IrqList *priv;

	irqStatus =  PWM->PISR & PWM->PIER & PWM_IRQ_ALL_BITS; /* get pending bits */
	PWM->PISR = irqStatus; /* clear IRQ bits */
	priv = PWM_IRQ;

	for (i = PWM_GROUP0_CH0; i < PWM_CH_NUM && irqStatus != 0; i++) {
		isIrq = irqStatus & HAL_BIT(0);
		if (isIrq && priv[i].callBack) {
			PWM_IrqMode sta = PWM_IRQ_PWMOUT;
			priv[i].callBack(priv[i].arg, &sta);
		}
		irqStatus >>= 2;
	}
}

/**********Capture******************************/

void HAL_PWM_InputIRQInit(PWM_InputIRQ *set)
{
	PWM_IRQ[set->ch].arg = set->arg;
	if (PWM_IRQ[set->ch].arg == NULL)
		HAL_PWM_DBG("INPUT IRQ CH = %d ARG = NULL\n", set->ch);
	PWM_IRQ[set->ch].callBack = set->callBack;
}

void HAL_PWM_IRQDeInit(PWM_CHID ch)
{
	PWM_IRQ[ch].arg = NULL;
	PWM_IRQ[ch].callBack = NULL;
}


void PWM_Input_IRQ_CallBack()
{
	uint32_t i;
	uint32_t irqStatus;
	uint32_t isRiseEdgeIrq;
	uint32_t isFallEdgeIrq;
	PWM_IrqList *priv;
	irqStatus =  PWM->CISR & PWM->CIER & PWM_IRQ_ALL_BITS; /* get pending bits */
	PWM->CISR = irqStatus; /* clear IRQ bits */
	priv = PWM_IRQ;
	for (i = PWM_GROUP0_CH0; i < PWM_CH_NUM && irqStatus != 0; i++) {
		isRiseEdgeIrq = irqStatus & HAL_BIT(0);
		if (isRiseEdgeIrq && priv[i].callBack) {
			PWM_IrqMode sta = PWM_IRQ_RISEEDGE;
			priv[i].callBack(priv[i].arg, &sta);
		}
		isFallEdgeIrq = irqStatus & HAL_BIT(1);
		if (isFallEdgeIrq && priv[i].callBack) {
			PWM_IrqMode sta = PWM_IRQ_FALLEDGE;
			priv[i].callBack(priv[i].arg, &sta);
		}
		irqStatus >>= 2;
	}
}

void PWM_ECT_IRQHandler(){
	if (PWM->CIER > 0)
		PWM_Input_IRQ_CallBack();
	if (PWM->PIER > 0)
		PWM_OutIRQ_CallBack();
}

