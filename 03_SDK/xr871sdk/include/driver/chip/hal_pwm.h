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

#ifndef _DRIVER_CHIP_HAL_PWM_H_
#define _DRIVER_CHIP_HAL_PWM_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_clock.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	PWM_GROUP0_CH0 = 0U,		// bit 0	PORTA_PIN8 or PORTA_PIN19
	PWM_GROUP0_CH1 = 1U,		// bit 1	PORTA_PIN9 or PORTA_PIN20
	PWM_GROUP1_CH2 = 2U,		// bit 2	PORTA_PIN10 or PORTA_PIN21
	PWM_GROUP1_CH3 = 3U,		// bit 3	PORTA_PIN11 or PORTA_PIN22
	PWM_GROUP2_CH4 = 4U,		// bit 4	PORTA_PIN12 or PORTB_PIN0
	PWM_GROUP2_CH5 = 5U,		// bit 5	PORTA_PIN13 or PORTB_PIN1
	PWM_GROUP3_CH6 = 6U,		// bit 6	PORTA_PIN14 or PORTB_PIN2
	PWM_GROUP3_CH7 = 7U,		// bit 7	PORTA_PIN15 or PORTB_PIN3
	PWM_CH_NUM = 8U,
	PWM_CH_NULL = PWM_CH_NUM,
}PWM_CHID;


typedef struct {
	__IO uint32_t PCR;        		/* PWM Control Register,							Address offset: 0x60+0x0+N*0X20(N=0~7) */
	__IO uint32_t PPR;        		/* PWM Period Register,							Address offset: 0x60+0x4+N*0X20(N=0~7) */
	__I  uint32_t PCNTR;        	/* PWM Count Register,								Address offset: 0x60+0x8+N*0X20(N=0~7) */
	__IO uint32_t CCR;        		/* Capture Control Register,							Address offset: 0x60+0xC+N*0X20(N=0~7) */
	__IO uint32_t CRLR;	        	/* Capture Rise Lock Register,						Address offset: 0x60+0x10+N*0X20(N=0~7) */
	__IO uint32_t CFLR;        		/* Capture Fall Lock Register,						Address offset: 0x60+0x14+N*0X20(N=0~7) */
	 uint32_t RESERVED0[2];
}PWM_CHANNEL_REG;

typedef struct {
	__IO uint32_t PIER;        		/* PWM IRQ Enable Register,						Address offset: 0x00 */
	__IO uint32_t PISR;        		/* PWM IRQ Status Register,							Address offset: 0x04 */
		 uint32_t RESERVED0[2];				/**/
	__IO uint32_t CIER;        		/* Capture IRQ Enable Register,						Address offset: 0x10 */
	__IO uint32_t CISR;        		/* Capture IRQ Status Register,						Address offset: 0x14 */
		 uint32_t RESERVED1[2];				/**/
	__IO uint32_t PCCR[4];        	/* PWM01 Clock Configuration Register				Address offset: 0x20 */
	__IO uint32_t PDZCR[4];        	/* PWM01 Dead Zone Control Register,				Address offset: 0x30 */
	__IO uint32_t PER;        		/* PWM Enable Register,							Address offset: 0x40 */
	__IO uint32_t CER;        		/* PWM Capture Enable Register,						Address offset: 0x44 */
		 uint32_t RESERVED2[6];
	PWM_CHANNEL_REG CH_REG[PWM_CH_NUM];

} PWM_T;

#define PWM	((PWM_T *)PWM_BASE)

/*Bit definition for PWMxx(G01,G23,G45,G67) Clock Configuration register ***************/

typedef enum {
	PWM_SRC_CLK_DIV_1	=	0U, 	// Src Clk Frequency division 1
	PWM_SRC_CLK_DIV_2	=	1U, 	// Src Clk Frequency division 2
	PWM_SRC_CLK_DIV_4	=	2U,  	// Src Clk Frequency division 4
	PWM_SRC_CLK_DIV_8	=	3U,  	// Src Clk Frequency division 8
	PWM_SRC_CLK_DIV_16	=	4U,  	// Src Clk Frequency division 16
	PWM_SRC_CLK_DIV_32	=	5U,  	// Src Clk Frequency division 32
	PWM_SRC_CLK_DIV_64	=	6U,  	// Src Clk Frequency division 64
	PWM_SRC_CLK_DIV_128	=	7U,  	// Src Clk Frequency division 128
	PWM_SRC_CLK_DIV_256	=	8U,  	// Src Clk Frequency division 256
}PWM_SRC_CLK_DIV;

#define PWM_SRC_CLK_DIV_V			((uint32_t)0x0000000F)	//bit 0~3
#define PWM_CH_CLK_GATING			HAL_BIT(4) 				//bit 4
#define PWM_LOW_CH_CLKBYPASS		HAL_BIT(5)				//bit 5
#define PWM_HIGH_CH_CLKBYPASS		HAL_BIT(6)				//bit 6
#define PWM_SRC_CLK_SELECT		((uint32_t)0x000000C0)//bit 7~8, bit8 ignore , 10:APB1 00:24M

/*  Bit definition for PWMxx(G01,G23,G45,G67) Dead Zone Control register  **************/

#define PWM_CH_DZ_EN		HAL_BIT(0) 				// bit 0
#define PWM_CH_DZ_INV		((uint32_t)0x0000FF00)	// bit 8~15 PWM Dead Zone interval value

/********************  Bit definition for PWM  Control register  **********************/

#define PWM_PCR_PRESCAL			((uint32_t)0x000000FF)	// bit 0~7
#define PWM_PCR_ACT_STA			HAL_BIT(8)	// bit 8
#define PWM_PCR_MODE			HAL_BIT(9)	// bit 9
#define PWM_PCR_PLUSE_START		HAL_BIT(10)	// bit 10
#define PWM_PCR_PERIODRDY		HAL_BIT(11)	// bit 11

/***************  Bit definition for PWM  Period register  ****************************/

#define PWM_PPR_ACT_CYCLE		((uint32_t)0x0000FFFF)	// bit0~15
#define PWM_PPR_ENTIER_CYCLE	((uint32_t)0xFFFF0000)	// bit16~31

/***************  Bit definition for PWM  Counter register  ***************************/

#define PWM_TIME_COUNTER						((uint32_t)0x0000FFFF)  // bit 0~15

/***************  Bit definition for Capture  Control register  ***********************/

#define PWM_CCR_CAPINV		HAL_BIT(0) 	// bit 0
#define PWM_CCR_CFLF		HAL_BIT(1) 	// bit 1
#define PWM_CCR_CRLF		HAL_BIT(2) 	// bit 2

/***************  Bit definition for Capture  Rise Lock register  *********************/

#define PWM_CRLR		((uint32_t)0x0000FFFF)  // bit 0~15

/***************  Bit definition for Capture Fall Lock register  **********************/

#define PWM_CFLR		((uint32_t)0x0000FFFF)  // bit 0~15

/***************  Bit definition for PWM Enable register  *****************************/

#ifdef __cplusplus
}
#endif

typedef enum {
	PWM_FUNDISABLE,
	PWM_FUNENABLE,
}PWM_FunCtrl;

typedef enum {
	PWM_GROUP0,			//CH0,CH1	PORTA_PIN8, PORTA_PIN9 or PORTA_PIN19, PORTA_PIN20
	PWM_GROUP1,			//CH2,CH3	PORTA_PIN10, PORTA_PIN11 or PORTA_PIN21, PORTA_PIN22
	PWM_GROUP2,			//CH4,CH5	PORTA_PIN12, PORTA_PIN13 or PORTB_PIN0, PORTB_PIN1
	PWM_GROUP3,			//CH6,CH7	PORTA_PIN14, PORTA_PIN15 or PORTB_PIN2, PORTB_PIN3
	PWM_GROUP_NUM,
	PWM_GROUP_NULL = PWM_GROUP_NUM,
}PWM_ChGroup;

typedef enum {
	PWM_IRQ_NULL,
	PWM_IRQ_PWMOUT,
	PWM_IRQ_RISEEDGE,
	PWM_IRQ_FALLEDGE,
	PWM_IRQ_BOTHEDGE,
} PWM_IrqMode;

typedef enum {
	PWM_CLK_HOSC,
	PWM_CLK_APB1,
}PWM_SrcClock;

typedef enum {
	PWM_LOWLEVE,
	PWM_HIGHLEVE,
}PWM_Out_polarity;

typedef enum {
	PWM_RISEEDGE,
	PWM_FALLENDG,
}PWM_InputEdge;


typedef enum {
	PWM_CAP_CONTINUITY,
	PWM_CAP_ONCE,
}PWM_InputMode;

typedef struct  {
	void *arg;
	/*Arg_IRQSta : PWM_IRQMode ; When you opend both edge irq ,
	this arg is IRQSta Passing to callback , enter the IRQ it's will be set ,user  don't need set this arg*/
	void(*callBack)(void *arg, void *arg_IrqSta);
}PWM_IrqList;

typedef struct {
	PWM_ChGroup chGroup;			//PWM Pair :0~3;	pwm_01, pwm_23, pwm_45, pwm_67
	PWM_SRC_CLK_DIV srcClkDiv;		//Frequency division
	PWM_SrcClock srcClk;			//Source clock
}PWM_SrcClk;

typedef struct  {
	PWM_CHID ch;
	uint32_t hz;
	PWM_Out_polarity polarity;		//LOWLEVE  or HIGHLEVE
	uint32_t srcClkActualFreq; 		//This value get from : uint32_t HAL_PWM_PairClkInit(HAL_PWM_SrcClk *Set)
}PWM_Output_Init;

typedef struct {
	PWM_ChGroup chGroup;
	PWM_Out_polarity lowChPolarity;
	PWM_Out_polarity highChPolarity;
	uint32_t hz;
	uint32_t srcClkActualFreq;
}PWM_Complementary_Mode;

typedef struct {
	PWM_CHID ch;
	uint16_t chClkDiv;				//the value range is 1~256
	uint32_t srcClkActualFreq;
}PWM_Input_Init;

typedef struct {
	uint32_t highLevelTime;
	uint32_t lowLevelTime;
	uint32_t periodTime;
}PWM_squareWaveInfo;

typedef struct {
	PWM_CHID ch;
	void *arg;
	/*Arg_IRQSta : PWM_IRQMode ; When you  need know IRQSta ,
	this arg is IRQSta Passing to callback , enter the IRQ it's will be set ,user  don't need set this arg*/
	void(*callBack)(void *arg,  void *arg_IrqSta);
}PWM_OutIRQ;

typedef struct {
	PWM_CHID ch;
	void *arg;
	/*Arg_IRQSta : PWM_IRQMode ; When you opend both edge irq ,
	this arg is IRQSta Passing to callback , enter the IRQ it's will be set ,user  don't need set this arg*/
	void(*callBack)(void *arg, void *arg_IrqSta);
}PWM_InputIRQ;

typedef struct  {
	PWM_SrcClk srcClk;
	PWM_CHID ch;
}PWM_DirectOutputClkInit;

typedef struct {
	PWM_CHID ch;
}PWM_Init_Param;

void HAL_PWM_ModuleIRQEnable();
void HAL_PWM_ModuleIRQDisable();
void HAL_PWM_OutIRQInit(PWM_OutIRQ *set);
PWM_IrqList *HAL_PWM_GetIRQList(PWM_CHID ch);
void HAL_PWM_InputIRQInit(PWM_InputIRQ *set);
void HAL_PWM_OUT_IRQEnable(PWM_CHID ch);
void HAL_PWM_OUT_IRQDisable(PWM_CHID ch);
void HAL_PWM_InputIRQEnable(PWM_IrqMode irq,PWM_CHID ch);
void HAL_PWM_InputIRQDisable(PWM_IrqMode irq,PWM_CHID ch);
void HAL_PWM_IRQDeInit(PWM_CHID ch);

uint32_t HAL_PWM_GetChClkFreq(PWM_CHID ch);
uint32_t HAL_PWM_GetEnterCycleValue(PWM_CHID ch);
uint32_t HAL_PWM_SrcClkInit(PWM_SrcClk *set);

int HAL_PWM_FallEdgeConterLockFlag(PWM_CHID ch);
void HAL_PWM_ClearFallEdgeConterLockFlag(PWM_CHID ch);
int HAL_PWM_RiseEdgeConterLockFlag(PWM_CHID ch);
void HAL_PWM_ClearRiseEdgeConterLockFlag(PWM_CHID ch);

HAL_Status HAL_PWM_CycleModeInit(PWM_Output_Init *set);
HAL_Status HAL_PWM_PluseModeInit(PWM_Output_Init *set);
HAL_Status HAL_PWM_PluseStart(PWM_CHID ch);
HAL_Status HAL_PWM_SetDutyRatio(PWM_CHID ch, uint16_t value);
HAL_Status HAL_PWM_OutModeEnableCh(PWM_CHID ch);
HAL_Status HAL_PWM_OutModeDisableCh(PWM_CHID ch);

HAL_Status HAL_PWM_ComplementaryModeInit(PWM_Complementary_Mode *set);
HAL_Status HAL_PWM_ComplementaryEnable(PWM_ChGroup chGroup);
HAL_Status HAL_PWM_ComplementaryDisable(PWM_ChGroup chGroup);
HAL_Status HAL_PWM_ComplementarySetDutyRatio(PWM_ChGroup chGroup, uint16_t value);
HAL_Status HAL_PWM_DeadZoneEnable(PWM_ChGroup chGroup);
HAL_Status HAL_PWM_DeadZoneDisable(PWM_ChGroup chGroup);
HAL_Status HAL_PWM_SetDeadZoneTime(PWM_ChGroup chGroup, uint8_t deadZoneTime);

HAL_Status HAL_PWM_InputInit(PWM_Input_Init *set);
HAL_Status HAL_PWM_InputEnableCh(PWM_CHID ch);
HAL_Status HAL_PWM_InputDisableCh(PWM_CHID ch);
HAL_Status HAL_PWM_InputSignalInverse(PWM_CHID ch, PWM_FunCtrl en);
PWM_squareWaveInfo HAL_PWM_CaptureResult(PWM_CHID ch);


void HAL_PWM_IO_Init(PWM_Init_Param *pwm_Param);
void HAL_PWM_DeInit(PWM_Init_Param *pwm_Param);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_PWM_H_ */

