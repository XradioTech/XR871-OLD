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

#include "../hal_base.h"
#include "codec.h"
#include "hal_ac101s.h"

#define AC101S_DBG_ON               1

#if (AC101S_DBG_ON == 1)
#define AC101S_DEBUG(fmt, arg...)   HAL_LOG(AC101S_DBG_ON, "[AC101S] "fmt, ##arg)
#else
#define AC101S_DEBUG(fmt, arg...)
#endif
#define AC101S_ERROR(fmt, arg...)   HAL_LOG(1, "[AC101S] "fmt, ##arg)

#define SYSCLK_11M                  (11289600U)
#define SYSCLK_12M                  (12288000U)
#define AC101S_VOLUME_MAX_LEVEL     VOLUME_LEVEL16

typedef enum {
	SYSCLK_DIV_1   = 1,
	SYSCLK_DIV_2   = 2,
	SYSCLK_DIV_3   = 3,
	SYSCLK_DIV_4   = 4,
	SYSCLK_DIV_6   = 6,
	SYSCLK_DIV_8   = 8,
	SYSCLK_DIV_12  = 12,
	SYSCLK_DIV_16  = 16,
	SYSCLK_DIV_24  = 24,
} SYS_CLKDIV;

typedef enum {
	EQ_TYPE_0  = 0,
	EQ_TYPE_1,
	EQ_TYPE_NULL,
} EQ_TYPE;

typedef struct {
	uint8_t regVal;
	SYS_CLKDIV clkDiv;
} CLK_DIVRegval;

typedef struct {
	uint32_t band_b0;
	uint32_t band_b1;
	uint32_t band_b2;
	uint32_t band_a1;
	uint32_t band_a2;
} band_coef;

typedef struct {
	band_coef band_1;
	band_coef band_2;
	band_coef band_3;
} EQ_Band;

typedef struct {
	EQ_TYPE scene;
	EQ_Band band;
} EQ_Scene;

static const CLK_DIVRegval DivRegval[] = {
		{0, SYSCLK_DIV_1},
		{1, SYSCLK_DIV_2},
		{2, SYSCLK_DIV_3},
		{3, SYSCLK_DIV_4},
		{4, SYSCLK_DIV_6},
		{5, SYSCLK_DIV_8},
		{6, SYSCLK_DIV_12},
		{7, SYSCLK_DIV_16},
		{8, SYSCLK_DIV_24},
};

static const EQ_Scene EqScene[] = {
		{EQ_TYPE_0, {{0x10000, 0x0, 0x0, 0x0, 0x0}, {0x010000, 0x0, 0x0, 0x0, 0x0}, {0x010000, 0x0, 0x0, 0x0, 0x0}}},
		/* BQ1: G=10, Fc=200, Q=20; BQ2: G=0, Fc=1000, Q=1; BQ3: G=0, Fc=10000, Q=1 */
		{EQ_TYPE_1, {{0x10034, 0xFFFE005E, 0xFF9B, 0xFFFE005E, 0xFFFCF}, {0x10000, 0xFFFE237B, 0xE0A1, 0xFFFE237B, 0xE0A1}, {0x010000, 0xFFFFA6A5, 0x5941, 0xFFFFA6A5, 0x5941}}},
};

static CODEC_MIC_Type mic_type = CODEC_MIC_ANALOG;
static uint8_t dac_mix = 0x03;
static uint8_t lout_auto_att = 0;

/*
 * Set speaker as the current output device.
 */
static void AC101S_SetSpeaker()
{
	AC101S_DEBUG("Route(PLAY): speaker..\n");
#ifdef DAC_ANA_REG25_EN
	/*enable DAC digital part*/
	snd_soc_update_bits(DAC_DIG_CTRL, (0x1<<DVCZCDEN)|(0x7<<DITHER_SGM)|
							(0x3<<DAC_PTN_SEL)|(0x1<<DAC_DIG_EN),
							(0x1<<DVCZCDEN)|(0x4<<DITHER_SGM)|(0x1<<DAC_DIG_EN));
	/*enable DAC analog part*/
	snd_soc_update_bits(DAC_ANA_CTRL1, (0x1<<DACEN)|(0X1<<VRDA_EN)|(0x1<<RSWITCH)|
	                         (0X1<<LOMUTE),(0x1<<DACEN)|(0X1<<VRDA_EN)|(0x1<<RSWITCH)|
	                         (0X1<<LOMUTE));
	/*enable LOUT*/
	snd_soc_update_bits(DAC_ANA_CTRL2, (0x1<<LINEOUTEN)|(0X1<<LINEODIFEN),
	                         (0x1<<LINEOUTEN)|(0X1<<LINEODIFEN));
#else
	/*reset*/
	//snd_soc_update_bits(CHIP_SOFT_RST, (0xFF<<0), (0x34<<0));

	/*disenable LOUT*/
	snd_soc_update_bits(SYS_FUNC_CTRL, (0x1<<DAC_ANA_OUT_EN),(0x0<<DAC_ANA_OUT_EN));

	/*set DAC mixer source*/
	snd_soc_update_bits(DAC_MIX_SR, (0x3<<DAC_MIX_SRC), (dac_mix<<DAC_MIX_SRC));

	snd_soc_update_bits(DAC_ANA_CTRL4, (0x1<<LOUTAUTOATT), (lout_auto_att<<LOUTAUTOATT));

	/*enable DAC digital part*/
	snd_soc_update_bits(DAC_DIG_CTRL, (0x1<<DVCZCDEN)|(0x7<<DITHER_SGM)|
							(0x3<<DAC_PTN_SEL)|(0x1<<DAC_DIG_EN),
							(0x1<<DVCZCDEN)|(0x4<<DITHER_SGM)|(0x1<<DAC_DIG_EN));

	/*enable DAC analog part*/
	snd_soc_update_bits(SYS_FUNC_CTRL, (0x1<<DAC_PLAY_FUNC_EN)|
							(0x1<<ADC_REC_FUNC_EN)|(0x1<<DAC_ANA_OUT_EN),
							(0x1<<DAC_PLAY_FUNC_EN)|(0x1<<ADC_REC_FUNC_EN)|
							(0x1<<DAC_ANA_OUT_EN));

	/*enable LOUT differential mode*/
	snd_soc_update_bits(DAC_ANA_CTRL2, (0x1<<LINEODIFEN), (0x0<<LINEODIFEN));

#endif
}

/*
 * Set main mic as the current input device.
 */
static void AC101S_SetMainMic()
{
	AC101S_DEBUG("Route(cap): main mic..\n");
#ifdef WAIT_MABIA_STABLE
	/*wait for voltage stable*/
	snd_soc_update_bits(PWR_CTRL2, (0x1<<MBIAS_EN), (0x1<<MBIAS_EN));
	snd_soc_update_bits(ADC_ANA_CTRL1, (0X1<<ADC_GEN),(0X1<<ADC_GEN));
	HAL_MSleep(200);
#endif
	/*set MBIAS vol*/
	snd_soc_update_bits(PWR_CTRL1, (0x3<<MBIAS_VCTRL), (0x2<<MBIAS_VCTRL));

	/*enable MBIAS*/
	snd_soc_update_bits(PWR_CTRL2, (0x1<<ALDO_EN)|(0x1<<DLDO_EN)|(0x1<<MBIAS_EN)|
							(0x1<<VREF_EN)|(0x1<<IREF_EN), (0x1<<ALDO_EN)|(0x1<<DLDO_EN)|
							(0x1<<MBIAS_EN)|(0x1<<VREF_EN)|(0x1<<IREF_EN));

	/*enable ADC analog part*/
	snd_soc_update_bits(ADC_ANA_CTRL1, (0X1<<ADC_GEN),(0X1<<ADC_GEN));

	/*enable HPF*/
	snd_soc_update_bits(AGC_CTRL, (0X1<<HPF_EN)|(0X2<<AGC_HYS_SET),
	                         (0X1<<HPF_EN)|(0X1<<AGC_HYS_SET));

	/*enable ADC digital part*/
	snd_soc_update_bits(ADC_DIG_CTRL, (0x2<<ADOUT_DTS)|(0X1<<ADOUT_DLY_EN)|(0x1<<ADC_DIG_EN),
							(0x2<<ADOUT_DTS)|(0X1<<ADOUT_DLY_EN)|(0x1<<ADC_DIG_EN));
}

/*
 * Set digital mic as the current input device.
 */
static void AC101S_SetDigitalMic()
{
	AC101S_DEBUG("Route(cap): digital mic..\n");

	/*enable MBIAS*/
	snd_soc_update_bits(PWR_CTRL2, (0x1<<ALDO_EN)|(0x1<<DLDO_EN)|(0x1<<MBIAS_EN)|
							(0x1<<VREF_EN)|(0x1<<IREF_EN), (0x1<<ALDO_EN)|(0x1<<DLDO_EN)|
							(0x1<<MBIAS_EN)|(0x1<<VREF_EN)|(0x1<<IREF_EN));
	/*enable HPF*/
	snd_soc_update_bits(AGC_CTRL, (0X1<<HPF_EN)|(0X2<<AGC_HYS_SET),
	                         (0X1<<HPF_EN)|(0X1<<AGC_HYS_SET));

	/*enable ADC digital part*/
	snd_soc_update_bits(ADC_DIG_CTRL, (0x1<<DIG_MIC_EN)|(0X1<<ADC_DIG_EN),
							(0x1<<DIG_MIC_EN)|(0X1<<ADC_DIG_EN));
}

/*
 * Set audio output/input device.
 */
static int32_t AC101S_SetRoute(AUDIO_Device device, CODEC_DevState state)
{
	switch (device) {
		case AUDIO_OUT_DEV_SPEAKER:
			AC101S_SetSpeaker();
			break;
		case AUDIO_IN_DEV_MAINMIC:
			if(mic_type == CODEC_MIC_ANALOG)
				AC101S_SetMainMic();
			else
				AC101S_SetDigitalMic();
			break;
		default:
			break;
	}
	return HAL_OK;
}

static int32_t AC101S_SetVolume(AUDIO_Device dev, uint8_t volume)
{
	AC101S_DEBUG("[set volume] dev(%d) volume(%d)..\n", (int)dev, (int)volume);
	if (volume > AC101S_VOLUME_MAX_LEVEL) {
		AC101S_DEBUG("[set volume] Wrong volume..\n");
		return HAL_INVALID;
	}

	uint32_t volume_val = volume;
	if (volume_val == 0) {
		snd_soc_update_bits(DAC_DVC, (0xFF<<DAC_DVC_VOL), (0x0<<DAC_DVC_VOL));
	} else {
		switch (dev) {
			case AUDIO_OUT_DEV_SPEAKER:
				snd_soc_update_bits(DAC_DVC, (0xFF<<DAC_DVC_VOL), (0x81<<DAC_DVC_VOL));
				snd_soc_update_bits(DAC_ANA_CTRL2, (0xF<<LINEOAMPGAIN), ((0x10 - volume_val)<<LINEOAMPGAIN));
				break;
			default:
				AC101S_DEBUG("[set volume] Wrong audio out device..\n");
				return HAL_INVALID;
		}
	}

	return HAL_OK;
}

static int32_t AC101S_SetTrigger(AUDIO_Device dev, uint8_t on)
{
	return HAL_OK;
}

static int32_t AC101S_Ioctl(AUDIO_Device dev, CODEC_ControlCmd cmd, uint32_t arg)
{
	int ret = -1;

	if (AUDIO_IN_DEV_MAINMIC != dev && AUDIO_OUT_DEV_SPEAKER != dev)
		return ret;

	switch (cmd) {
		case CODEC_CMD_SET_MIXSER:
		{
			CODEC_ROUTE_Mixser *mix = (CODEC_ROUTE_Mixser *)arg;
			if (mix == NULL)
				return ret;
			dac_mix = mix->dac_mix ? 0x03 : 0x01;
			ret = 0;
			break;
		}
		case CODEC_CMD_SET_AUTO_ATT:
			if (arg == 0)
				lout_auto_att = 0;
			else
				lout_auto_att = 1;
			ret = 0;
		default :
			break;
	}

	return ret;
}

static int32_t AC101S_SetEqScene(uint8_t scene)
{
	const EQ_Scene *eqScene = EqScene;

	do {
	   if (eqScene->scene == scene) {
	   		AC101S_DEBUG("[set EqScene] scene(%u)..\n", scene);
	   		uint8_t i, j, reg_base;
			const uint32_t *reg_val = &eqScene->band.band_1.band_b0;
	   		for (i = 0, reg_base = EQ1_B0_H; i < 3; i++, reg_base += 0x10) {
				for (j = 0; j < 5; j++) {
					snd_soc_update_bits(reg_base++, (0x7<<0), (((*reg_val >> 16) & 0x7)<<0));
					snd_soc_update_bits(reg_base++, (0xFF<<0), (((*reg_val >> 8) & 0xFF)<<0));
					snd_soc_update_bits(reg_base++, (0xFF<<0), ((*reg_val & 0xFF)<<0));
					reg_val++;
				}
			}
			break;
	   }
		if (eqScene->scene == (EQ_TYPE_NULL - 1))
			break;

	   eqScene++;
	} while (eqScene->scene < EQ_TYPE_NULL);
	snd_soc_update_bits(EQ_CTRL, (0x01<<EQ_EN), ((scene == 0 ? 0 :1)<<EQ_EN));

	return HAL_OK;
}

static int32_t AC101S_SetPll(const DAI_FmtParam *fmtParam)
{
	return HAL_OK;
}

static int32_t AC101S_SetClkdiv(const DAI_FmtParam *fmtParam,uint32_t sampleRate)
{
	uint8_t nadc, ndac;
	uint32_t sysclk;
	sysclk = ((sampleRate % 1000) != 0) ? SYSCLK_11M : SYSCLK_12M;
	nadc = ndac = sysclk / (128 * sampleRate);

	const CLK_DIVRegval *divRegval = DivRegval;
	do {
	   if (divRegval->clkDiv == nadc) {
			snd_soc_update_bits(ADC_CLK_SET, (0xF<<NADC), (divRegval->regVal<<NADC));
			snd_soc_update_bits(DAC_CLK_SET, (0xF<<NDAC), (divRegval->regVal<<NDAC));
			break;
	   }
	   divRegval++;
	} while (divRegval->clkDiv < SYSCLK_DIV_24);

	return HAL_OK;
}

static int32_t AC101S_SetFotmat(const DAI_FmtParam *fmtParam)
{
	uint8_t slot_width, word_width;
	slot_width = (fmtParam->slotWidth - 4) / 4;
	word_width = (fmtParam->wordWidth - 4) / 4;

	if (slot_width > 7 || word_width > 7)
		return HAL_INVALID;

	snd_soc_update_bits(I2S_LRCK_CTRL2, (0xFF<<LRCK_PERIODL),(0x1F<<LRCK_PERIODL));
	snd_soc_update_bits(I2S_FMT_CTRL2, (0x7<<SR)|(0x7<<SW),(word_width<<SR)|(slot_width<<SW));

	return HAL_OK;
}

static int32_t AC101S_ShutDown(bool playOn, bool recordOn)
{
	if (playOn == 0 && recordOn == 0) {
		snd_soc_update_bits(DAC_DIG_CTRL, (0x1<<DAC_DIG_EN), (0x0<<DAC_DIG_EN));
		#ifdef DAC_ANA_REG25_EN
		snd_soc_update_bits(DAC_ANA_CTRL1, (0x1<<DACEN)|(0X1<<LOMUTE),
								(0x0<<DACEN)|(0X0<<LOMUTE));
		#else
		/*enable DAC analog part*/
		snd_soc_update_bits(SYS_FUNC_CTRL, (0x1<<DAC_PLAY_FUNC_EN)|
								(0x1<<ADC_REC_FUNC_EN)|(0x1<<AGC_GEN)|(0x1<<DAC_ANA_OUT_EN),
								(0x1<<DAC_PLAY_FUNC_EN)|(0x1<<ADC_REC_FUNC_EN)|
								(0x1<<AGC_GEN)|(0x0<<DAC_ANA_OUT_EN));
		#endif

		snd_soc_update_bits(PWR_CTRL2, (0x1<<MBIAS_EN), (0x0<<MBIAS_EN));
		snd_soc_update_bits(ADC_DIG_CTRL, (0x1<<ADC_DIG_EN),(0x0<<ADC_DIG_EN));
		snd_soc_update_bits(ADC_ANA_CTRL1, (0X1<<ADC_GEN), (0X0<<ADC_GEN));

	} else if (playOn == 0) {
		dac_mix = 0x03;
		snd_soc_update_bits(DAC_DIG_CTRL, (0x1<<DAC_DIG_EN), (0x0<<DAC_DIG_EN));
		#ifdef DAC_ANA_REG25_EN
		snd_soc_update_bits(DAC_ANA_CTRL1, (0x1<<DACEN)|(0X1<<LOMUTE),
								(0x0<<DACEN)|(0X0<<LOMUTE));
		#else
		/*enable DAC analog part*/
		snd_soc_update_bits(SYS_FUNC_CTRL, (0x1<<VREF_SPUP_STA)|(0x1<<DAC_PLAY_FUNC_EN)|
								(0x1<<ADC_REC_FUNC_EN)|(0x1<<AGC_GEN)|(0x1<<DAC_ANA_OUT_EN),
								(0x0<<VREF_SPUP_STA)|(0x1<<DAC_PLAY_FUNC_EN)|(0x1<<ADC_REC_FUNC_EN)|
								(0x1<<AGC_GEN)|(0x0<<DAC_ANA_OUT_EN));
		#endif
	} else if (recordOn == 0) {
		snd_soc_update_bits(PWR_CTRL2, (0x1<<MBIAS_EN), (0x0<<MBIAS_EN));
		snd_soc_update_bits(ADC_DIG_CTRL, (0x1<<ADC_DIG_EN),(0x0<<ADC_DIG_EN));
		snd_soc_update_bits(ADC_ANA_CTRL1, (0X1<<ADC_GEN), (0X0<<ADC_GEN));
	} else {
		return HAL_INVALID;
	}
	return HAL_OK;
}

static int32_t AC101S_SetPower(CODEC_Req req, void *arg)
{
	return HAL_OK;
}

static int32_t AC101S_SetSysClk(CODEC_Req req, void *arg)
{
	return HAL_OK;
}

static int32_t AC101S_SetInitParam(CODEC_Req req, void *arg)
{
	snd_soc_update_bits(SYS_FUNC_CTRL, (0x1<<DAC_ANA_OUT_EN),(0x0<<DAC_ANA_OUT_EN));
	snd_soc_update_bits(CHIP_SOFT_RST, (0xFF<<0), (0x34<<0));

	if (req == HAL_CODEC_INIT) {
		const CODEC_HWParam *param = (const CODEC_HWParam *)arg;
		if (param) {
			mic_type = param->mainmic_type;
			snd_soc_update_bits(ADC_ANA_CTRL1, (0x1F<<PGA_GAIN_CTRL),
                         (param->mainmic_analog_val<<PGA_GAIN_CTRL));
			snd_soc_update_bits(ADC_DVC, (0xFF<<ADC_DVC_VOL), (param->mainmic_digital_val<<ADC_DVC_VOL));
		}
	}

	return HAL_OK;
}

static int32_t AC101S_JackDetect(CODEC_Req req, void *arg)
{
	return HAL_OK;
}

const struct codec_ctl_ops ac101s_ctl_ops =  {
    .setRoute       = AC101S_SetRoute,
    .setVolume      = AC101S_SetVolume,
    .setTrigger     = AC101S_SetTrigger,
    .setEqScene     = AC101S_SetEqScene,
    .ioctl          = AC101S_Ioctl,
};

const struct codec_dai_ops ac101s_dai_ops =  {
    .setPll     = AC101S_SetPll,
    .setClkdiv  = AC101S_SetClkdiv,
    .setFormat  = AC101S_SetFotmat,
    .shutDown   = AC101S_ShutDown,
};

const struct codec_ops ac101s_ops =  {
    .setPower       = AC101S_SetPower,
    .setSysClk      = AC101S_SetSysClk,
    .setInitParam   = AC101S_SetInitParam,
    .jackDetect     = AC101S_JackDetect,
};

const CODEC AC101S = {
    .type           = AUDIO_CODEC_AC101S,
    .RegLength      = CODEC_I2C_REG_LENGTH8,
    .RegValLength   = CODEC_I2C_REGVAL_LENGTH8,
    .ops            = &ac101s_ops,
    .dai_ops        = &ac101s_dai_ops,
    .ctl_ops        = &ac101s_ctl_ops,
};
