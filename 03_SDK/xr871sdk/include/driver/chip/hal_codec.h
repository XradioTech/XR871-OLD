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

#ifndef _DRIVER_CHIP_HAL_CODEC_H_
#define _DRIVER_CHIP_HAL_CODEC_H_

#include <stdbool.h>
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_i2c.h"
#include "driver/chip/hal_audio.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct {
	GPIO_Port     ctrl_port;
	GPIO_Pin      ctrl_pin;
	GPIO_PinState ctrl_on_state;
	GPIO_PinState ctrl_off_state;
} SPK_Param;

typedef enum {
	AUDIO_DEVICE_HEADPHONE       = 1,
	AUDIO_DEVICE_SPEAKER,
	AUDIO_DEVICE_HEADPHONEMIC,
	AUDIO_DEVICE_MAINMIC,
	AUDIO_DEVICE_NONE
} AUDIO_Device;

typedef struct {
	uint8_t level;
	uint8_t reg_val;
} Volume;

typedef enum {
	VOLUME_LEVEL0,
	VOLUME_LEVEL1,
	VOLUME_LEVEL2,
	VOLUME_LEVEL3,
	VOLUME_LEVEL4,
	VOLUME_LEVEL5,
	VOLUME_LEVEL6,
	VOLUME_LEVEL7,
	VOLUME_LEVEL8,
	VOLUME_LEVEL9,
	VOLUME_LEVEL10,
	VOLUME_LEVEL11,
	VOLUME_LEVEL12,
	VOLUME_LEVEL13,
	VOLUME_LEVEL14,
	VOLUME_LEVEL15,
	VOLUME_LEVEL16,
	VOLUME_LEVEL17,
	VOLUME_LEVEL18,
	VOLUME_LEVEL19,
	VOLUME_LEVEL20,
	VOLUME_LEVEL21,
	VOLUME_LEVEL22,
	VOLUME_LEVEL23,
	VOLUME_LEVEL24,
	VOLUME_LEVEL25,
	VOLUME_LEVEL26,
	VOLUME_LEVEL27,
	VOLUME_LEVEL28,
	VOLUME_LEVEL29,
	VOLUME_LEVEL30,
	VOLUME_LEVEL31,
	VOLUME_MAX_LEVEL = VOLUME_LEVEL31,
} Vollevel;

typedef struct {
	uint8_t                     name[10];
	uint8_t                     devAddr;
	uint8_t                     RegLength;
	uint8_t                     RegValLength;
	struct codec_ops            *ops;
	struct codec_dai_ops        *dai_ops;
	struct codec_ctl_ops        *ctl_ops;
} CODEC,*CODECP;

typedef enum {
	HAL_CODEC_INIT      = 0,
	HAL_CODEC_DEINIT
} CODEC_Req;

typedef struct {
	PCM_ClkMode             clkMode;
	PCM_TranFmt             transferFormat;
	PCM_SignalInv           signalInterval;
	uint32_t                slotWidth; /*16,32,64,128,256*/
	uint32_t                wordWidth;
	uint32_t                freqIn;
	uint32_t                freqOut;
	uint32_t                pllId;
} DAI_FmtParam;

typedef struct {
	uint8_t        speaker_double_used;
	uint8_t        double_speaker_val;
	uint8_t        single_speaker_val;
	uint8_t        headset_val;
	uint8_t        mainmic_val;
	uint8_t        headsetmic_val;
} CODEC_InitParam;

typedef struct {
	uint32_t         sampleRate;
	AUDIO_Device     audioDev;
	DAI_FmtParam     *fmtParam;
} DATA_Param;

#define CODEC_I2C_REG_LENGTH8               1
#define CODEC_I2C_REGVAL_LENGTH8            1
#define CODEC_I2C_REGVAL_LENGTH16           2

#define MCLK1                               1
#define BCLK1                               2

typedef int32_t (*hw_write)(I2C_ID i2cId, uint16_t devAddr, uint8_t memAddr, uint8_t *buf, int32_t size);
typedef int32_t (*hw_read)(I2C_ID i2cId, uint16_t devAddr, uint8_t memAddr, uint8_t *buf, int32_t size);

typedef struct {
	uint8_t          *name;
	hw_write         write;
	hw_read          read;
	CODEC_InitParam  *param;
	uint8_t          i2cId;
} CODEC_Param;

struct codec_dai_ops {
	int32_t (*startup)(AUDIO_Device device);
	int32_t (*setPll)(DAI_FmtParam *fmtParam);
	int32_t (*setClkdiv)(DAI_FmtParam *fmtParam,uint32_t sampleRate);
	int32_t (*setFormat)(DAI_FmtParam *fmtParam);
	int32_t (*shutDown)(bool playOn, bool recordOn);
};

struct codec_ctl_ops {
	int32_t (*setRoute)(AUDIO_Device device);
	int32_t (*setVolume)(AUDIO_Device dev, uint32_t volume);
	int32_t (*setTrigger)(AUDIO_Device dev, uint32_t on);
};

struct codec_ops {
	int32_t (*setPower)(CODEC_Req req, void *arg);
	int32_t (*setSysClk)(CODEC_Req req, void *arg);
	int32_t (*setInitParam)(CODEC_Req req, void *arg);
	int32_t (*jackDetect)(CODEC_Req req, void *arg);
};

int32_t snd_soc_read(uint32_t reg);
int32_t snd_soc_write(uint32_t reg, uint32_t reg_val);
int32_t snd_soc_update_bits(uint32_t reg, uint32_t mask,uint32_t value);

HAL_Status HAL_CODEC_DeInit();
HAL_Status HAL_CODEC_Init(const CODEC_Param *param);
HAL_Status HAL_CODEC_Close(uint32_t dir);
HAL_Status HAL_CODEC_Open(DATA_Param *param);
HAL_Status HAL_CODEC_VOLUME_LEVEL_Set(AUDIO_Device dev,int volume);
HAL_Status HAL_CODEC_ROUTE_Set(AUDIO_Device dev);
HAL_Status HAL_CODEC_Mute(AUDIO_Device dev, uint32_t mute);
HAL_Status HAL_CODEC_Trigger(AUDIO_Device dev, uint32_t on);
uint32_t HAL_CODEC_MUTE_STATUS_Get();
HAL_Status HAL_CODEC_MUTE_STATUS_Init(int status);
HAL_Status HAL_CODEC_INIT_VOLUME_Set(AUDIO_Device dev,int volume);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_CODEC_H_ */
