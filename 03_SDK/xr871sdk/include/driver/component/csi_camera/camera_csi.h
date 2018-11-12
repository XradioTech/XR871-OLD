
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

#ifndef __CAMERA_CSI_H__
#define __CAMERA_CSI_H__

typedef struct {
	GPIO_Port Cam_Reset_Port;
	GPIO_Pin Cam_Reset_Pin;
	GPIO_Port Cam_Pwdn_Port;
	GPIO_Pin Cam_Pwdn_Pin;
}Cam_PowerCtrlCfg;


typedef enum {
	LIGHT_AUTO,
	LIGHT_SUNNY,
	LIGHT_COLUDY,
	LIGHT_OFFICE,
	LIGHT_HOME,
} CAM_LIGHT_MODE;


typedef enum {
	COLOR_SATURATION_0, /*!< -2 */
	COLOR_SATURATION_1, /*!< -1*/
	COLOR_SATURATION_2, /*!< The default */
	COLOR_SATURATION_3, /*!< 1 */
	COLOR_SATURATION_4, /*!< 2 */
} CAM_COLOR_SATURATION;


typedef enum {
	BRIGHT_0,  /*!< birghtness -2 */
	BRIGHT_1,  /*!< -1 */
	BRIGHT_2,  /*!< The default */
	BRIGHT_3,  /*!< 1 */
	BRIGHT_4,  /*!< 2 */
} CAM_BRIGHTNESS;

typedef enum {
	CONTARST_0, /*!< -2 */
	CONTARST_1, /*!< -1 */
	CONTARST_2, /*!< The default */
	CONTARST_3, /*!< 1 */
	CONTARST_4, /*!< 2 */
} CAM_CONTARST;

/**
  * @brief effects.
  */
typedef enum {
	IMAGE_NOMAL,
	IMAGE_NEGATIVE,
	IMAGE_BLACK_WHITE,
	IMAGE_SLANT_RED,
	IMAGE_SLANT_GREEN,
	IMAGE_SLANT_BLUE,
	IMAGE_VINTAGE,
} CAM_SPECAIL_EFFECTS;


#endif





