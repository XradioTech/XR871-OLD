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
#ifndef BBC_MAIN_H
#define BBC_MAIN_H

typedef struct bbc_senor_upload{
	unsigned int PhotoSensit;
	float Bme280Tem;
	float Bme280Hum;
	unsigned int MotorSpeed;
	unsigned int LampRBright;
	unsigned int LampGBright;
	unsigned int LampBBright;
}senor_upload;

typedef enum {
	DEV_CALL_BACK 		= 2,
	DEV_CALL_DI_BACK	= 3
} DEV_CALL_STATUS;

typedef struct DevDataCallBack{
	unsigned char NONE_CALLBACK_FLAG;
	unsigned char RGB_R_CALLBACK_FLAG;
	unsigned char RGB_G_CALLBACK_FLAG;
	unsigned char RGB_B_CALLBACK_FLAG;
	unsigned char MOTOR_CALLBACK_FLAG;
}DEV_DATA_CALLBACK;

extern senor_upload SenorUpload;

#define RGB_CONVERT_BMSG		100		//48000 / 256
#define MOTOR_CONVERT_BMSG	240		//2400 / 10

int bbc_senor_task_init();

#endif