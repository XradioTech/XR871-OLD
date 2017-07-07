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

#ifndef _AD_BUTTON_TASK_H_
#define _AD_BUTTON_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	AD_BUTTON_CMD_LONG_PRESS,
	AD_BUTTON_CMD_SHORT_PRESS,
	AD_BUTTON_CMD_REPEAT,
	AD_BUTTON_CMD_RELEASE,
	AD_BUTTON_CMD_NULL,
}AD_BUTTON_CMD;

typedef enum {
	AD_BUTTON_0,
	AD_BUTTON_1,
	AD_BUTTON_NUM,
	AD_BUTTON_ALL_RELEASE,
}AD_BUTTON_ID;

typedef struct {
	AD_BUTTON_CMD cmd;
	AD_BUTTON_ID id;
}AD_Button_Cmd_Info;


void ad_button_init();
void ad_button_deinit();
void ad_button_ctrl(AD_Button_Cmd_Info *button_cmd);

#ifdef __cplusplus
}
#endif

#endif /* _AD_BUTTON_TASK_H_ */


