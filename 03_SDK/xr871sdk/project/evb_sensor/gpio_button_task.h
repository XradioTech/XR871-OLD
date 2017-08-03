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

#ifndef _BUTTON_TASK_H_
#define _BUTTON_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	GPIO_BUTTON_CMD_LONG_PRESS,
	GPIO_BUTTON_CMD_SHORT_PRESS,
	GPIO_BUTTON_CMD_REPEAT,
	GPIO_BUTTON_CMD_RELEASE,
	GPIO_BUTTON_CMD_NULL,
}GPIO_BUTTON_CMD;

typedef enum {
	GPIO_BUTTON_0,
	GPIO_BUTTON_1,
	GPIO_BUTTON_2, //button0 + button1 combonation button, don't have cb func
	GPIO_BUTTON_NUM,
}GPIO_BUTTON_ID;

typedef struct {
	GPIO_BUTTON_CMD cmd;
	GPIO_BUTTON_ID id;
}GPIO_Button_Cmd_Info;

void gpio_button_ctrl_init(void);
void gpio_button_ctrl_deinit(void);
void gpio_button_ctrl(GPIO_Button_Cmd_Info *button_cmd);

#ifdef __cplusplus
}
#endif

#endif /* _BUTTON_TASK_H_ */

