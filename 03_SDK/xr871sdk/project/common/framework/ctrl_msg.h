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

#ifndef _CTRL_MSG_H_
#define _CTRL_MSG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ctrl_msg_type {
	CTRL_MSG_TYPE_SYSTEM = 0,
	CTRL_MSG_TYPE_NETWORK,
	CTRL_MSG_TYPE_VKEY,
	CTRL_MSG_VOLUME,
};

enum ctr_msg_subtype {
	CTRL_MSG_SUB_TYPE_AD_BUTTON = 0,
	CTRL_MSG_SUB_TYPE_GPIO_BUTTON,
};

struct ctrl_msg {
	uint16_t	type;
	uint16_t	subtype;
	uint32_t	data;
};

static __inline uint16_t ctrl_msg_get_type(struct ctrl_msg *msg)
{
	return msg->type;
}

static __inline uint16_t ctrl_msg_get_subtype(struct ctrl_msg *msg)
{
	return msg->subtype;
}

static __inline uint32_t ctrl_msg_get_data(struct ctrl_msg *msg)
{
	return msg->data;
}

static __inline void ctrl_msg_set(struct ctrl_msg *msg, uint16_t type,
                                  uint16_t subtype, uint32_t data)
{
	msg->type = type;
	msg->subtype = subtype;
	msg->data = data;
}

int ctrl_msg_init(uint32_t queue_len);
int ctrl_msg_deinit(void);
int ctrl_msg_send(struct ctrl_msg *msg, uint32_t wait_ms);
int ctrl_msg_recv(struct ctrl_msg *msg, uint32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* _CTRL_MSG_H_ */
