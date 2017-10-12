/*
 * sys_ctrl.h
 *
 *  Created on: 2017Äê9ÔÂ18ÈÕ
 *      Author: lijunjie
 */

#ifndef SYS_CTRL_H_
#define SYS_CTRL_H_

#include <stdlib.h>
#include "event_queue.h"
#include "publisher.h"
#include "observer.h"

#define SYS_CTRL_PRIO_QUEUE (1)

#define ALL_SUBTYPE (0xFFFF)

typedef enum ctrl_msg_type{
	CTRL_MSG_TYPE_SYSTEM = 0,
	CTRL_MSG_TYPE_NETWORK,
	CTRL_MSG_TYPE_VKEY,
	CTRL_MSG_VOLUME,
	TEST_SYS_CTRL,
} ctrl_msg_type;

typedef enum key_msg_subtype {
	CTRL_MSG_SUB_TYPE_AD_BUTTON = 0,
	CTRL_MSG_SUB_TYPE_GPIO_BUTTON,
	CTRL_MSG_SUB_TYPE_ALL = ALL_SUBTYPE,
} key_msg_subtype;

/** @brief Get type from event */
#define EVENT_TYPE(event)  ((uint16_t)((event) >> 16))

/** @brief Get subtype from event */
#define EVENT_SUBTYPE(event)  ((uint16_t)((event) & 0xFFFF))

/** @brief Make event from type and subtype */
#define MK_EVENT(type, subtype) \
    (((uint32_t)(type) << 16) | ((uint32_t)(subtype) & 0xFFFF))

#define CMP_EVENT_TYPE(event1, event2)  ((event1 ^ event2) & 0xFFFF0000)


static __inline observer_base *sys_callback_observer_create(uint16_t type, uint16_t subtype, void (*cb)(uint32_t event, uint32_t arg))
{
	return callback_observer_create(MK_EVENT(type, subtype), cb);
}

int sys_ctrl_create(void);

int sys_ctrl_attach(observer_base *obs);

int sys_ctrl_detach(observer_base *obs);

int sys_event_send(uint16_t type, uint16_t subtype, uint32_t data, uint32_t wait_ms);

int sys_event_send_with_destruct(uint16_t type, uint16_t subtype, uint32_t data, void (*destruct)(uint32_t data), uint32_t wait_ms);

#define sys_event_send_with_free(type, subtype, data, wait_ms) \
	sys_event_send_with_destruct(type, subtype, data, (void (*)(uint32_t))free, wait_ms)


#endif /* SYS_CTRL_H_ */
