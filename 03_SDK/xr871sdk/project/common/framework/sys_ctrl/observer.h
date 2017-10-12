/*
 * observer.h
 *
 *  Created on: 2017Äê9ÔÂ8ÈÕ
 *      Author: lijunjie
 */

#ifndef OBSERVER_H_
#define OBSERVER_H_

#include <stdlib.h>
#include "sys/list.h"
#include "kernel/os/os.h"


typedef enum observer_types
{
	EVENT_OBSERVER,
	CALLBACK_OBSERVER,
	THREAD_OBSERVER,
} observer_types;

typedef enum observer_modes
{
	SINGLE_OBSERVER,
	CONTINUE_OBSERVER,
} observer_modes;

typedef struct observer_base
{
	struct list_head node;
	uint32_t event;
	int state;
//	uint32_t type;
//	observer_modes mode;

	void (*trigger)(struct observer_base *base, uint32_t event, uint32_t arg);
} observer_base;

observer_base *event_observer_create(uint32_t event);

OS_Status event_wait(observer_base *base, OS_Time_t timeout);

observer_base *callback_observer_create(uint32_t event, void (*cb)(uint32_t event, uint32_t arg));

observer_base *thread_observer_create(uint32_t event, void (*run)(uint32_t event, uint32_t arg), uint32_t stackSize, OS_Priority prio);

void thread_observer_throw(struct observer_base *base, void (*exception)(int ret));

#endif /* OBSERVER_H_ */
