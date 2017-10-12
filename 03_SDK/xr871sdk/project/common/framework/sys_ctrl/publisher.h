/*
 * publisher.h
 *
 *  Created on: 2017Äê9ÔÂ8ÈÕ
 *      Author: lijunjie
 */

#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include "sys/list.h"
#include "kernel/os/os.h"
#include "observer.h"


typedef struct publisher_base
{
	struct list_head head;
	struct event_queue *queue;
	OS_Thread_t thd;
	OS_Mutex_t lock;	// or uint32_t sync by atomic;
	int state;

	int (*attach)(struct publisher_base *base, observer_base *obs);
	int (*detach)(struct publisher_base *base, observer_base *obs);
	int (*notify)(struct publisher_base *base, uint32_t event, uint32_t arg);
	int (*compare)(uint32_t newEvent, uint32_t obsEvent);
} publisher_base;

publisher_base *publisher_create(struct event_queue *queue, int (*compare)(uint32_t newEvent, uint32_t obsEvent),
								 OS_Priority prio, uint32_t stack);


#endif /* PUBLISHER_H_ */
