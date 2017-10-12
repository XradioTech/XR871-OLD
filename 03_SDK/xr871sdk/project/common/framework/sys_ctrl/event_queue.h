/*
 * event_queue.h
 *
 *  Created on: 2017Äê9ÔÂ15ÈÕ
 *      Author: lijunjie
 */

#ifndef EVENT_QUEUE_H_
#define EVENT_QUEUE_H_

typedef struct event_msg {
//	uint16_t	type;
//	uint16_t	subtype;	/* subtype = 0 means all subtype */
	uint32_t 	event;
	uint32_t	data;
	void (*destruct)(uint32_t data);
} event_msg;

typedef struct event_queue
{
//	int (*init)(struct event_queue *base, uint32_t queue_len);
	int (*deinit)(struct event_queue *base);
	int (*send)(struct event_queue *base, struct event_msg *msg, uint32_t wait_ms);
	int (*recv)(struct event_queue *base, struct event_msg *msg, uint32_t wait_ms);
} event_queue;


struct event_queue *prio_event_queue_create(uint32_t queue_len);

struct event_queue *normal_event_queue_create(uint32_t queue_len);

#endif /* EVENT_QUEUE_H_ */
