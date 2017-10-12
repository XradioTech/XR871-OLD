/*
 * container.h
 *
 *  Created on: 2017Äê9ÔÂ13ÈÕ
 *      Author: lijunjie
 */

#ifndef CONTAINER_H_
#define CONTAINER_H_

typedef struct container_base
{
	uint32_t size;

//	int (*init)(struct container_base *base, uint32_t config);
	int (*deinit)(struct container_base *base);
	int (*control)(struct container_base *base, uint32_t cmd, uint32_t arg);
	int (*push)(struct container_base *base, uint32_t item, uint32_t timeout);
	int (*pop)(struct container_base *base, uint32_t *item, uint32_t timeout);
} container_base;

container_base *sorted_list_create(uint32_t size, int (*compare)(uint32_t newArg, uint32_t oldArg));

#endif /* CONTAINER_H_ */
