/*
 *  LibNoPoll: A websocket library
 *  Copyright (C) 2017 Advanced Software Production Line, S.L.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation; either version 2.1
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this program; if not, write to the Free
 *  Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307 USA
 *
 *  You may find a copy of the license under this software is released
 *  at COPYING file. This is LGPL software: you are welcome to develop
 *  proprietary applications using this library without any royalty or
 *  fee but returning back any change, improvement or addition in the
 *  form of source code, project image, documentation patches, etc.
 *
 *  For commercial support on build Websocket enabled solutions
 *  contact us:
 *
 *      Postal address:
 *         Advanced Software Production Line, S.L.
 *         Av. Juan Carlos I, Nº13, 2ºC
 *         Alcalá de Henares 28806 Madrid
 *         Spain
 *
 *      Email address:
 *         info@aspl.es - http://www.aspl.es/nopoll
 */

#include <nopoll.h>

#define LOG_DOMAIN "nopoll-FreeRTOS"

#if defined(NOPOLL_OS_FREERTOS)

noPollPtr nopoll_freertos_mutex_create(void)
{
	OS_Mutex_t *mutex  = nopoll_new(OS_Mutex_t, 1);
	if (mutex == NULL)
		return NULL;

	if (OS_MutexCreate(mutex) == OS_OK)
		return mutex;
	else
		return NULL;
}

void nopoll_freertos_mutex_destroy(noPollPtr mutex)
{
	if (OS_MutexDelete(mutex) == OS_OK)
		nopoll_free(mutex);
}

void nopoll_freertos_mutex_lock(noPollPtr mutex)
{
	OS_MutexLock(mutex, 10000);
}

void nopoll_freertos_mutex_unlock(noPollPtr mutex)
{
	OS_MutexUnlock(mutex);
}

int nopoll_freertos_gettimeofday(struct timeval *tv, noPollPtr notUsed)
{
	return gettimeofday(tv, NULL);
}

#endif
