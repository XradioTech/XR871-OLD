#include "kernel/os/os_time.h"
#include "HTTPClientWrapper.h"

time_t httpc_time(time_t *timer)
{
	if (!timer)
		return (time_t)OS_GetTime();
	*timer = (time_t)OS_GetTime();
	return *timer;
}