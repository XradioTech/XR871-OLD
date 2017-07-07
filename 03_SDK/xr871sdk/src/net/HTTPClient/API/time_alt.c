#include "kernel/os/os_time.h"
#include "HTTPClientWrapper.h"

time_t time_alt(time_t *timer)
{
	if (!timer)
		return (time_t)OS_GetTime();
	*timer = (time_t)OS_GetTime();
	return *timer;
}