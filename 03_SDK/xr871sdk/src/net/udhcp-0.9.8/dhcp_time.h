#ifndef DHCP_TMER_H_H
#define DHCP_TMER_H_H

#ifdef DHCPD_TIMEALT
#include <time.h>
time_t time_alt(time_t *timer);

#define time time_alt

#endif

#endif
