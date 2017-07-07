#ifndef PING_H
#define PING_H

#include "lwip/inet.h"
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C" {//}
#endif
#define PING_TO		5000    /* timeout to wait every reponse(ms) */
//#define PING_TO		1000    /* timeout to wait every reponse(ms) */
#define PING_ID		0xABCD
#define PING_ID2		0xBBCD
#define PING_DATA_SIZE	100     /* size of send frame buff, not include ICMP frma head */

struct ping_data {
   ip_addr_t sin_addr;
   u32_t count;                /* number of ping */
};

#define GET_TICKS	OS_GetTicks
s32_t ping(struct ping_data *data);

#ifdef __cplusplus
}
#endif

#endif
