#ifndef PING_H
#define PING_H

#include "lwip/inet.h"
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C" {//}
#endif

struct ping_data {
   ip_addr_t sin_addr;
   u32_t count;                /* number of ping */
   u32_t data_long;          /* the ping packet data long */
};

s32_t ping(struct ping_data *data);

#ifdef __cplusplus
}
#endif

#endif
