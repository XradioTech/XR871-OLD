#ifndef _AIRKISS_ACK_H_
#define _AIRKISS_ACK_H_

#ifdef __cplusplus
extern "C" {
#endif

void airkiss_ack_start(uint32_t airkiss_random_num, struct netif *netif);
void airkiss_ack_stop();

#ifdef __cplusplus
}
#endif

#endif /*_AIRKISS_ASK_H_*/


