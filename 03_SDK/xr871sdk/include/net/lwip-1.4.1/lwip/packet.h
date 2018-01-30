/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __LWIP_PACKET_H__
#define __LWIP_PACKET_H__

#include "opt.h"

#if LWIP_PACKET /* don't build if not configured for use in lwipopts.h */

#include "pbuf.h"
#include "def.h"
#include "netif/etharp.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pkt_pcb;

/** Function prototype for packet pcb receive callback functions.
 * @param arg user supplied argument (pkt_pcb.recv_arg)
 * @param pcb the pkt_pcb which received data
 * @param p the packet buffer that was received
 * @param addr the remote ethernet address from which the packet was received
 *
 * Note: the packet is never eaten by a packet PCB receive callback function.
 */
typedef void (*pkt_recv_fn)(void *arg, struct pkt_pcb *pcb, struct pbuf *p);

struct pkt_pcb {
  struct pkt_pcb *next;

  /* network interface which was bound with */
  struct netif *netif;

  /* SOCK_RAW or not */
  u8_t is_raw;

  /* ethernet protocol ID, in network byte order */
  u16_t protocol;

  /** receive callback function */
  pkt_recv_fn recv;
  /* user-supplied argument for the recv callback */
  void *recv_arg;
};

/* The following functions is the application layer interface to the
   PACKET code. */
struct pkt_pcb * pkt_new        (u8_t is_raw, u16_t proto);
void             pkt_remove     (struct pkt_pcb *pcb);
err_t            pkt_bind       (struct pkt_pcb *pcb, int ifindex, u16_t proto);

void             pkt_recv       (struct pkt_pcb *pcb, pkt_recv_fn recv, void *recv_arg);
err_t            pkt_sendto     (struct pkt_pcb *pcb, struct pbuf *p, int ifindex, u16_t proto, u8_t *dst);
err_t            pkt_send       (struct pkt_pcb *pcb, struct pbuf *p);

/* The following functions are the lower layer interface to PACKET. */
void             pkt_input      (struct pbuf *p, struct netif *inp, u16_t proto);
#define pkt_init() /* Compatibility define, not init needed. */

#ifdef __cplusplus
}
#endif

#endif /* LWIP_PACKET */

#endif /* __LWIP_PACKET_H__ */
