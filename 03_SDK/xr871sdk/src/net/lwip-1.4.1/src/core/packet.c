/**
 * @file
 * Implementation of packet protocol PCBs for low-level handling of
 * different types of protocols besides (or overriding) those
 * already available in lwIP.
 *
 */

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

#include "lwip/opt.h"

#if LWIP_PACKET /* don't build if not configured for use in lwipopts.h */

#include "lwip/def.h"
#include "lwip/memp.h"
#include "lwip/netif.h"
#include "lwip/packet.h"
#include "lwip/stats.h"
#include "arch/perf.h"

#include <string.h>

/** The list of packet PCBs */
static struct pkt_pcb *pkt_pcbs;

/**
 * Determine if in incoming datalink packet is covered by a packet PCB
 * and if so, pass it to a user-provided receive callback function.
 *
 * Given an incoming datalink packet (as a chain of pbufs) this function
 * finds a corresponding packet PCB and calls the corresponding receive
 * callback function.
 *
 * @param p pbuf to be demultiplexed to a packet PCB.
 * @param inp network interface on which the packet was received.
 * @param proto ethernet protocol ID, in network byte order
 *
 * Note: the packet is never eaten by a packet PCB receive callback function.
 */
void
pkt_input(struct pbuf *p, struct netif *inp, u16_t proto)
{
  struct pkt_pcb *pcb = pkt_pcbs;

  /* loop through all packet pcbs */
  /* this allows multiple pcbs to match against the packet by design */
  while (pcb != NULL) {
    if ((pcb->protocol == proto || pcb->protocol == PP_HTONS(ETHTYPE_ALL)) &&
        (pcb->netif == NULL || pcb->netif->num == inp->num)) {
      /* receive callback function available? */
      if (pcb->recv != NULL) {
        /* the receive callback function never eat the packet */
        pcb->recv(pcb->recv_arg, pcb, p);
      }
    }
    pcb = pcb->next;
  }
}

/**
 * Bind a packet PCB.
 *
 * @param pcb packet PCB to be bound with a network interface.
 * @param ifindex index of network interface to bind with. Use 0 to
 * bind to all network interfaces.
 *
 * @return lwIP error code.
 * - ERR_OK. Successful. No error occured.
 * - ERR_ARG. No matched network interface is found.
 */
err_t
pkt_bind(struct pkt_pcb *pcb, int ifindex, u16_t proto)
{
  struct netif *netif;

  if (ifindex == 0) {
    pcb->netif = NULL; /* match any network interface */
    pcb->protocol = proto;
    return ERR_OK;
  }

  for (netif = netif_list; netif != NULL; netif = netif->next) {
    if (netif->num == ifindex) {
      pcb->netif = netif;
      pcb->protocol = proto;
      return ERR_OK;
    }
  }
  return ERR_ARG;
}

/**
 * Set the callback function for received packets that match the
 * packet PCB's protocol and binding.
 */
void
pkt_recv(struct pkt_pcb *pcb, pkt_recv_fn recv, void *recv_arg)
{
  /* remember recv() callback and user data */
  pcb->recv = recv;
  pcb->recv_arg = recv_arg;
}

/**
 * Output the datalink packet through the given network interface.
 */
static LWIP_INLINE err_t
pkt_output(struct netif *netif, struct pbuf *p)
{
#if 0 /* enable to send EAPOL packet even if link is down  */
  if (!netif_is_link_up(netif)) {
    LWIP_DEBUGF(PKT_DEBUG | LWIP_DBG_LEVEL_WARNING, ("pkt_output: link down\n"));
    return ERR_RTE;
  }
#endif
  LWIP_DEBUGF(PKT_DEBUG | LWIP_DBG_TRACE, ("pkt_output: sending packet %p\n", (void *)p));

  return netif->linkoutput(netif, p); /* send the packet */
}

/**
 * Send the datalink packet through the given network interface.
 *
 * @param pcb the packet pcb which to send
 * @param p the packet data to send
 * @param ifindex index of network interface to send packet.
 * @param proto the protocol number of the ethernet frame payload.
 * @param dst the destination ethernet address to which the packet is sent
 */
err_t
pkt_sendto(struct pkt_pcb *pcb, struct pbuf *p, int ifindex, u16_t proto, u8_t *dst)
{
  err_t err;
  struct netif *netif = NULL;
  struct eth_hdr *ethhdr;
  struct pbuf *q; /* q will be sent down the stack */
  s16_t inc_len = (!pcb->is_raw) ? sizeof(struct eth_hdr) : ETH_PAD_SIZE;

  /* not enough space to add an (or part of) ethernet header to first pbuf in given p chain? */
  if (inc_len > 0 && pbuf_header(p, inc_len) != 0) {
    if (pcb->is_raw) {
      /* no enough space to add ETH_PAD_SIZE, move the ethernet header to a new pbuf */
      inc_len = sizeof(struct eth_hdr);
    }
    /* allocate header in new pbuf */
    q = pbuf_alloc(PBUF_RAW, inc_len, PBUF_RAM);
    /* new header pbuf could not be allocated? */
    if (q == NULL) {
      LWIP_DEBUGF(RAW_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS, ("pkt_sendto: could not allocate header\n"));
      return ERR_MEM;
    }
    /* move ethernet header from pbuf p to q if necessary */
    ethhdr = (struct eth_hdr *)q->payload;
    inc_len -= ETH_PAD_SIZE; /* real ethernet header size */
    if (pcb->is_raw && p->len >= inc_len) {
      SMEMCPY(&ethhdr->dest, p->payload, inc_len);
      pbuf_header(p, -inc_len);
    }
    if (p->tot_len != 0) {
      /* chain header q in front of given pbuf p */
      pbuf_chain(q, p);
    }
    /* { first pbuf q points to header pbuf } */
    LWIP_DEBUGF(RAW_DEBUG, ("pkt_sendto: added header pbuf %p before given pbuf %p\n", (void *)q, (void *)p));
  }  else {
    /* first pbuf q equals given pbuf */
    q = p;
    ethhdr = (struct eth_hdr *)q->payload;
  }

  if (!pcb->is_raw) {
    /* create ethernet header, source address will be set when sent */
    SMEMCPY(ethhdr->dest.addr, dst, ETHARP_HWADDR_LEN);
    ethhdr->type = proto;
  }

  if (pcb->netif != NULL) {
    /* bound to netif, ignore @ifindex */
    netif = pcb->netif; /* bound network interface to send packet */
    if (!pcb->is_raw) {
      /* add ethernet source address */
      SMEMCPY(ethhdr->src.addr, netif->hwaddr, ETHARP_HWADDR_LEN);
    } else {
      if (memcmp(ethhdr->src.addr, netif->hwaddr, ETHARP_HWADDR_LEN) != 0) {
        LWIP_DEBUGF(PKT_DEBUG | LWIP_DBG_LEVEL_WARNING,
          ("pkt_sendto: ethernet source address not match the bound netif\n"));
        err = ERR_RTE;
        goto out;
      }
    }
    err = pkt_output(netif, q);
    goto out;
  }

  /* not bound, iterate through netifs to find the right one */
  for (netif = netif_list; netif != NULL; netif = netif->next) {
    if (netif->name[0] == 'l' && netif->name[1] == 'o')
      continue; /* skip loopif */
    if (!pcb->is_raw) {
      if (netif->num != ifindex)
        continue;
    } else {
      if (memcmp(ethhdr->src.addr, netif->hwaddr, ETHARP_HWADDR_LEN) != 0)
        continue;
    }
    err = pkt_output(netif, q);
    goto out;
  }

  err = ERR_RTE;
  LWIP_DEBUGF(PKT_DEBUG | LWIP_DBG_LEVEL_WARNING, ("pkt_sendto: no match netif to send packet\n"));

out:
  if (q != p) {
    pbuf_free(q);
  }
  return err;
}

/**
 * Send the datalink packet through the given network interface.
 *
 * @param pcb the packet pcb which to send
 * @param p the packet data to send
 */
err_t
pkt_send(struct pkt_pcb *pcb, struct pbuf *p)
{
  if (!pcb->is_raw) {
    return ERR_ARG;
  }
  return pkt_sendto(pcb, p, -1, 0, NULL);
}

/**
 * Remove an packet PCB.
 *
 * @param pcb packet PCB to be removed. The PCB is removed from the list of
 * packet PCB's and the data structure is freed from memory.
 *
 * @see pkt_new()
 */
void
pkt_remove(struct pkt_pcb *pcb)
{
  struct pkt_pcb *pcb2;
  /* pcb to be removed is first in list? */
  if (pkt_pcbs == pcb) {
    /* make list start at 2nd pcb */
    pkt_pcbs = pkt_pcbs->next;
    /* pcb not 1st in list */
  } else {
    for(pcb2 = pkt_pcbs; pcb2 != NULL; pcb2 = pcb2->next) {
      /* find pcb in pkt_pcbs list */
      if (pcb2->next != NULL && pcb2->next == pcb) {
        /* remove pcb from list */
        pcb2->next = pcb->next;
      }
    }
  }
  memp_free(MEMP_PKT_PCB, pcb);
}

/**
 * Create a packet PCB.
 *
 * @return The packet PCB which was created. NULL if the PCB data structure
 * could not be allocated.
 *
 * @param is_raw is SOCK_RAW or not
 * @param proto the protocol number of the ethernet frame payload
 *
 * @see pkt_remove()
 */
struct pkt_pcb *
pkt_new(u8_t is_raw, u16_t proto)
{
  struct pkt_pcb *pcb;

  LWIP_DEBUGF(PKT_DEBUG | LWIP_DBG_TRACE, ("pkt_new\n"));

  pcb = (struct pkt_pcb *)memp_malloc(MEMP_PKT_PCB);
  /* could allocate packet PCB? */
  if (pcb != NULL) {
    /* initialize PCB to all zeroes */
    memset(pcb, 0, sizeof(struct pkt_pcb));
    pcb->is_raw = is_raw;
    pcb->protocol = proto;
    pcb->next = pkt_pcbs;
    pkt_pcbs = pcb;
  }
  return pcb;
}

#endif /* LWIP_PACKET */
