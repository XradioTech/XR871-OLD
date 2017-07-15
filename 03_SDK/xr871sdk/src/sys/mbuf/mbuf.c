/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mbuf_util.h"
#include "sys/mbuf.h"

#if (!defined(__CONFIG_ARCH_DUAL_CORE) || defined(__CONFIG_ARCH_APP_CORE))

#include "lwip/pbuf.h"
#include "lwip/mem.h"
#include "lwip/memp.h"

/* Init mbuf data info from pbuf, no sanity checks */
#define MBUF_DATA_INIT(m, p)						\
	do {								\
		(m)->m_pbuf = (p);					\
		(m)->m_data = (p)->payload;				\
		(m)->m_len = (p)->tot_len;				\
		(m)->m_pkthdr.len = (p)->tot_len;			\
		(m)->m_headspace = pbuf_head_space(p);			\
		if ((p)->mb_flags & PBUF_FLAG_MBUF_SPACE)		\
			(m)->m_tailspace = LWIP_MBUF_TAIL_SPACE;	\
		(p)->mb_flags |= PBUF_FLAG_MBUF_REF;			\
	} while (0)


/*
 * Alloc a new mbuf without pbuf, zero the header.
 */
static __inline struct mbuf *mb_alloc()
{
	struct mbuf *m = memp_malloc(MEMP_MBUF);
	if (m) {
		MB_MEMSET(m, 0, sizeof(struct mbuf));
		m->m_flags = M_PKTHDR;
	} else {
		MBUF_ERROR("Out of memory in pbuf pool for mbuf!\n");
	}
	return m;
}

/*
 * @tx: 1 means mbuf is used to do Tx, always alloc it from PBUF_RAM.
 * @return a mbuf including @len data
 */
struct mbuf *mb_get(int len, int tx)
{
	if (len < 0)
		return NULL;

	if (len > PBUF_POOL_BUFSIZE) {
		MBUF_ERROR("Why you need so many data? len %d\n", len);
	}

	struct mbuf *m = mb_alloc();
	if (m == NULL)
		return NULL;

	struct pbuf *p;
#if LWIP_PBUF_POOL_SMALL
	int pbuf_pool_small = 0;
#endif
	int tot_len = len;
	pbuf_type type = (tx || len > PBUF_POOL_BUFSIZE) ? PBUF_RAM : PBUF_POOL;
	if (type == PBUF_RAM) {
		/* space is not reserved after pbuf_alloc(), add more space */
		tot_len += LWIP_MBUF_HEAD_SPACE + LWIP_MBUF_TAIL_SPACE;
	}
#if LWIP_PBUF_POOL_SMALL
	else if (len <= PBUF_POOL_SMALL_BUFSIZE) {
		pbuf_pool_small = 1;
	}

retry:
	p = pbuf_alloc_ext(PBUF_MBUF_RAW, tot_len, type, pbuf_pool_small);
	if (p == NULL) {
		MBUF_DEBUG("Out of memory in pbuf pool, tot_len %d, type %d, small %d\n",
			tot_len, type, pbuf_pool_small);
		if (pbuf_pool_small) {
			/* try to get pbuf from bigger pbuf pools */
			pbuf_pool_small = 0;
			goto retry;
		}
		MBUF_DEBUG("Out of memory in pbuf pool, tot_len %d, type %d\n",
			tot_len, type);
		mb_free(m);
		return NULL;
	}
#else /* LWIP_PBUF_POOL_SMALL */
	p = pbuf_alloc(PBUF_MBUF_RAW, tot_len, type);
	if (p == NULL) {
		MBUF_DEBUG("Out of memory in pbuf pool, tot_len %d, type %d\n",
			tot_len, type);
		mb_free(m);
		return NULL;
	}
#endif /* LWIP_PBUF_POOL_SMALL */
	if (type == PBUF_RAM) {
		/* reserved head and tail space of pbuf */
		pbuf_header(p, -LWIP_MBUF_HEAD_SPACE);
		p->len -= LWIP_MBUF_TAIL_SPACE;
		p->tot_len -= LWIP_MBUF_TAIL_SPACE;
		p->mb_flags |= PBUF_FLAG_MBUF_SPACE;
	}

	MBUF_DATA_INIT(m, p);
	return m;
}

/*
 * Free a mbuf.
 */
void mb_free(struct mbuf *m)
{
	struct pbuf *p = m->m_pbuf;
	if (p) {
		p->mb_flags &= ~PBUF_FLAG_MBUF_REF;
		pbuf_free(p);
	}
	memp_free(MEMP_MBUF, m);
}

/*
 * Create a new mbuf including all pbuf data.
 */
struct mbuf *mb_pbuf2mbuf(void *p)
{
	struct pbuf *pb = p;
	struct mbuf *m;

	if ((pbuf_clen(pb) > 1) || ((pb->mb_flags & PBUF_FLAG_MBUF_SPACE) == 0)) {
		/* copy all data from @pb to a new single pbuf */
		m = mb_get(pb->tot_len, 1);
		if (m == NULL)
			return NULL;

		if (pbuf_copy_partial(pb, m->m_data, pb->tot_len, 0) != pb->tot_len) {
			mb_free(m);
			return NULL;
		}
	} else {
		/* no need to copy data, link @pb to a new mbuf header */
		m = mb_alloc();
		if (m == NULL)
			return NULL;

		pbuf_ref(pb); /* @pb is referenced by @m now */
		MBUF_DATA_INIT(m, pb);
	}
	return m;
}

/*
 * Return a pbuf included in a mbuf
 */
void *mb_mbuf2pbuf(struct mbuf *m)
{
	struct pbuf *p = m->m_pbuf;

	p->payload = m->m_data;
	p->tot_len = m->m_len;
	p->len = m->m_len;
	if (m->m_tailspace >= LWIP_MBUF_TAIL_SPACE)
		p->mb_flags |= PBUF_FLAG_MBUF_SPACE;
	pbuf_ref(p); /* add reference to avoid freed from @m */
	return p;
}
#endif /* (!defined(__CONFIG_ARCH_DUAL_CORE) || defined(__CONFIG_ARCH_APP_CORE)) */

#if (!defined(__CONFIG_ARCH_DUAL_CORE) || defined(__CONFIG_ARCH_NET_CORE))
#include "lwip/opt.h"
#include "sys/ducc/ducc_net.h"

/* Add space at the head of mbuf, no sanity checks */
#define MBUF_HEAD_ADJ(m, increment)			\
	do {						\
		(m)->m_headspace -= (increment);	\
		(m)->m_data -= (increment);		\
		(m)->m_len += (increment);		\
		(m)->m_pkthdr.len += (increment);	\
	} while (0)

/* Add space at the tail of mbuf, no sanity checks */
#define MBUF_TAIL_ADJ(m, increment) 			\
	do {						\
		(m)->m_tailspace -= (increment);	\
		(m)->m_len += (increment);		\
		(m)->m_pkthdr.len += (increment);	\
	} while (0)

/* copy some members of mbuf from @s to @d */
#define MBUF_PKTHDR_INIT(d, s, l)						  \
	do {									  \
		(d)->m_flags = ((s)->m_flags & M_COPYFLAGS) | M_PKTHDR;		  \
		MB_MEMCPY(&(d)->m_pkthdr, &(s)->m_pkthdr, sizeof(struct pkthdr)); \
		(d)->m_pkthdr.len = l;						  \
	} while (0)


#ifdef __CONFIG_ARCH_DUAL_CORE
/*
 * @tx: 1 means mbuf is used to do Tx, always alloc it from PBUF_RAM.
 * @return a mbuf including @len data
 */
struct mbuf *mb_get(int len, int tx)
{
	struct ducc_param_mbuf_get param;
	param.len = len;
	param.tx = tx;
	param.mbuf = NULL;
	if (ducc_net_ioctl(DUCC_NET_CMD_MBUF_GET, &param) != 0) {
		return NULL;
	} else {
		struct mbuf *m = param.mbuf;
		MBUF_APP2NET(m);
		return m;
	}
}

/*
 * Free a mbuf.
 */
void mb_free(struct mbuf *m)
{
	MBUF_NET2APP(m);
	ducc_net_ioctl(DUCC_NET_CMD_MBUF_FREE, m);
}

#endif /* __CONFIG_ARCH_DUAL_CORE */

/*
 * Trim data from head or tail.
 *
 * @return 0 on success, -1 on failure.
 */
int mb_adj(struct mbuf *m, int req_len)
{
	if (req_len >= 0) {
		/* Trim from head. */
		if (req_len > m->m_len) {
			MBUF_ERROR("Failed to trim from head, %d > %d\n",
				req_len, (int)m->m_len);
			return -1; /* no enough data to trim */
		}
		MBUF_HEAD_ADJ(m, -req_len);
	} else {
		/* Trim from tail. */
		req_len = -req_len;
		if (req_len > m->m_len) {
			MBUF_ERROR("Failed to trim from tail, %d > %d\n",
				req_len, (int)m->m_len);
			return -1; /* no enough data to trim */
		}
		MBUF_TAIL_ADJ(m, -req_len);
	}
	return 0;
}

/*
 * Copy data from an mbuf chain starting "off" bytes from the beginning,
 * continuing for "len" bytes, into the indicated buffer.
 *
 * @return the number of bytes copied
 */
int mb_copydata(const struct mbuf *m, int off, int len, uint8_t *cp)
{
	if (off < 0 || len <= 0) {
		MBUF_ERROR("Invalid args: off %d, len %d\n", off, len);
		return 0;
	}
#if 0
	struct pbuf *p = m->m_pbuf;
	if (p == NULL)
		return 0;

	if ((pbuf_clen(p) != 1) || ((p->mb_flags & PBUF_FLAG_MBUF_SPACE) == 0)) {
		MBUF_ERROR("Invalid mbuf!\n");
		return 0;
	}
#endif
	int copy_len = m->m_len - off;
	if (copy_len > len)
		copy_len = len;
	MB_MEMCPY(cp, m->m_data + off, copy_len);
	return copy_len;
}

/*
 * Copy a packet header mbuf chain into a completely new chain.
 */
struct mbuf *mb_dup(struct mbuf *m)
{
	if (m == NULL) {
		MBUF_ERROR("Invalid args: m is NULL!\n");
		return NULL;
	}

	int32_t head_increment = LWIP_MBUF_HEAD_SPACE - (int32_t)m->m_headspace;
	int32_t tail_increment = LWIP_MBUF_TAIL_SPACE - (int32_t)m->m_tailspace;
	struct mbuf *nm = mb_get(m->m_len - head_increment - tail_increment, 1);
	if (nm == NULL)
		return NULL;

	MBUF_HEAD_ADJ(nm, head_increment);
	MBUF_TAIL_ADJ(nm, tail_increment);

	MB_MEMCPY(nm->m_data, m->m_data, m->m_len);
	MBUF_PKTHDR_INIT(nm, m, m->m_len);
	return nm;
}

/*
 * Rearange an mbuf chain so that len bytes are contiguous
 * and in the data area of an mbuf (so that mtod will work
 * for a structure of size len).  Returns the resulting
 * mbuf chain on success, frees it and returns null on failure.
 * If there is room, it will add up to max_protohdr-len extra bytes to the
 * contiguous region in an attempt to avoid being called next time.
 */
struct mbuf *mb_pullup(struct mbuf *m, int len) // NOT really support!
{
	if (m->m_len < len) {
		mb_free(m);
		return NULL;
	}
	return m;
}

/*
 * Partition an mbuf chain in two pieces, returning the tail --
 * all but the first len0 bytes.  In case of failure, it returns NULL and
 * attempts to restore the chain to its original state.
 *
 * Note that the resulting mbufs might be read-only, because the new
 * mbuf can end up sharing an mbuf cluster with the original mbuf if
 * the "breaking point" happens to lie within a cluster mbuf. Use the
 * M_WRITABLE() macro to check for this case.
 */
struct mbuf *mb_split(struct mbuf *m0, int len0)
{
	if (m0 == NULL || len0 <= 0) {
		MBUF_ERROR("Invalid args: m0 %p, len0 %d\n", m0, len0);
		return NULL;
	}

	if (m0->m_len <= len0) {
		MBUF_ERROR("m0->m_len %d < len0 %d\n", (int)m0->m_len, len0);
		return NULL;
	}

	/* create a new mbuf to save all the tail data */
	int len = m0->m_len - len0;
	struct mbuf *m = mb_get(len, 1);
	if (m == NULL)
		return NULL;

	MB_MEMCPY(m->m_data, m0->m_data + len0, len);
	MBUF_PKTHDR_INIT(m, m0, len);

	MBUF_TAIL_ADJ(m0, -len); /* adjust @m0 */
	return m;
}

/*
 * Append the specified data to the indicated mbuf chain,
 * Extend the mbuf chain if the new data does not fit in
 * existing space.
 *
 * Return 1 if able to complete the job; otherwise 0.
 */
int mb_append(struct mbuf *m, int len, const uint8_t *cp)
{
	if (len > m->m_tailspace) {
		MBUF_ERROR("Failed to append data, %d > %d\n",
			len, (int)m->m_tailspace);
		return 0;
	}

	uint8_t *dst = m->m_data + m->m_len;
	MBUF_TAIL_ADJ(m, len);
	if (cp)
		MB_MEMCPY(dst, cp, len);
	return 1;
}

/*
 * Adjust the mbuf to reserve space directly.
 *
 * @return 0 on success, -1 on failure.
 */
int mb_reserve(struct mbuf *m, int len, uint16_t headspace, uint16_t tailspace)
{
	uint8_t *buf_end = m->m_data + m->m_len + m->m_tailspace;
	int buf_len = m->m_headspace + m->m_len + m->m_tailspace;

	if (buf_len < headspace + len + tailspace) {
		MBUF_ERROR("Failed to reserve, (%d + %d + %d) < (%d + %d + %d)\n",
			m->m_headspace, m->m_len, m->m_tailspace,
			headspace, len, tailspace);
		return -1;
	}

	m->m_tailspace = tailspace;
	m->m_len = len;
	m->m_pkthdr.len = len;
	m->m_data = buf_end - tailspace - len;
	m->m_headspace = buf_len - tailspace - len;
	return 0;
}
#endif /* (!defined(__CONFIG_ARCH_DUAL_CORE) || defined(__CONFIG_ARCH_NET_CORE)) */
