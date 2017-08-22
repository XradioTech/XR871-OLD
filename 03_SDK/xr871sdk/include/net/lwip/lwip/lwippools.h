/**
 * Define pools used by mbuf.
 *
 * LWIP_MEMPOOL(pool_name, number_elements, element_size, pool_description)
 *     creates a pool name MEMP_pool_name. description is used in stats.c
 */

#if LWIP_MBUF_SUPPORT

#if (MEMP_MEM_MALLOC && LWIP_XR_MEM)
#define MEMP_NUM_MBUF   96
#else
#define MEMP_NUM_MBUF   64
#endif

LWIP_MEMPOOL(MBUF,      MEMP_NUM_MBUF,    sizeof(struct mbuf),      "MBUF")

#endif /* LWIP_MBUF_SUPPORT */
