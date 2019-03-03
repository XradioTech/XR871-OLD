/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : CdxFifoStream.h
 * Description : Stream
 * History :
 *
 */

#ifndef CDX_FIFO_STREAM_H
#define CDX_FIFO_STREAM_H

#include <CdxTypes.h>
#include <cdx_log.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CdxFifoStreamS CdxFifoStreamT;

struct CdxFifoStreamOpsS {
    void (*lock)(CdxFifoStreamT *);
    void (*unlock)(CdxFifoStreamT *);
    cdx_int32 (*in)(CdxFifoStreamT *, void *buf, cdx_uint32 len);
    cdx_int32 (*out)(CdxFifoStreamT *, void *buf, cdx_uint32 len);
    cdx_int32 (*valid)(CdxFifoStreamT *);    /* get the size of valid data in the fifo */
    cdx_int32 (*avail)(CdxFifoStreamT *);    /* get the remain room of the fifo */
    cdx_int32 (*seteos)(CdxFifoStreamT *);   /* mean no more data */
    cdx_bool (*iseos)(CdxFifoStreamT *);     /* mean no more data */
};

struct CdxFifoStreamS {
    const struct CdxFifoStreamOpsS *ops;
};

static inline void CdxFifoStreamLock(struct CdxFifoStreamS *stream)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->lock);
    return stream->ops->lock(stream);
}

static inline void CdxFifoStreamUnlock(struct CdxFifoStreamS *stream)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->unlock);
    return stream->ops->unlock(stream);
}

static inline cdx_int32 CdxFifoStreamIn(struct CdxFifoStreamS *stream, void *buf, cdx_uint32 len)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->in);
    return stream->ops->in(stream, buf, len);
}

static inline cdx_int32 CdxFifoStreamOut(struct CdxFifoStreamS *stream, void *buf, cdx_uint32 len)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->out);
    return stream->ops->out(stream, buf, len);
}

static inline cdx_int32 CdxFifoStreamValid(struct CdxFifoStreamS *stream)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->valid);
    return stream->ops->valid(stream);
}

static inline cdx_int32 CdxFifoStreamAvail(struct CdxFifoStreamS *stream)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->avail);
    return stream->ops->avail(stream);
}

static inline cdx_int32 CdxFifoStreamSeteos(struct CdxFifoStreamS *stream)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->seteos);
    return stream->ops->seteos(stream);
}

static inline cdx_bool CdxFifoStreamIseos(struct CdxFifoStreamS *stream)
{
    CDX_CHECK(stream);
    CDX_CHECK(stream->ops);
    CDX_CHECK(stream->ops->iseos);
    return stream->ops->iseos(stream);
}

#ifdef __cplusplus
}
#endif

#endif

