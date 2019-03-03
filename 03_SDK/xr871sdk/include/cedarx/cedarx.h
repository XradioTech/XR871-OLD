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

#ifndef _CEDARX_H_
#define _CEDARX_H_

typedef enum {
    DEMUX_THREAD,
} CDX_THREAD_NAME;

int CedarxStreamListInit(void);
int CedarxStreamRegisterHttps(void);
int CedarxStreamRegisterSsl(void);
int CedarxStreamRegisterFlash(void);
int CedarxStreamRegisterFile(void);
int CedarxStreamRegisterFifo(void);
int CedarxStreamRegisterHttp(void);
int CedarxStreamRegisterTcp(void);
int CedarxStreamRegisterCustomer(void);

int CedarxParserListInit(void);
int CedarxParserRegisterM3U(void);
int CedarxParserRegisterM4A(void);
int CedarxParserRegisterAAC(void);
int CedarxParserRegisterAMR(void);
int CedarxParserRegisterMP3(void);
int CedarxParserRegisterWAV(void);

int CedarxDecoderListInit(void);
int CedarxDecoderRegisterAAC(void);
int CedarxDecoderRegisterAMR(void);
int CedarxDecoderRegisterMP3(void);
int CedarxDecoderRegisterWAV(void);

int CedarxWriterListInit(void);
int CedarxWriterRegisterFile(void);
int CedarxWriterRegisterCallback(void);
int CedarxWriterRegisterCustomer(void);

int CedarxMuxerListInit(void);
int CedarxMuxerRegisterAmr(void);
int CedarxMuxerRegisterPcm(void);

int CedarxEncoderListInit(void);
int CedarxEncoderRegisterAmr(void);
int CedarxEncoderRegisterPcm(void);

int CedarxThreadStackSizeSet(CDX_THREAD_NAME name, int size);

#endif
