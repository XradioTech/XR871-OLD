/*
 * Private includes and definitions for userspace use of XZ Embedded
 *
 * Author: Lasse Collin <lasse.collin@tukaani.org>
 *
 * This file has been put into the public domain.
 * You can do whatever you want with this file.
 */

#ifndef XZ_OPT_H
#define XZ_OPT_H

/* Uncomment to enable CRC64 support. */
/* #define XZ_USE_CRC64 */

/* Uncomment as needed to enable BCJ filter decoders. */
/* #define XZ_DEC_X86 */
/* #define XZ_DEC_POWERPC */
/* #define XZ_DEC_IA64 */
/* #define XZ_DEC_ARM */
#define XZ_DEC_ARMTHUMB
/* #define XZ_DEC_SPARC */

#endif /* XZ_OPT_H */
