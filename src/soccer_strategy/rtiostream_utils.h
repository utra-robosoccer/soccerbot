/* Copyright 2012 The MathWorks, Inc. */

#ifndef _RTIOSTREAM_UTILS_H_
#define _RTIOSTREAM_UTILS_H_


#ifdef __cplusplus
extern "C" {
#endif
    
/* Target-side rtIOStream utility APIs */

#include "rtwtypes.h" /* include definition of uint32_T */
#include <stddef.h>   /* include definition of size_t */

/* MIN utility */
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
          
/* Blocks until all requested outgoing data is sent. */
extern int rtIOStreamBlockingSend(const int streamID,
                                  const void * const src,
                                  uint32_T size);

/* Blocks until all requested incoming data is received. */
extern int rtIOStreamBlockingRecv(const int streamID,
                                  void * const dst,
                                  uint32_T size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RTIOSTREAM_UTILS_H_ */
