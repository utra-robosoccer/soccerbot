/* Copyright 2012-2013 The MathWorks, Inc. */

#include "rtiostream_utils.h"

/* include rtIOStream interface to use */
#include "rtiostream.h" 

/* TARGET_CONNECTIVITY_TESTING might be defined by some MathWorks tests for
 * testing purposes only. In this case, we force SIZE_MAX to be 4 so we 
 * could test the pointer arithmetic in rtIOStreamBlockingSend and
 * rtIOStreamBlockingRecv.
 */ 
#ifdef TARGET_CONNECTIVITY_TESTING
    #define SIZE_MAX 4
#else
    /* define SIZE_MAX if not already defined (e.g. by a C99 compiler) */
    #ifndef SIZE_MAX
        #define SIZE_MAX ((size_t)-1)
    #endif
#endif

#ifndef MemUnit_T
   /* External Mode */
   typedef unsigned char IOUnit_T; 
#else
   /* SIL/PIL */
   #ifdef HOST_WORD_ADDRESSABLE_TESTING
      /* rtIOStream will handle data in single byte chunks 
       *
       * uint8_T can be > 8-bits for certain portable word sizes 
       * cases (e.g. C2000) so use native type instead */
      typedef unsigned char IOUnit_T;
   #else
      /* rtIOStream will handle data in MemUnit_T size chunks */
      typedef MemUnit_T IOUnit_T;
   #endif
#endif

/* Blocks until all requested outgoing data is sent */
int rtIOStreamBlockingSend(const int streamID, 
                           const void * const src, 
                           uint32_T size) {

    size_t transferAmount;
    size_t sizeSent;
    int errorCode      = RTIOSTREAM_NO_ERROR;   
    const IOUnit_T * srcPtr = (const IOUnit_T *) src;
    
    /* use a variable to avoid SIZE_MAX being treated as a constant
     * which leads to compiler warnings for "MIN" on platforms where
     * SIZE_MAX > UINT32_MAX */
    size_t sizeMax = SIZE_MAX;
    while (size > 0) {
        /* support full uint32 size */
        transferAmount = (size_t) MIN(sizeMax, size);        
        errorCode = rtIOStreamSend(streamID,
                                   (const void *) srcPtr,
                                   transferAmount,
                                   &sizeSent);
        if (errorCode == RTIOSTREAM_ERROR) {
            return errorCode;
        }
        else {            
            size -= (uint32_T) sizeSent;
            srcPtr += sizeSent;
        }
    }
    return errorCode;
}

/* Blocks until all requested incoming data is received */
int rtIOStreamBlockingRecv(const int streamID,
                           void * const dst,
                           uint32_T size) {

   size_t transferAmount;
   size_t sizeRecvd;
   int errorCode      = RTIOSTREAM_NO_ERROR;
   IOUnit_T * dstPtr = (IOUnit_T *) dst;
   
   /* use a variable to avoid SIZE_MAX being treated as a constant
    * which leads to compiler warnings for "MIN" on platforms where 
    * SIZE_MAX > UINT32_MAX */
   size_t sizeMax = SIZE_MAX;
   while (size > 0) {
      /* support full uint32 size */
      transferAmount = (size_t) MIN(sizeMax, size);      
      errorCode = rtIOStreamRecv(streamID, 
                                 (void *) dstPtr, 
                                 transferAmount, 
                                 &sizeRecvd);
      if (errorCode == RTIOSTREAM_ERROR) {
            return errorCode;
      }
      else {          
         size -= (uint32_T) sizeRecvd;
         dstPtr += sizeRecvd;
      }
   }
   return errorCode;
}


