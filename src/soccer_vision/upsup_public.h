/* 
 * Copyright 1994-2002 The MathWorks, Inc.
 */
#ifndef upsup_public_h
#define upsup_public_h

/*
 * Maps out a section of a circular buffer.  When the buffer is not
 * wrapped, only section 1 is required.  When the buffer is wrapped
 * both sections are required.
 */
typedef struct BufMem_tag {
    int_T   nBytes1;    /* nbytes in sections 1          */
    char_T  *section1;  /* ptr to start of section 1     */

    int_T   nBytes2;    /* nbytes in section 2 (0 if not wrapped)          */
    char_T  *section2;  /* ptr to start of section 2 (NULL if not wrapped) */
} BufMem;



/*
 * For each of nActiveBufs (buffers with data in them) we have a list of
 * the buffer memory (bufs) and a list of the tid's with which this buffer
 * is associated.
 */
typedef struct ExtBufMemList_tag {
    int_T        nActiveBufs; /* num bufs with data to upload */

    const BufMem *bufs; /* sections of each buffer for uploading */
    const int_T  *tids; /* tid associated with each section      */
} ExtBufMemList; 

typedef struct BdUploadInfo_tag BdUploadInfo;

#endif /* upsup_public_h */
