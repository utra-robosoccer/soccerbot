/*
 * Copyright 1994-2017 The MathWorks, Inc.
 *
 * File: ext_svr.c     
 *
 * Abstract:
 *  External mode server interface (TCPIP example).  Provides functions
 *  that get called by main routine (model-name.c):
 *    o ExtParseArgsAndInitUD:  parse args and create UserData
 *    o ExtWaitForStartPkt:     return true if waiting for host to start
 *    o rt_ExtModeInit:         external mode initialization
 *    o rt_ExtModeSleep:        pause the process
 *    o rt_PktServerWork:       server for setting/getting packets from host
 *    o rt_PktServer:           server dispatcher - for multi-tasking targets
 *    o rt_UploadServerWork:    server for setting data upload packets on host
 *    o rt_UploadServer:        server dispatcher - for multi-tasking targets
 *    o rt_ExtModeShutdown:     external mode termination
 *
 *  Parameter downloading and data uploading supported for single and
 *  multi-tasking targets.
 */

/*****************
 * Include files *
 *****************/

/*ANSI C headers*/
#ifndef EXTMODE_DISABLEPRINTF  
#include <stdio.h>
#endif

#include <stdlib.h>
#include <string.h>

#if defined(VXWORKS)
 /*VxWorks headers*/
# include <selectLib.h>
# include <sockLib.h>
# include <inetLib.h>
# include <ioLib.h>
# include <taskLib.h>
#endif

/*Real Time Workshop headers*/
#include "rtwtypes.h"
#include "multiword_types.h"
#include "rtw_extmode.h"

#include "ext_types.h"
#include "ext_share.h"
#include "ext_test.h"
#include "ext_svr_transport.h"
#include "updown.h"
#include "updown_util.h"
#include "dt_info.h"


/*Uncomment to test 4 byte reals*/
/*#define real_T float*/

/***********************
 * Logical definitions *
 ***********************/
#if (!defined(__cplusplus))
#  ifndef false
#   define false                       (0U)
#  endif
#  ifndef true
#   define true                        (1U)
#  endif
#endif

/**********************
 * External Variables *
 **********************/
extern int_T           volatile startModel;
extern TargetSimStatus volatile modelStatus;
#ifdef VXWORKS
extern SEM_ID uploadSem;
extern SEM_ID pktSem;
#endif
extern boolean_T host_upstatus_is_uploading;

/********************
 * Global Variables *
 ********************/

/*
 * Flags.
 */
PRIVATE boolean_T   connected       = false;
PRIVATE boolean_T   commInitialized = false;
boolean_T gblSetParamPktReceived = false;

/*
 * Pointer to opaque user data (defined by ext_svr_transport.c).
 */
PRIVATE ExtUserData *extUD          = NULL;

/*
 * Buffer used to receive packets.
 */
PRIVATE int_T pktBufSize = 0;
PRIVATE char  *pktBuf    = NULL;


#ifndef EXTMODE_DISABLESIGNALMONITORING
#ifndef EXTMODE_DISABLEPRINTF 
PRIVATE char ERRMSG_PROCESSSELECTSIGNAL[] = 
            "\nError in UploadLogInfoInit(). Most likely a memory\n"
            "allocation error or an attempt to re-initialize the\n"
            "signal selection during the data logging process\n"
            "(i.e., multiple EXT_SELECT_SIGNAL packets were received\n"
            "before the logging session terminated or an\n"
            "EXT_CANCEL_LOGGING packet was received)\n";

PRIVATE char ERRMSG_PROCESSSELECTTRIGGER[] = 
            "\nError in UploadInitTrigger(). Most likely a memory\n"
            "allocation error or an attempt to re-initialize the\n"
            "trigger selection during the data logging process\n"
            "(i.e., multiple EXT_SELECT_TRIGGER packets were received\n"
            "before the logging session terminated or an\n"
            "EXT_CANCEL_LOGGING packet was received)\n";
#else
PRIVATE char ERRMSG_PROCESSSELECTSIGNAL[] = "";
PRIVATE char ERRMSG_PROCESSSELECTTRIGGER[] = "";
#endif
#endif

/********************
 *  If DAEMON_MODE  *
 ********************/
#ifdef DAEMON_MODE
/*
 * This defines the size of each data chunk (in bytes) that the
 * daemon acknowledgement scheme uses for large data
 */
const int_T daemonChunkSize = 32;
#endif

PRIVATE boolean_T SendPktHdrToHost(const ExtModeAction action,
                                         const int size);

/*******************
 * Local Functions *
 *******************/

/* Function: GrowRecvBufIfNeeded ===============================================
 * Abstract:
 *  Allocate or increase the size of buffer for receiving packets from target.
 */
PRIVATE boolean_T GrowRecvBufIfNeeded(const int pktSize)
{
    if (pktSize > pktBufSize) {
        if (pktBuf != NULL) {
            free(pktBuf);
            pktBufSize = 0;
        }

        pktBuf = (char *)malloc(pktSize);
        if (pktBuf == NULL) return(EXT_ERROR);

        pktBufSize = pktSize;
    }
    return(EXT_NO_ERROR);
} /* end GrowRecvBufIfNeeded */


/* Function: GetPktHdr =========================================================
 * Abstract:
 *  Attempts to retrieve a packet header from the host.  If a header is in 
 *  fact retrieved, the reference arg, 'hdrAvail' will be returned as true.
 *
 *  EXT_NO_ERROR is returned on success, EXT_ERROR is returned on failure.
 *
 * NOTES:
 *  o It is not necessarily an error for 'hdrAvail' to be returned as false.
 *    It typically means that we were polling for packets and none were
 *    available.
 */
PRIVATE boolean_T GetPktHdr(PktHeader *pktHdr, boolean_T *hdrAvail)
{
    int_T     nGot      = 0; /* assume */
    int_T     nGotTotal = 0;
    int_T     pktSize   = sizeof(PktHeader);
    boolean_T error     = EXT_NO_ERROR;
    
    /* Get the header. */
    while(nGotTotal < pktSize) {
        error = ExtGetHostPkt(extUD,
            pktSize - nGotTotal, &nGot, (char_T *)((char_T *)pktHdr + nGotTotal));
        if (error) goto EXIT_POINT;

        nGotTotal += nGot;

        if (nGotTotal == 0) break;
    }
    assert((nGot == 0) || (nGotTotal == pktSize));

EXIT_POINT:
    *hdrAvail = (boolean_T)(nGot > 0);
    return(error);
} /* end GetPktHdr */


/* Function: ClearPkt ==========================================================
 * Abstract:
 *  Remove the data from the communication line one byte at a time.  This
 *  function is called when there was not enough memory to receive an entire
 *  packet.  Since the data was never received, it must be discarded so that
 *  other packets can be sent.
 */
PRIVATE void ClearPkt(const int pktSize)
{
    int_T     nGot;
    boolean_T error     = EXT_NO_ERROR;
    int_T     nGotTotal = 0;
    char      buffer;

    /* Get and discard the data one char at a time. */
    while(nGotTotal < pktSize) {
        error = ExtGetHostPkt(extUD, 1, &nGot, (char_T *)&buffer);
        if (error) {
#ifndef EXTMODE_DISABLEPRINTF            
            fprintf(stderr,"ExtGetHostPkt() failed.\n");
#endif
            goto EXIT_POINT;
        }
        nGotTotal += nGot;
    }

EXIT_POINT:
    return;

} /* end ClearPkt */

#ifdef DAEMON_MODE
/* Function: ClearPktUsingAck ==================================================
 * Abstract:
 *  Derived from ClearPkt(). For the general behaviour see ClearPkt() above.
 *  This variant is used by DAEMON_MODE to clear data in small chunks.
 *  After receiving each chunk reply with a EXT_DAEMON_ACK.
 */
PRIVATE void ClearPktUsingAck(const int pktSize)
{
    int_T     nGot;
    boolean_T error     = EXT_NO_ERROR;
    int_T     nGotTotal = 0;
    char      buffer;
    
    int_T     i;
    int_T     nChunks   = pktSize/daemonChunkSize;
    int_T     bytesLeft = pktSize%daemonChunkSize;

    /* Get and discard the data one char at a time.
	 * Receive chunks here, and reply with ack after receiving each chunk
	 */
    for(i=0; i<nChunks; i++){
        nGotTotal = 0;
        while(nGotTotal < daemonChunkSize){
            error = ExtGetHostPkt(extUD,
                        1,
                        &nGot,
                        (char_T *)&buffer);
            if (error) {
#ifndef EXTMODE_DISABLEPRINTF                
                fprintf(stderr,"ExtGetHostPkt() failed.\n");
#endif
                goto EXIT_POINT;
            }
            nGotTotal += nGot;
        }
        SendPktHdrToHost(EXT_DAEMON_ACK, 0);
    }
    
    /*
	 * Receive the bytes left over and reply with an ack after you receive all
	 */
    nGotTotal=0;
    if(bytesLeft != 0){
        while(nGotTotal<bytesLeft){
            error = ExtGetHostPkt(extUD,
                        1,
                        &nGot,
                        (char_T *)&buffer);
            if (error) {
#ifndef EXTMODE_DISABLEPRINTF                
                fprintf(stderr,"ExtGetHostPkt() failed.\n");
#endif
                goto EXIT_POINT;
            }
        nGotTotal += nGot;
        }
        SendPktHdrToHost(EXT_DAEMON_ACK, 0);
    }

EXIT_POINT:
    return;

} /* end ClearPktUsingAck */
#endif


/* Function: GetPkt ============================================================
 * Abstract:
 *  Receive nBytes from the host.  Return a buffer containing the bytes or
 *  NULL if an error occurs.  Note that the pointer returned is that of the
 *  global pktBuf.  If the buf needs to be grown to accommodate the package,
 *  it is realloc'd.  This function will try to get the requested number
 *  of bytes indefinitely - it is assumed that the data is either already there,
 *  or will show up in a "reasonable" amount of time.
 */
PRIVATE const char *GetPkt(const int pktSize)
{
    int_T     nGot;
    boolean_T error     = EXT_NO_ERROR;
    int_T     nGotTotal = 0;

    error = GrowRecvBufIfNeeded(pktSize);
    if (error != EXT_NO_ERROR) {
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,"Previous pkt from host thrown away due to lack of memory.\n");
#endif
        ClearPkt(pktSize);
        goto EXIT_POINT;
    }
    
    /* Get the data. */
    while(nGotTotal < pktSize) {
        error = ExtGetHostPkt(extUD,
            pktSize - nGotTotal, &nGot, (char_T *)(pktBuf + nGotTotal));
        if (error) {
#ifndef EXTMODE_DISABLEPRINTF                
            fprintf(stderr,"ExtGetHostPkt() failed.\n");
#endif
            goto EXIT_POINT;
        }

    nGotTotal += nGot;
    }

EXIT_POINT:
    return((error == EXT_NO_ERROR) ? pktBuf : NULL);
} /* end GetPkt */

#ifdef DAEMON_MODE
/* Function: GetPktUsingAck ============================================================
 * Abstract:
 *  Derived from GetPkt(). For the general behaviour see GetPkt() above.
 *  This variant is used by DAEMON_MODE to fetch data in small chunks.
 *  After receiving each chunk reply with a EXT_DAEMON_ACK.
 */
PRIVATE const char *GetPktUsingAck(const int pktSize)
{
    int_T     nGot;
    boolean_T error     = EXT_NO_ERROR;
    int_T     nGotTotal = 0;
    
    int_T     i;
    int_T     nChunks   = pktSize/daemonChunkSize;
    int_T     bytesLeft = pktSize%daemonChunkSize;

    error = GrowRecvBufIfNeeded(pktSize);
    if (error != EXT_NO_ERROR) {
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,"Previous pkt from host thrown away due to lack of memory.\n");
#endif
        ClearPktUsingAck(pktSize);
        goto EXIT_POINT;
    }
    
    /*
	 * Receive chunks here, and reply with ack after receiving each chunk
	 */
    for(i=0; i<nChunks; i++){
        nGotTotal = 0;
        while(nGotTotal < daemonChunkSize){
            error = ExtGetHostPkt(extUD,
                        daemonChunkSize - nGotTotal,
                        &nGot,
                        (char_T *)(pktBuf + i*daemonChunkSize + nGotTotal));
            if (error) {
#ifndef EXTMODE_DISABLEPRINTF                
                fprintf(stderr,"ExtGetHostPkt() failed.\n");
#endif
            goto EXIT_POINT;
            }
            nGotTotal += nGot;
        }
        SendPktHdrToHost(EXT_DAEMON_ACK, 0);
    }
    
    /*
	 * Receive the bytes left over and reply with an ack after you receive all
	 */
    nGotTotal=0;
    if(bytesLeft != 0){
        while(nGotTotal<bytesLeft){
            error = ExtGetHostPkt(extUD,
                        bytesLeft - nGotTotal,
                        &nGot,
                        (char_T *)(pktBuf + i*daemonChunkSize + nGotTotal));
            if (error) {
#ifndef EXTMODE_DISABLEPRINTF                
                fprintf(stderr,"ExtGetHostPkt() failed.\n");
#endif
                goto EXIT_POINT;
            }
        nGotTotal += nGot;
        }
        SendPktHdrToHost(EXT_DAEMON_ACK, 0);
    }

EXIT_POINT:
    return((error == EXT_NO_ERROR) ? pktBuf : NULL);
} /* end GetPktUsingAck */
#endif


#ifndef EXTMODE_DISABLESIGNALMONITORING
/* Forward declaration */
void UploadServerWork(int32_T, int_T numSampTimes);
#endif

/* Function: DisconnectFromHost ================================================
 * Abstract:
 *  Disconnect from the host.
 */
PRIVATE void DisconnectFromHost(int_T numSampTimes)
{
    int i;
    boolean_T upload_signals = host_upstatus_is_uploading;

    for (i=0; i<NUM_UPINFOS; i++) {
        UploadPrepareForFinalFlush(i);

#if defined(VXWORKS)
        /*
         * UploadPrepareForFinalFlush() has already called semGive(uploadSem)
         * two times.  Now the server thread will wait until the upload thread
         * has processed all of the data in the buffers for the final upload
         * and exhausted the uploadSem semaphores.  If the server thread
         * attempts to call UploadServerWork() while the upload thread is in
         * the middle of processing the buffers, the target code may crash
         * with a NULL pointer exception (the buffers are destroyed after
         * calling UploadLogInfoTerm).
         */
        while(semTake(uploadSem, NO_WAIT) != ERROR) {
            semGive(uploadSem);
            taskDelay(1000);
        }
#else
#ifndef EXTMODE_DISABLESIGNALMONITORING
        if (upload_signals) {
            UploadServerWork(i, numSampTimes);
        }
#endif
#endif

        UploadLogInfoTerm(i, numSampTimes);
    }
    
    connected       = false;
    commInitialized = false;
    
    ExtCloseConnection(extUD);
} /* end DisconnectFromHost */


/* Function: ForceDisconnectFromHost ===========================================
 * Abstract:
 *  Force a disconnect from the host.  This is not a graceful shutdown and
 *  should only be used when the integrity of the external mode connection
 *  is in question.  To shutdown the connection gracefully, use
 *  DisconnectFromHost().
 */
PRIVATE void ForceDisconnectFromHost(int_T numSampTimes)
{
    int i;
    connected       = false;
    commInitialized = false;

    for (i=0; i<NUM_UPINFOS; i++) {
        UploadEndLoggingSession(i, numSampTimes);
    }

    ExtForceDisconnect(extUD);
} /* end ForceDisconnectFromHost */


/* Function: ProcessConnectPkt =================================================
 * Abstract:
 *  Process the EXT_CONNECT packet and send response to host.
 */
PRIVATE boolean_T ProcessConnectPkt(RTWExtModeInfo *ei)
{
    int_T                   nSet;
    PktHeader               pktHdr;
    int_T                   tmpBufSize;
    uint32_T                *tmpBuf = NULL;
    boolean_T               error   = EXT_NO_ERROR;
    
    const DataTypeTransInfo *dtInfo    = rteiGetModelMappingInfo(ei);
    uint_T                  *dtSizes   = dtGetDataTypeSizes(dtInfo);
    int_T                   nDataTypes = dtGetNumDataTypes(dtInfo);

    assert(connected);
    assert(!comminitialized);

    /*
     * Send the 1st of two EXT_CONNECT_RESPONSE packets to the host. 
     * The packet consists purely of the pktHeader.  In this special
     * case the pktSize actually contains the number of bits per byte
     * (not always 8 - see TI compiler for C30 and C40).
     */
    pktHdr.type = (uint32_T)EXT_CONNECT_RESPONSE;
    pktHdr.size = (uint32_T)8; /* 8 bits per byte */

    error = ExtSetHostPkt(extUD,sizeof(pktHdr),(char_T *)&pktHdr,&nSet);
    if (error || (nSet != sizeof(pktHdr))) {
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,
            "ExtSetHostPkt() failed for 1st EXT_CONNECT_RESPONSE.\n");
#endif
        goto EXIT_POINT;
    }

    /* Send 2nd EXT_CONNECT_RESPONSE packet containing the following 
     * fields:
     *
     * CS1 - checksum 1 (uint32_T)
     * CS2 - checksum 2 (uint32_T)
     * CS3 - checksum 3 (uint32_T)
     * CS4 - checksum 4 (uint32_T)
     *
     * intCodeOnly   - flag indicating if target is integer only (uint32_T)
     * 
     * MWChunkSize   - multiword data type chunk size on target (uint32_T)
     * 
     * targetStatus  - the status of the target (uint32_T)
     *
     * nDataTypes    - # of data types        (uint32_T)
     * dataTypeSizes - 1 per nDataTypes       (uint32_T[])
     */

    {
        int nPktEls    = 4 +                        /* checkSums       */
                         1 +                        /* intCodeOnly     */
                         1 +                        /* MW chunk size   */
                         1 +                        /* targetStatus    */
                         1 +                        /* nDataTypes      */
                         dtGetNumDataTypes(dtInfo); /* data type sizes */

        tmpBufSize = nPktEls * sizeof(uint32_T);
        tmpBuf     = (uint32_T *)malloc(tmpBufSize);
        if (tmpBuf == NULL) {
            error = EXT_ERROR; goto EXIT_POINT;
        }
    }
    
    /* Send packet header. */
    pktHdr.type = EXT_CONNECT_RESPONSE;
    pktHdr.size = tmpBufSize;

    error = ExtSetHostPkt(extUD,sizeof(pktHdr),(char_T *)&pktHdr,&nSet);
    if (error || (nSet != sizeof(pktHdr))) {
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,
            "ExtSetHostPkt() failed for 2nd EXT_CONNECT_RESPONSE.\n");
#endif
        goto EXIT_POINT;
    }
   
    /* Checksums, target status & SL_DOUBLESize. */
    tmpBuf[0] = rteiGetChecksum0(ei);
    tmpBuf[1] = rteiGetChecksum1(ei);
    tmpBuf[2] = rteiGetChecksum2(ei);
    tmpBuf[3] = rteiGetChecksum3(ei);

#if INTEGER_CODE == 0
    tmpBuf[4] = (uint32_T)0;
#else
    tmpBuf[4] = (uint32_T)1;
#endif

    tmpBuf[5] = (uint32_T)sizeof(uchunk_T);
    
    tmpBuf[6] = (uint32_T)modelStatus;

    /* nDataTypes and dataTypeSizes */
    {        
        int i;
        tmpBuf[7] = (uint32_T)nDataTypes;
        for (i=0; i<nDataTypes; i++) {
            tmpBuf[8+i] = (uint32_T)dtSizes[i];
        }
    }
    
    /* Send the packet. */
    error = ExtSetHostPkt(extUD,tmpBufSize,(char_T *)tmpBuf,&nSet);
    if (error || (nSet != tmpBufSize)) {
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,
            "ExtSetHostPkt() failed.\n");
#endif
        goto EXIT_POINT;
    }

    commInitialized = true;

EXIT_POINT:
    free(tmpBuf);
    return(error);
} /* end ProcessConnectPkt */


/* Function: SendPktHdrToHost ==================================================
 * Abstract:
 *  Send a packet header to the host.
 */
PRIVATE boolean_T SendPktHdrToHost(
    const ExtModeAction action,
    const int           size)  /* # of bytes to follow pkt header */
{
    int_T     nSet;
    PktHeader pktHdr;
    boolean_T error = EXT_NO_ERROR;

    pktHdr.type = (uint32_T)action;
    pktHdr.size = size;

    error = ExtSetHostPkt(extUD,sizeof(pktHdr),(char_T *)&pktHdr,&nSet);
    if (error || (nSet != sizeof(pktHdr))) {
        error = EXT_ERROR;
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,"ExtSetHostPkt() failed.\n");
#endif
        goto EXIT_POINT;
    }

EXIT_POINT:
    return(error);
} /* end SendPktHdrToHost */


/* Function: SendPktDataToHost =================================================
 * Abstract:
 *  Send packet data to host. You are responsible for sending a header
 *  prior to sending the header.
 */
PRIVATE boolean_T SendPktDataToHost(const char *data, const int size)
{
    int_T     nSet;
    boolean_T error = EXT_NO_ERROR;

    error = ExtSetHostPkt(extUD,size,data,&nSet);
    if (error || (nSet != size)) {
        error = EXT_ERROR;
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,"ExtSetHostPkt() failed.\n");
#endif
        goto EXIT_POINT;
    }

EXIT_POINT:
    return(error);
} /* end SendPktDataToHost */


/* Function: SendPktToHost =====================================================
 * Abstract:
 *  Send a packet to the host.  Packets can be of two forms:
 *      o packet header only
 *          the type is used as a flag to notify Simulink of an event
 *          that has taken place on the target (event == action == type)
 *      o pkt header, followed by data
 */
PUBLIC boolean_T SendPktToHost(
    const ExtModeAction action,
    const int           size,  /* # of bytes to follow pkt header */
    const char          *data)
{
    boolean_T error = EXT_NO_ERROR;
    
#ifdef VXWORKS
    semTake(pktSem, WAIT_FOREVER);
#endif

    error = SendPktHdrToHost(action,size);
    if (error != EXT_NO_ERROR) goto EXIT_POINT;

    if (data != NULL) {
        error = SendPktDataToHost(data, size);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
    } else {
        assert(size == 0);
    }

EXIT_POINT:
#ifdef VXWORKS
    semGive(pktSem);
#endif
    return(error);
} /* end SendPktToHost */


/* Function:  SendResponseStatus ===============================================
 *  
 */
PRIVATE boolean_T SendResponseStatus(const ExtModeAction  response,
                                     const ResponseStatus status,
                                     int32_T upInfoIdx)
{
    int32_T   msg[2];
    boolean_T error = EXT_NO_ERROR;

    msg[0] = (int32_T)status;
    msg[1] = upInfoIdx;

    error = SendPktToHost(response,2*sizeof(int32_T),(char_T *)&msg);
    return(error);

} /* end SendResponseStatus */


#ifndef EXTMODE_DISABLEPARAMETERTUNING
/* Function: ProcessSetParamPkt ================================================
 * Receive and process the EXT_SETPARAM packet.
 */
PRIVATE boolean_T ProcessSetParamPkt(RTWExtModeInfo *ei,
                                     const int pktSize)
{
    int32_T    msg;
    const char *pkt;
    boolean_T  error = EXT_NO_ERROR;

    /*
     * Receive packet and set parameters.
     */
    
#ifdef DAEMON_MODE
    pkt = GetPktUsingAck(pktSize);
#else
    pkt = GetPkt(pktSize);
#endif

    if (pkt == NULL) {
        msg = (int32_T)NOT_ENOUGH_MEMORY;
        SendPktToHost(EXT_SETPARAM_RESPONSE,sizeof(int32_T),(char_T *)&msg);
        error = EXT_ERROR; 
        goto EXIT_POINT;
    }
    SetParam(ei, pkt);

    msg = (int32_T)STATUS_OK;
    error = SendPktToHost(EXT_SETPARAM_RESPONSE,sizeof(int32_T),(char_T *)&msg);
    if (error != EXT_NO_ERROR) goto EXIT_POINT;

EXIT_POINT:
    return(error);
} /* end ProcessSetParamPkt */
#endif /* ifndef EXTMODE_DISABLEPARAMETERTUNING */

#ifndef EXTMODE_DISABLEPARAMETERTUNING 
/* Function: ProcessGetParamsPkt ===============================================
 *  Respond to the hosts request for the parameters by gathering up all the
 *  params and sending them to the host.
 */
PRIVATE boolean_T ProcessGetParamsPkt(RTWExtModeInfo *ei)
{
    int_T                         i;
    int_T                         nBytesTotal;
    boolean_T                     error    = EXT_NO_ERROR;
    const DataTypeTransInfo       *dtInfo  = rteiGetModelMappingInfo(ei);
    const DataTypeTransitionTable *dtTable = dtGetParamDataTypeTrans(dtInfo);

    if (dtTable != NULL) {
        /*
         * We've got some params in the model.  Send their values to the
         * host.
         */
        int_T        nTrans   = dtGetNumTransitions(dtTable);
        const uint_T *dtSizes = dtGetDataTypeSizes(dtInfo);

 #ifdef VERBOSE
    #ifndef EXTMODE_DISABLEPRINTF             
        printf("\nUploading initial parameters....\n");
 #endif
#endif 

        /*
         * Take pass 1 through the transitions to figure out how many
         * bytes we're going to send.
         */
        nBytesTotal = 0;
        for (i=0; i<nTrans; i++) {
            int_T dt     = dtTransGetDataType(dtTable, i);
            int_T dtSize = dtSizes[dt];
            int_T nEls   = dtTransNEls(dtTable, i); /* complexity accounted for in trans tbl num of els */
            int_T nBytes = dtSize * nEls;

            nBytesTotal += nBytes;
        }

        /*
         * Send the packet header.
         */
        error = SendPktHdrToHost(EXT_GETPARAMS_RESPONSE,nBytesTotal);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;

        /*
         * Take pass 2 through the transitions and send the parameters.
         */
        for (i=0; i<nTrans; i++) {
            char_T *tranAddress  = dtTransGetAddress(dtTable, i);
            int_T  dt            = dtTransGetDataType(dtTable, i);
            int_T  dtSize        = dtSizes[dt];
            int_T  nEls          = dtTransNEls(dtTable, i); /* complexity accounted for in trans tbl num of els */
            int_T  nBytes        = dtSize * nEls;

            error = SendPktDataToHost(tranAddress, nBytes);
            if (error != EXT_NO_ERROR) goto EXIT_POINT;
        }
    } else {
        /*
         * We've got no params in the model.
         */
        error = SendPktHdrToHost(EXT_GETPARAMS_RESPONSE,0);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
    }

EXIT_POINT:
    return(error);
} /* end ProcessGetParamsPkt */
#endif /* ifndef EXTMODE_DISABLEPARAMETERTUNING */


#ifndef EXTMODE_DISABLESIGNALMONITORING
/* Function: ProcessSelectTriggerSignalPkt ===========================================
 * Receive and process the EXT_SELECT_TRIGGER or EXT_SELECT_SIGNALS packet.
 */
PRIVATE boolean_T ProcessSelectTriggerSignalPkt(const ExtModeAction ACTION_ID, 
                                                RTWExtModeInfo *ei,
                                                const int pktSize,
                                                int_T numSampTimes,
                                                char* errMsg)
{
    const char *pkt;
    int32_T    upInfoIdx;
    boolean_T  error = EXT_NO_ERROR;

#ifdef DAEMON_MODE
    pkt = GetPktUsingAck(pktSize);
#else
    pkt = GetPkt(pktSize);
#endif

    if (pkt == NULL) {
        SendResponseStatus(ACTION_ID, NOT_ENOUGH_MEMORY, -1);
        return(EXT_ERROR);
    }

    (void)memcpy(&upInfoIdx, pkt, sizeof(int32_T)); /* Extract upInfoIdx */
    switch(ACTION_ID) {
    case EXT_SELECT_TRIGGER_RESPONSE:
#ifndef EXTMODE_DISABLEPRINTF  
        PRINT_VERBOSE(
                    ("got EXT_SELECT_TRIGGER packet for upInfoIdx : %d\n", upInfoIdx));
#endif
        error = UploadInitTrigger(ei, pkt+sizeof(int32_T), upInfoIdx);
        break;
    case EXT_SELECT_SIGNALS_RESPONSE:
#ifndef EXTMODE_DISABLEPRINTF  
        PRINT_VERBOSE(
                    ("got EXT_SELECT_SIGNALS packet for upInfoIdx : %d\n", upInfoIdx));
#endif
        error = UploadLogInfoInit(ei, numSampTimes, pkt+sizeof(int32_T), upInfoIdx);
        break;
    default:
        break;
    }

    if (error != EXT_NO_ERROR) {
        SendResponseStatus(ACTION_ID, NOT_ENOUGH_MEMORY, upInfoIdx);
#ifndef EXTMODE_DISABLEPRINTF            
        printf("%s\n", errMsg);
#endif
        return(error);
    }

    error = SendResponseStatus(ACTION_ID, STATUS_OK, upInfoIdx);
    return(error); /* Can be EXT_NO_ERROR */
} /* end ProcessSelectTriggerSignalPkt */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */


#ifndef EXTMODE_DISABLESIGNALMONITORING
/* Function: ProcessCancelLoggingArmTriggerPkt ===========================================
 * Receive and process the EXT_CANCEL_LOGGING or EXT_ARM_TRIGGER packet.
 */
PRIVATE boolean_T ProcessCancelLoggingArmTriggerPkt(const ExtModeAction ACTION_ID, 
                        const int pktSize, 
                        int_T numSampTimes)
{
    const char *pkt;
    int32_T    upInfoIdx;
    boolean_T  error = EXT_NO_ERROR;

#ifdef DAEMON_MODE
    pkt = GetPktUsingAck(pktSize);
#else
    pkt = GetPkt(pktSize);
#endif

    if (pkt == NULL) {
        SendResponseStatus(ACTION_ID, NOT_ENOUGH_MEMORY, -1);
        return(EXT_ERROR);
    }
            
    (void)memcpy(&upInfoIdx, pkt, sizeof(int32_T)); /* Extract upInfoIdx */
        
    switch(ACTION_ID) {
    case EXT_CANCEL_LOGGING_RESPONSE:
#ifndef EXTMODE_DISABLEPRINTF   
        PRINT_VERBOSE(
                ("got EXT_CANCEL_LOGGING packet for upInfoIdx : %d\n", upInfoIdx));
#endif
        UploadCancelLogging(upInfoIdx);
        break;
    case EXT_ARM_TRIGGER_RESPONSE:
#ifndef EXTMODE_DISABLEPRINTF
        PRINT_VERBOSE(
                ("got EXT_ARM_TRIGGER packet for upInfoIdx : %d\n", upInfoIdx));
#endif
        UploadArmTrigger(upInfoIdx, numSampTimes);
        break;
    default:
        break;
    }

    error = SendResponseStatus(ACTION_ID, STATUS_OK, upInfoIdx);
    return(error); /* Can be EXT_NO_ERROR */
} /* end ProcessCancelLoggingArmTriggerPkt */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */


#ifdef EXTMODE_DISABLEPARAMETERTUNING
PRIVATE boolean_T AcknowledgeSetParamPkt(const int pktSize)
{
    int32_T    msg;
    const char *pkt;
    boolean_T  error = EXT_NO_ERROR;

    pkt = GetPkt(pktSize);
    msg = (int32_T)STATUS_OK;
    error = SendPktToHost(EXT_SETPARAM_RESPONSE, sizeof(int32_T), (char_T *)&msg);
    return(error);
}
#endif /* ifdef EXTMODE_DISABLEPARAMETERTUNING */

#ifdef EXTMODE_DISABLESIGNALMONITORING
PRIVATE boolean_T AcknowledgeSignalActionPkt(const int pktSize, const ExtModeAction ACTION_ID)
{
    const char *pkt;
    int32_T    upInfoIdx;
    boolean_T  error = EXT_NO_ERROR;

    pkt = GetPkt(pktSize);
    (void)memcpy(&upInfoIdx, pkt, sizeof(int32_T));
    error = SendResponseStatus(ACTION_ID, STATUS_OK, upInfoIdx);
    return(error);
} 
#endif /* ifdef EXTMODE_DISABLESIGNALMONITORING */

/*********************
 * Visible Functions *
 *********************/


/* Function: ExtParseArgsAndInitUD =============================================
 * Abstract:
 *  Pass remaining arguments (main program should have NULL'ed out any args
 *  that it processed) to external mode.
 *  
 *  The actual, transport-specific parsing routine (implemented in
 *  ext_svr_transport.c) MUST NULL out all entries of argv that it processes.
 *  The main program depends on this in order to determine if any unhandled
 *  command line options were specified (i.e., if the main program detects
 *  any non-null fields after the parse, it throws an error).
 *
 *  Returns an error string on failure, NULL on success.
 *
 * NOTES:
 *  The external mode UserData is created here so that the specified command-
 *  line options can be stored.
 */
PUBLIC const char_T *ExtParseArgsAndInitUD(const int_T  argc,
                                           const char_T *argv[])
{
    const char_T *error = NULL;
    
    /*
     * Create the user data.
     */
    extUD = ExtUserDataCreate();
    if (extUD == NULL) {
        error = "Could not create external mode user data.  Out of memory.\n";
        goto EXIT_POINT;
    }

    /*
     * Parse the transport-specific args.
     */
    error = ExtProcessArgs(extUD,argc,argv);
    if (error != NULL) goto EXIT_POINT;
        
EXIT_POINT:
    if (error != NULL) {
        ExtUserDataDestroy(extUD);
        extUD = NULL;
    }
    return(error);
} /* end ExtParseArgsAndInitUD */


/* Function: ExtWaitForStartPkt ================================================
 * Abstract:
 *  Return true if waiting for host to tell us when to start.
 */
PUBLIC boolean_T ExtWaitForStartPkt(void)
{
    return(ExtWaitForStartPktFromHost(extUD));
} /* end ExtWaitForStartPkt */


#ifndef EXTMODE_DISABLESIGNALMONITORING
/* Function: UploadServerWork =================================================
 * Abstract:
 *  Upload model signals to host for a single upInfo.
 */
void UploadServerWork(int32_T upInfoIdx, int_T numSampTimes)
{
    int_T         i;
    ExtBufMemList upList;
    boolean_T     error = EXT_NO_ERROR;

#ifdef VXWORKS
    /*
     * Don't spin the CPU unless we've got data to upload.
     * The upload.c/UploadBufAddTimePoint function gives the sem
     * each time that data is added.
     */
taskUnsafe();
    semTake(uploadSem, WAIT_FOREVER);
taskSafe();
#endif

    if (!connected) goto EXIT_POINT;
    
    UploadBufGetData(&upList, upInfoIdx, numSampTimes);
    while(upList.nActiveBufs > 0) {
        for (i=0; i<upList.nActiveBufs; i++) {
            const BufMem *bufMem = &upList.bufs[i];

            /*
             * We call SendPktDataToHost() instead of SendPktToHost() because
             * the packet header is combined with packet payload.  We do this
             * to avoid the overhead of making two calls for each upload
             * packet - one for the head and one for the payload.
             */
            error = SendPktDataToHost(
                bufMem->section1,
                bufMem->nBytes1);
            if (error != EXT_NO_ERROR) {
#ifndef EXTMODE_DISABLEPRINTF                    
                fprintf(stderr,"SendPktDataToHost() failed on data upload.\n");
#endif
                goto EXIT_POINT;
            }
            
            if (bufMem->nBytes2 > 0) {

                error = SendPktDataToHost(
                    bufMem->section2,
                    bufMem->nBytes2);
                if (error != EXT_NO_ERROR) {
#ifndef EXTMODE_DISABLEPRINTF                        
                    fprintf(stderr,"SendPktDataToHost() failed on data upload.\n");
#endif
                    goto EXIT_POINT;
                }
            }
            /* confirm that the data was sent */
            UploadBufDataSent(upList.tids[i], upInfoIdx);
        }
        UploadBufGetData(&upList, upInfoIdx, numSampTimes);
    }
    
EXIT_POINT:
    if (error != EXT_NO_ERROR) {
        /* An error in this function is caused by a physical failure in the
         * external mode connection.  We assume this failure caused the host
         * to disconnect.  The target must be disconnected and returned to a
         * state where it is running and can be re-connected to by the host.
         */
        ForceDisconnectFromHost(numSampTimes);
    }
}
/* end UploadServerWork */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */

#ifndef EXTMODE_DISABLESIGNALMONITORING
/* Function: rt_UploadServerWork ===============================================
 * Abstract:
 *  Wrapper function that calls UploadServerWork once for each upInfo
 */
PUBLIC void rt_UploadServerWork(int_T numSampTimes)
{
    int i;
    
    for (i=0; i<NUM_UPINFOS; i++) {
        UploadServerWork(i, numSampTimes);
    }
} /* end rt_UploadServerWork */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */

/* Function: rt_ExtModeInit ====================================================
 * Abstract:
 *  Called once at program startup to do any initialization related to external
 *  mode. 
 */
PUBLIC boolean_T rt_ExtModeInit(void)
{
    int i;
    boolean_T error = EXT_NO_ERROR;

#ifdef TMW_EXTMODE_TESTING_REQ
#  ifndef TMW_EXTMODE_TESTING
#ifndef EXTMODE_DISABLEPRINTF 
    fprintf(stderr,"Error: External mode tests should use the external mode test harness.\n");
#endif
    error = EXT_ERROR;
#  endif
#endif
    if (error != EXT_NO_ERROR) goto EXIT_POINT;

    error = ExtInit(extUD);
    if (error != EXT_NO_ERROR) goto EXIT_POINT;

    for (i=0; i<NUM_UPINFOS; i++) {
        UploadLogInfoReset(i);
    }

    rtExtModeTestingInit();

EXIT_POINT:
    return(error);
} /* end rt_ExtModeInit */


/* Function: rt_ExtModeSleep ===================================================
 * Abstract:
 *  Called by grt_main, ert_main, and grt_malloc_main  to "pause".  It attempts
 *  to do this in a way that does not hog the processor.
 */
#ifndef VXWORKS
PUBLIC void rt_ExtModeSleep(
    long sec,  /* number of seconds to wait       */
    long usec) /* number of micro seconds to wait */
{
    ExtModeSleep(extUD,sec,usec);
} /* end rt_ExtModeSleep */
#endif


/* Function: rt_PktServerWork ==================================================
 * Abstract:
 *  If not connected, establish communication of the packet line and the
 *  data upload line.  If connected, send/receive packets and parameters
 *  on the packet line.
 */
PUBLIC void rt_PktServerWork(RTWExtModeInfo *ei,
                             int_T          numSampTimes,
                             boolean_T      *stopReq)
{
    PktHeader  pktHdr;
    boolean_T  hdrAvail;
    boolean_T  error             = EXT_NO_ERROR;
    boolean_T  disconnectOnError = false;
    
    /*
     * If not connected, attempt to make connection to host.
     */
    if (!connected) {
        rtExtModeTestingKillIfOrphaned(false);

        error = ExtOpenConnection(extUD,&connected);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
    }

    /*
     * If ExtOpenConnection is not blocking and there are no pending
     * requests to open a connection, we'll still be unconnected.
     */
    if (!connected) goto EXIT_POINT; /* nothing to do */
    
    /*
     * Process packets.
     */

    /* Wait for a packet. */
    error = GetPktHdr(&pktHdr, &hdrAvail);
    if (error != EXT_NO_ERROR) {
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr, "\nError occurred getting packet header.\n");
#endif
        disconnectOnError = true;
        goto EXIT_POINT;
    }
    rtExtModeTestingKillIfOrphaned(hdrAvail);
    
    if (!hdrAvail) goto EXIT_POINT; /* nothing to do */

    /*
     * This is the first packet.  Should contain the string:
     * 'ext-mode'.  Its contents are not important to us.
     * It is used as a flag to start the handshaking process.
     */
    if (!commInitialized) {
        pktHdr.type = EXT_CONNECT;
    }

    /* 
     * At this point we know that we have a packet: process it.
     */
#ifdef VXWORKS
    taskSafe();
#endif
    switch(pktHdr.type) {

    case EXT_GET_TIME:
    {
        /* Skip verbosity print out - we get too many of these */
        /*PRINT_VERBOSE(("got EXT_GET_TIME packet.\n"));*/
        time_T t = rteiGetT(ei);
        
        error = SendPktToHost(
            EXT_GET_TIME_RESPONSE,sizeof(time_T),
            (char_T *)&t);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
        break;
    }

    case EXT_ARM_TRIGGER:
    {
#ifndef EXTMODE_DISABLESIGNALMONITORING
        error = ProcessCancelLoggingArmTriggerPkt(EXT_ARM_TRIGGER_RESPONSE, pktHdr.size, numSampTimes);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#else
        error = AcknowledgeSignalActionPkt(pktHdr.size, EXT_ARM_TRIGGER_RESPONSE);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;
    }

    case EXT_SELECT_SIGNALS:
    {
#ifndef EXTMODE_DISABLESIGNALMONITORING
        error = ProcessSelectTriggerSignalPkt(EXT_SELECT_SIGNALS_RESPONSE, ei, pktHdr.size, numSampTimes, ERRMSG_PROCESSSELECTSIGNAL);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#else
        error = AcknowledgeSignalActionPkt(pktHdr.size, EXT_SELECT_SIGNALS_RESPONSE);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;
    }

    case EXT_SELECT_TRIGGER: 
    {
#ifndef EXTMODE_DISABLESIGNALMONITORING
        error = ProcessSelectTriggerSignalPkt(EXT_SELECT_TRIGGER_RESPONSE, ei, pktHdr.size, -1, ERRMSG_PROCESSSELECTTRIGGER);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#else
        error = AcknowledgeSignalActionPkt(pktHdr.size, EXT_SELECT_TRIGGER_RESPONSE);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;
    }

    case EXT_CONNECT:
    {
        PRINT_VERBOSE(("got EXT_CONNECT packet.\n"));
        error = ProcessConnectPkt(ei);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
        break;
    }

    case EXT_SETPARAM:
    {
#ifndef EXTMODE_DISABLEPARAMETERTUNING
        PRINT_VERBOSE(("got EXT_SETPARAM packet.\n"));
        if (pktHdr.size == 0) {
            gblSetParamPktReceived = true;
        } else {
            error = ProcessSetParamPkt(ei, pktHdr.size);
            if (error != EXT_NO_ERROR) {
                goto EXIT_POINT;
            }
            else {
                gblSetParamPktReceived = true;
            }
        }
#else
        PRINT_VERBOSE(("discard EXT_SETPARAM packet.\n"));
        error = AcknowledgeSetParamPkt(pktHdr.size);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;
    }

    case EXT_GETPARAMS:
    {
#ifndef EXTMODE_DISABLEPARAMETERTUNING
        PRINT_VERBOSE(("got EXT_GETPARAMS packet.\n"));
        error = ProcessGetParamsPkt(ei);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;
    }

    case EXT_DISCONNECT_REQUEST:
    {
        PRINT_VERBOSE(("got EXT_DISCONNECT_REQUEST packet.\n"));
        
        /*
         * Note that from the target's point of view this is
         * more a "notify" than a "request".  The host needs to
         * have this acknowledged before it can begin closing
         * the connection.
         */
        error = SendPktToHost(EXT_DISCONNECT_REQUEST_RESPONSE, 0, NULL);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;

        DisconnectFromHost(numSampTimes);

        break;
    }

    case EXT_DISCONNECT_REQUEST_NO_FINAL_UPLOAD:
    {
        PRINT_VERBOSE(("got EXT_DISCONNECT_REQUEST_NO_FINAL_UPLOAD packet.\n"));
        
        /*
         * The target receives this packet when the host is
         * immediately terminating the extmode communication due
         * to some error.  The target should not send back a
         * response or a final upload of data because the host is
         * expecting neither.  The target must be disconnected and
         * returned to a state where it is running and can be
         * re-connected to by the host.
         */
        ForceDisconnectFromHost(numSampTimes);

        break;
    }

    case EXT_MODEL_START:
        PRINT_VERBOSE(("got EXT_MODEL_START packet.\n"));
#ifdef VXWORKS
        {
            extern SEM_ID startStopSem;
            semGive(startStopSem);
        }
#endif
        startModel = true;
        error = SendPktToHost(EXT_MODEL_START_RESPONSE, 0, NULL);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
        break;

    case EXT_MODEL_STOP:
        PRINT_VERBOSE(("got EXT_MODEL_STOP packet.\n"));
        *stopReq = true;
        break;
#ifndef EXTMODE_DISABLETESTING
    case EXT_MODEL_PAUSE:
        PRINT_VERBOSE(("got EXT_MODEL_PAUSE packet.\n"));
        modelStatus = TARGET_STATUS_PAUSED;
        startModel  = false;

        error = SendPktToHost(EXT_MODEL_PAUSE_RESPONSE, 0, NULL);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
        break;
    case EXT_MODEL_STEP:
        PRINT_VERBOSE(("got EXT_MODEL_STEP packet.\n"));
        if ((modelStatus == TARGET_STATUS_PAUSED) && !startModel) {
            startModel = true;
        }
        
        error = SendPktToHost(EXT_MODEL_STEP_RESPONSE, 0, NULL);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
        break;
    case EXT_MODEL_CONTINUE:
        PRINT_VERBOSE(("got EXT_MODEL_CONTINUE packet.\n"));
        if (modelStatus == TARGET_STATUS_PAUSED) {
            modelStatus = TARGET_STATUS_RUNNING;
            startModel  = false;
        }
        
        error = SendPktToHost(EXT_MODEL_CONTINUE_RESPONSE, 0, NULL);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;

    case EXT_CANCEL_LOGGING:
    {
#ifndef EXTMODE_DISABLESIGNALMONITORING
        error = ProcessCancelLoggingArmTriggerPkt(EXT_CANCEL_LOGGING_RESPONSE, pktHdr.size, numSampTimes);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#else
        error = AcknowledgeSignalActionPkt(pktHdr.size, EXT_CANCEL_LOGGING_RESPONSE);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
#endif
        break;
    }

    default:
#ifndef EXTMODE_DISABLEPRINTF            
        fprintf(stderr,"received invalid packet.\n");
#endif
        break;
    } /* end switch */

EXIT_POINT:
    if (error != EXT_NO_ERROR) {
        if (disconnectOnError) {
#ifndef EXTMODE_DISABLEPRINTF                
            fprintf(stderr,
                "Error occurred in rt_PktServerWork.\n"
                "Disconnecting from host!\n");
#endif
            /* An error in this function which causes disconnectOnError to be
             * set to true is caused by a physical failure in the external mode
             * connection.  We assume this failure caused the host to disconnect.
             * The target must be disconnected and returned to a state
             * where it is running and can be re-connected to by the host.
             */
            ForceDisconnectFromHost(numSampTimes);
        }
    }
#ifdef VXWORKS
    taskUnsafe();
#endif
} /* end rt_PktServerWork */


/* Function: rt_PktServer ======================================================
 * Abstract:
 *  Call rt_PktServerWork forever.   Used only for RTOS (e.g., Tornado/VxWorks
 *  when running as a low priority task.
 */
#ifdef VXWORKS
PUBLIC void rt_PktServer(RTWExtModeInfo *ei,
                         int_T          numSampTimes,
                         boolean_T      *stopReq)
{
    for(;;) {
        rt_PktServerWork(ei,numSampTimes,stopReq); 
    }
}
#endif


/* Function: rt_UploadServer ===================================================
 * Abstract:
 *  Call rt_UploadServerWork forever.   Used only for RTOS (e.g.,
 *  Tornado/VxWorks when running as a low priority task.
 */
#ifdef VXWORKS
#ifndef EXTMODE_DISABLESIGNALMONITORING
PUBLIC void rt_UploadServer(int_T numSampTimes)
{
    for(;;) {
        rt_UploadServerWork(numSampTimes);
    }
} /* end rt_UploadServer */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */
#endif


/* Function: rt_SetPortInExtUD =================================================
 * Abstract:
 *  Set the port in the external mode user data structure.
 */
#ifdef VXWORKS
PUBLIC void rt_SetPortInExtUD(const int_T port)
{
    ExtUserDataSetPort(extUD, port);
} /* end rt_SetPortInExtUD */
#endif

/* Function: ExtModeShutdown ==================================================
 * Abstract:
 *  Called when target program terminates to enable cleanup of external 
 *  mode for a given upInfo.
 */
PRIVATE void ExtModeShutdown(int32_T upInfoIdx, int_T numSampTimes)
{
    /*
     * Make sure buffers are flushed so that the final points get to
     * host (this is important for the case of the target reaching tfinal
     * while data uploading is in progress).
     */
    UploadPrepareForFinalFlush(upInfoIdx);
#ifndef EXTMODE_DISABLESIGNALMONITORING
    UploadServerWork(upInfoIdx, numSampTimes);
#endif

    UploadLogInfoTerm(upInfoIdx, numSampTimes);

    if (pktBuf != NULL) {
        free(pktBuf);
        pktBuf = NULL;
    }

} /* end ExtModeShutdown */

/* Function: rt_ExtModeShutdown ================================================
 * Abstract:
 *  Called when target program terminates to enable cleanup of external 
 *  mode.
 */
PUBLIC boolean_T rt_ExtModeShutdown(int_T numSampTimes)
{
    int i;
    boolean_T error = EXT_NO_ERROR;

    for (i=0; i<NUM_UPINFOS; i++) {
        ExtModeShutdown(i, numSampTimes);
    }

    if (commInitialized) {
        error = SendPktToHost(EXT_MODEL_SHUTDOWN, 0, NULL);
        if (error != EXT_NO_ERROR) {
#ifndef EXTMODE_DISABLEPRINTF                
            fprintf(stderr,
                "\nError sending EXT_MODEL_SHUTDOWN packet to host.\n");
#endif
        }
        commInitialized = false;
    }
    if (connected) {
        connected = false;
        modelStatus = TARGET_STATUS_WAITING_TO_START;        
    }

    ExtShutDown(extUD);
    ExtUserDataDestroy(extUD);
    
    rtExtModeTestingRemoveBatMarker();
    
    return(error);
} /* end rt_ExtModeShutdown */

#ifndef EXTMODE_DISABLESIGNALMONITORING
/* Function: rt_UploadCheckTrigger =============================================
 * Abstract:
 *  Wrapper function that calls UploadCheckTrigger once for each upInfo
 */
PUBLIC void rt_UploadCheckTrigger(int_T numSampTimes)
{
    int i;
    for (i=0; i<NUM_UPINFOS; i++) {
        UploadCheckTrigger(i, numSampTimes);
    }
} /* end rt_UploadCheckTrigger */

/* Function: rt_UploadCheckEndTrigger ==========================================
 * Abstract:
 *  Wrapper function that calls UploadCheckTrigger once for each upInfo
 */
PUBLIC void rt_UploadCheckEndTrigger(void)
{
    int i;
    
    for (i=0; i<NUM_UPINFOS; i++) {
        UploadCheckEndTrigger(i);
    }
} /* end rt_UploadCheckEndTrigger */

/* Function: rt_UploadBufAddTimePoint ==========================================
 * Abstract:
 *  Wrapper function that calls UploadBufAddTimePoint once for each upInfo
 */
PUBLIC void rt_UploadBufAddTimePoint(int_T tid, real_T taskTime)
{
    int i;
    
    for (i=0; i<NUM_UPINFOS; i++) {
        UploadBufAddTimePoint(tid, taskTime, i);
    }
} /* end rt_UploadBufAddTimePoint */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */

/* [EOF] ext_svr.c */
