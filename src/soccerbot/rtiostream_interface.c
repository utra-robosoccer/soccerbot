/*
 * Copyright 1994-2014 The MathWorks, Inc.
 *
 * File: rtiostream_interface.c     
 *
 * Abstract: 
 * Provide a target-side communications driver interface for Simulink external
 * mode.
 */


/***************** TRANSPORT-INDEPENDENT INCLUDES *****************************/

#ifndef EXTMODE_DISABLEPRINTF  
#include <stdio.h>
#endif

#include <stdlib.h>
#include <string.h>

#include "updown_util.h"
#include "rtwtypes.h"
#include "rtw_extmode.h"

/* include rtIOStream interface to use */
#include "rtiostream.h"

/* include Target-side rtIOStream utility APIs */
#include "rtiostream_utils.h"

#ifdef VXWORKS
# include <sockLib.h>
# include <inetLib.h>
# include <selectLib.h>
extern SEM_ID commSem;
#endif

#ifdef LEGO
#include "ecrobot_interface.h"
#endif

#include "ext_types.h"
#include "ext_share.h"

/* Logical definitions */
#if (!defined(__cplusplus))
#  ifndef false
#   define false                       (0U)
#  endif
#  ifndef true
#   define true                        (1U)
#  endif
#endif

/* 
 * ExtCloseConnection() sets this to FALSE (to signal the low layer rtiostream)
 * on an external mode disconnection.
 * 
 * rtIOStreamRecv() reads a few spurious bytes before receiving valid data on its
 * first call during external mode start or connect. Setting receivedSyncByteE
 * to FALSE forces rtIOStreamRecv() to look for the first character 'e' in 'ext-mode',
 * which is the first valid external mode packet. All preceding bytes are rejected and
 * subsequent bytes including the string 'ext-mode' are forwarded.
 * 
 * For a more detailed description refer to the
 * comments in the associated rtiostream_serial_daemon.c/cpp file.
 */
#if (defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_SAM_DUE)) && !defined(_ROTH_LEONARDO_)
extern volatile boolean_T receivedSyncByteE;
#endif

typedef struct ExtUserData_tag {
    boolean_T waitForStartPkt;
    int streamID;
#ifdef VXWORKS 
    int_T processArgsArgc;
    const char_T ** processArgsArgv;
#endif
} ExtUserData;


/* Function: ExtProcessArgs ====================================================
 * Abstract:
 *  Process the arguments specified by the user when invoking the target
 *  program.  In the case of this TCPIP example the args handled by external
 *  mode are:
 *      o -port #
 *          specify tcpip port number
 *      
 *      o -w
 *          wait for a start packet from the target before starting to execute
 *          the real-time model code
 *
 *  If any unrecognized options are encountered, ignore them.
 *
 * NOTES:
 *  o An error string is returned on failure, NULL is returned on success.
 *
 *  o IMPORTANT!!!
 *    As the arguments are processed, their strings must be NULL'd out in
 *    the argv array.  ext_svr will search argv when this function returns,
 *    and if any non-NULL entries are encountered an "unrecognized option" 
 *    packet will be displayed.
 */
PUBLIC const char_T *ExtProcessArgs(
    ExtUserData   *UD,
    const int_T   argc,
    const char_T  *argv[])
{
    const char_T *error          = NULL;
    int_T        count           = 1;

#if defined(ON_TARGET_WAIT_FOR_START) && ON_TARGET_WAIT_FOR_START == 1
    boolean_T    waitForStartPkt = true;
#else
    boolean_T    waitForStartPkt = false;
#endif

    while(count < argc) {
        const char_T *option = argv[count++];
        
        if (option == NULL) continue;

        if (strcmp(option, "-w") == 0) {
            /* 
             * -w (wait for packet from host) option
             */
            waitForStartPkt = true;
            
            argv[count-1] = NULL;
        } else if (strcmp(option, "-ignore-arg") == 0 && (count != argc)) {
            /*
             * Special argument used by sbruntests to mark an executable
             * with a particular ID allowing the executable to be killed
             * when the sbruntests session is killed.
             *
             * -ignore-arg SBRUNTESTS_SESSION_ID#
             */
            count++;     
            argv[count-2] = NULL;
            argv[count-1] = NULL;
        }
    }

    assert(UD != NULL);
    UD->waitForStartPkt = waitForStartPkt;

#ifdef VXWORKS
    /* Store argv and argc for processing in ExtUserDataSetPort. 
     * rtIOStreamOpen will be called in ExtUserDataSetPort */
    UD->processArgsArgc = argc;
    UD->processArgsArgv = (const char_T **) argv;
#else
    /* rtIOStreamOpen combines argument processing and opening the stream */
    UD->streamID = rtIOStreamOpen(argc, (void *) argv);
#endif    
    return(error);
} /* end ExtProcessArgs */


/* Function: ExtUserDataCreate =================================================
 * Abstract:
 *  Create the user data.
 */
PUBLIC ExtUserData *ExtUserDataCreate(void)
{
    static ExtUserData UD;

    return &UD;
} /* end ExtUserDataCreate */


/* Function: ExtInit ===========================================================
 * Abstract:
 *  Called once at program startup to do any initialization related to external
 *  mode.  For the TCPIP, example, a socket is created to listen for
 *  connection requests from the host.  EXT_NO_ERROR is returned on success,
 *  EXT_ERROR on failure.
 *
 * NOTES:
 *  o This function should not block.
 */
PUBLIC boolean_T ExtInit(ExtUserData *UD)
{
    boolean_T error      = EXT_NO_ERROR;

    UNUSED_PARAMETER(UD);


    /* rtIOStreamOpen has already been called so no further
     * action required here */

    return(error);
} /* end ExtInit */

/* Function: ExtSetHostPkt =====================================================
 * Abstract:
 *  Sets (sends) the specified number of bytes on the comm line.  As long as
 *  an error does not occur, this function is guaranteed to set the requested
 *  number of bytes.  The number of bytes set is returned via the 'nBytesSet'
 *  parameter.  EXT_NO_ERROR is returned on success, EXT_ERROR is returned on
 *  failure.
 *
 * NOTES:
 *  o it is always o.k. for this function to block if no room is available
 */
PUBLIC boolean_T ExtSetHostPkt(
    const ExtUserData *UD,
    const int         nBytesToSet,
    const char        *src,
    int               *nBytesSet)
{
    boolean_T errorCode = EXT_NO_ERROR;
	int_T rtIOStreamErrorStatus;
	*nBytesSet = 0;	/* assume */

    #ifdef VXWORKS
        semTake(commSem, WAIT_FOREVER);
    #endif

    /* Blocks until all requested outgoing data is sent */
    rtIOStreamErrorStatus = rtIOStreamBlockingSend(UD->streamID,
                                                   (const void * const) src,
                                                   (uint32_T) nBytesToSet);
    
    if (rtIOStreamErrorStatus == RTIOSTREAM_ERROR) {
        errorCode = EXT_ERROR;
    } else { 
        *nBytesSet = nBytesToSet;
    }
    
    #ifdef VXWORKS
        semGive(commSem);
    #endif

    return errorCode;

} /* end ExtSetHostPkt */

/* Function: ExtGetHostPkt =====================================================
 * Abstract:
 *  Attempts to get the specified number of bytes from the comm line.  The
 *  number of bytes read is returned via the 'nBytesGot' parameter.
 *  EXT_NO_ERROR is returned on success, EXT_ERROR is returned on failure.
 *
 * NOTES:
 *  o it is not an error for 'nBytesGot' to be returned as 0
 *  o not guaranteed to read total requested number of bytes
 */
PUBLIC boolean_T ExtGetHostPkt(
    const ExtUserData *UD,
    const int         nBytesToGet,
    int               *nBytesGot, /* out */
    char              *dst)       /* out */
{
    boolean_T error = EXT_NO_ERROR;
    int_T result;
    size_t stNBytesGot;

    result = rtIOStreamRecv(UD->streamID, dst, (size_t) nBytesToGet, &stNBytesGot);

    if (result == RTIOSTREAM_ERROR) {
        error = EXT_ERROR;
    } else {
        *nBytesGot = (int) stNBytesGot;
    }

    return(error);
} /* end ExtGetHostPkt */


/* Function: ExtWaitForStartPktFromHost ========================================
 * Abstract:
 *  Return true if the model should not start executing until told to do so
 *  by the host.
 */
PUBLIC boolean_T ExtWaitForStartPktFromHost(ExtUserData *UD)
{
    return(UD->waitForStartPkt);
} /* end ExtWaitForStartPktFromHost */


/* Function: ExtUserDataDestroy ================================================
 * Abstract:
 *  Destroy the user data.
 */
PUBLIC void ExtUserDataDestroy(ExtUserData *UD)
{
    UNUSED_PARAMETER(UD);
} /* end ExtUserDataDestroy */

/* Function: ExtUserDataSetPort ================================================
 * Abstract:
 *  Set the port in the external mode user data structure.
 */
#ifdef VXWORKS
PUBLIC void ExtUserDataSetPort(ExtUserData *UD, const int_T port)
{
#define PORT_NUM_STR_DEFAULT "00255\0"
#define PORT_NUM_STR_LEN 6 /* allocates extra space for sprintf below */
#define PORT_ARG_STR "-port"
    /* create an extended argc and argv that includes -port */    
    const int_T argc = UD->processArgsArgc + 2;
    const char_T * portArgStr = PORT_ARG_STR;
    char_T portNumStr[PORT_NUM_STR_LEN] = PORT_NUM_STR_DEFAULT;
    const char_T ** argv = (const char_T **) calloc(argc, sizeof(char_T *));
    int_T count;
    if (argv == NULL) {
#ifndef EXTMODE_DISABLEPRINTF        
       (void)fprintf(stderr,
                "ExtUserDataSetPort: Memory allocation error.\n");
#endif
       /* any unprocessed args in the original argv will be caught by caller */
       return;
    }
    /* set a non-default port if valid */ 
    if ( (port >=255) && (port <= 65535)) {
        sprintf(portNumStr, "%5d", port);
    }
    /* copy original argv */
    for (count = 0; count < UD->processArgsArgc; count++) {
      argv[count] = UD->processArgsArgv[count];
    }
    /* add port */
    argv[argc-2] = portArgStr;
    argv[argc-1] = (const char_T *) portNumStr;
        
    /* rtIOStreamOpen combines argument processing and opening the stream */
    UD->streamID = rtIOStreamOpen(argc, (void *) argv);

    /* We assume -port argument has been processed by rtIOStream.
     * Copy back other elements of processed argv which should have been set to NULL. */
    for (count = 0; count < UD->processArgsArgc; count++) {
       UD->processArgsArgv[count] = argv[count];
    }
    free(argv);
} /* end ExtUserDataSetPort */
#endif


/* Function: ExtModeSleep =====================================================
 * Abstract:
 *  This function is used if the target is in pause mode or waiting for a
 *  start packet from the host. By calling ExtModeSleep prior to checking 
 *  the receive buffer, we can prevent excessive CPU loading.
 */
#ifndef VXWORKS
PUBLIC void ExtModeSleep(
    const ExtUserData *UD,
    const long        sec,  /* # of secs to wait        */
    const long        usec) /* # of micros secs to wait */
{

    UNUSED_PARAMETER(UD);
    UNUSED_PARAMETER(sec);
    UNUSED_PARAMETER(usec);

    /* This function is intentionally left empty */

} /* end ExtModeSleep */
#endif


/* Function: ExtOpenConnection =================================================
 * Abstract:
 *  The rtIOStream driver must open a connection implicitly as required. Hence
 *  the implementation of this function is empty.
 */
PUBLIC boolean_T ExtOpenConnection(
    ExtUserData *UD,
    boolean_T   *outConnectionMade)
{
    boolean_T          error          = EXT_NO_ERROR;
    UNUSED_PARAMETER(UD);

    *outConnectionMade = 1;
    return error;

} /* end ExtOpenConnection */


/* Function: ExtForceDisconnect ================================================
 * Abstract:
 *  Called by rt_UploadServerWork() in ext_svr.c when there is an extmode
 *  communication error (e.g. a tcp/ip disconnection between the host and target
 *  caused by a cable problem or extremely high network traffic).  In this case,
 *  we want the target to disconnect from the host even if it can't communicate
 *  with the host because we assume that the communication problem caused the
 *  host to disconnect.  This function will perform all steps necessary to
 *  shutdown the communication and leave the target in a state ready to be
 *  reconnected.
 */
PUBLIC void ExtForceDisconnect(ExtUserData *UD)
{

    UNUSED_PARAMETER(UD);

} /* end ExtForceDisconnect */

/* Function: ExtCloseConnection ================================================
 * Abstract:
 *  Called when the target needs to disconnect from the host (disconnect
 *  procedure is initiated by the host).
 */
PUBLIC void ExtCloseConnection(ExtUserData *UD)
{
    UNUSED_PARAMETER(UD);
    
    #ifdef LEGO
    ecrobot_term_bt_connection();
    #endif
    
    #if (defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_SAM_DUE)) && !defined(_ROTH_LEONARDO_)
    receivedSyncByteE = false;
    #endif
            
} /* end ExtCloseConnection */


/* Function: ExtShutDown =======================================================
 * Abstract:
 *  Called when the target program is terminating.
 */
PUBLIC void ExtShutDown(ExtUserData *UD)
{
    rtIOStreamClose(UD->streamID);
} /* end ExtShutDown */

/* LocalWords:  NULL'd svr extmode tcp ip
 */
