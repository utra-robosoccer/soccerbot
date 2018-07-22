/*
 * Copyright 1994-2014 The MathWorks, Inc.
 *
 * File: ext_work.c     
 *
 * Abstract:
 *   
 */

#if !defined(EXTMODE_DISABLEPRINTF) || !defined(EXTMODE_DISABLE_ARGS_PROCESSING)
#include <stdio.h>
#endif

#include <stdlib.h>        /* for exit() */
#include <string.h>        /* optional for strcmp */

#include "rtwtypes.h"
#include "rtw_extmode.h"

#include "ext_types.h"
#include "ext_share.h"
#include "updown.h"
#include "ext_svr.h"
#include "ext_svr_transport.h"
#include "ext_work.h" /* includes all VxWorks headers */

/* Logical definitions */
#if (!defined(__cplusplus))
#  ifndef false
#   define false                       (0U)
#  endif
#  ifndef true
#   define true                        (1U)
#  endif
#endif

int_T           volatile startModel  = false;
TargetSimStatus volatile modelStatus = TARGET_STATUS_WAITING_TO_START;

#ifdef VXWORKS
SEM_ID volatile uploadSem = NULL;
SEM_ID volatile commSem   = NULL;
SEM_ID volatile pktSem    = NULL;
int_T           extern_pkt_tid;
int_T           extern_upload_tid;

void rtExtModeTornadoStartup(RTWExtModeInfo *ei,
                             int_T          numSampTimes,
                             boolean_T      *stopReqPtr,
                             int_T          priority,
                             int32_T        stack_size,
                             SEM_ID         startStopSem)
{
    uploadSem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
    commSem   = semBCreate(SEM_Q_PRIORITY, SEM_FULL);
    pktSem    = semBCreate(SEM_Q_PRIORITY, SEM_FULL);

    rt_ExtModeInit();

    extern_pkt_tid = taskSpawn("tExternPkt",
        priority+(numSampTimes), VX_FP_TASK, stack_size, (FUNCPTR)rt_PktServer, 
        (int_T) ei, (int_T) numSampTimes, (int_T) stopReqPtr, 0, 0, 0, 0, 0, 0, 0);
    if (extern_pkt_tid == ERROR) {
#ifndef EXTMODE_DISABLEPRINTF
        printf("handle taskpawn error");
#endif
    }

    extern_upload_tid = taskSpawn("tExternUpload",
        priority+(numSampTimes)+1,VX_FP_TASK, stack_size,(FUNCPTR)rt_UploadServer,
        (int_T) numSampTimes, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (extern_upload_tid == ERROR) {
#ifndef EXTMODE_DISABLEPRINTF
        printf("handle taskpawn error");
#endif
    }

    /*
     * Pause until receive model start packet - if external mode.
     * Make sure the external mode tasks are running so that 
     * we are listening for commands from the host.
     */
    if (ExtWaitForStartPkt()) {
#ifndef EXTMODE_DISABLEPRINTF
        printf("\nWaiting for start packet from host.\n");
#endif
        semTake(startStopSem, WAIT_FOREVER);
    }
    modelStatus = TARGET_STATUS_RUNNING;
}

void rtExtModeTornadoCleanup(int_T numSampTimes)
{
    taskDelete(extern_upload_tid);
    taskDelete(extern_pkt_tid);
    rt_ExtModeShutdown(numSampTimes);
    semDelete(uploadSem);
    semDelete(commSem);
    semDelete(pktSem);
}

/* Function ====================================================================
 *  Used by Tornado/VxWorks to set the port number used by external mode.
 *  Tornado does not parse the port number the same way as the other code
 *  formats.  Instead, the port number is specified as an argument to the
 *  spawn command which passes it into rt_main().  This function is called
 *  by rt_main() to set the port number in the external mode user data
 *  structure.
 */
void rtExtModeTornadoSetPortInExtUD(const int_T port)
{
    rt_SetPortInExtUD(port);
}

#else /* VXWORKS == 0 */

/* Function ====================================================================
 * Pause the process (w/o hogging the cpu) until receive step packet (which
 * means the startModel flag moves to true) or until we are no longer
 * in the paused state.  The packet/upload server must continue to process
 * events (otherwise the host would not be able to communicate with the target).
 */
void rtExtModePauseIfNeeded(RTWExtModeInfo *ei,
                            int_T          numSampTimes,
                            boolean_T      *stopReqPtr)
{
    while((modelStatus == TARGET_STATUS_PAUSED) && 
          !startModel && !(*stopReqPtr)) {
        rt_ExtModeSleep(0L, 375000L);
        rt_PktServerWork(ei,numSampTimes,stopReqPtr);
#ifndef EXTMODE_DISABLESIGNALMONITORING
        rt_UploadServerWork(numSampTimes);
#endif
    }
    startModel = false; /* reset to false - if we were stepped we want to
                         *                  stop again next time we get
                         *                  back here.
                         */
} /* end rtExtModePauseIfNeeded */

/* Function ====================================================================
 * Pause the process (w/o hogging the cpu) until receive start packet
 * from the host.  The packet/upload server must continue to process
 * events (otherwise the host would not be able to communicate with the target).
 */
void rtExtModeWaitForStartPkt(RTWExtModeInfo *ei,
                              int_T          numSampTimes,
                              boolean_T      *stopReqPtr)
{
    /*
     * Pause until receive model start packet.
     */
    if (ExtWaitForStartPkt()) {
        while(!startModel && !(*stopReqPtr)) {
            rt_ExtModeSleep(0L, 375000L);
            rt_PktServerWork(ei,numSampTimes,stopReqPtr);
#ifndef EXTMODE_DISABLESIGNALMONITORING
            rt_UploadServerWork(numSampTimes);
#endif
        }
    }
    if (modelStatus != TARGET_STATUS_PAUSED) {
        modelStatus = TARGET_STATUS_RUNNING;
    } else {
        /* leave in pause mode */
    }
}
#endif

void rtExtModeOneStep(RTWExtModeInfo *ei,
                      int_T          numSampTimes,
                      boolean_T      *stopReqPtr)
{
    /*
     * In a multi-tasking environment, this would be removed from the base rate
     * and called as a "background" task.
     */
    if (modelStatus != TARGET_STATUS_PAUSED) {
        rt_PktServerWork(ei,numSampTimes,stopReqPtr);
#ifndef EXTMODE_DISABLESIGNALMONITORING
        rt_UploadServerWork(numSampTimes);
#endif
    }
}

void rtExtModeUpload(int_T tid, real_T taskTime)
{
#ifndef EXTMODE_DISABLESIGNALMONITORING
    rt_UploadBufAddTimePoint(tid, taskTime);
#endif
}

void rtExtModeCheckEndTrigger(void)
{
#ifndef EXTMODE_DISABLESIGNALMONITORING
    rt_UploadCheckEndTrigger();
#endif
}

void rtExtModeUploadCheckTrigger(int_T numSampTimes)
{
#ifndef EXTMODE_DISABLESIGNALMONITORING
    rt_UploadCheckTrigger(numSampTimes);
#endif
}

void rtExtModeCheckInit(int_T numSampTimes)
{
    UNUSED_PARAMETER(numSampTimes);
    if (rt_ExtModeInit() != EXT_NO_ERROR){
#ifndef EXTMODE_DISABLEPRINTF            
            printf("Error calling rt_ExtModeInit!\n");
            exit(EXIT_FAILURE);         /*Error out if rt_ExtModeInit() returns EXT_ERROR. */
#endif
    }
}

void rtExtModeShutdown(int_T numSampTimes)
{
    rt_ExtModeShutdown(numSampTimes);
}

void rtExtModeParseArgs(int_T        argc, 
                        const char_T *argv[],
                        real_T       *unused)
{
    UNUSED_PARAMETER(unused);

#ifdef  EXTMODE_DISABLE_ARGS_PROCESSING
    /* Some targets do not support command line args, so bypass the args 
     * processing and directly call into ExtParseArgsAndInitUD.
     */

    /* initialize external mode */
    ExtParseArgsAndInitUD(0, NULL);

#else   

    /*
     * Parse the external mode arguments.
     */
    {
        const char_T *extParseErrorPkt = ExtParseArgsAndInitUD(argc, argv);
        if (extParseErrorPkt != NULL) {
#ifndef EXTMODE_DISABLEPRINTF            
            printf(
                "\nError processing External Mode command line arguments:\n");
            printf("\t%s",extParseErrorPkt);
#endif
            exit(EXIT_FAILURE);
        }
    }
#endif  /* EXTMODE_DISABLE_ARGS_PROCESSING */
}

/* Start of ERT specific functions and data */

static void displayUsage(void)
{
#ifndef EXTMODE_DISABLEPRINTF    
    (void) printf("usage: model_name -tf <finaltime> -w -port <TCPport>\n");
    (void) printf("arguments:\n");
    (void) printf("  -tf <finaltime> - overrides final time specified in "
                  "Simulink (inf for no limit).\n");
    (void) printf("  -w              - waits for Simulink to start model "
                  "in External Mode.\n");
    (void) printf("  -port <TCPport> - overrides 17725 default port in "
                  "External Mode, valid range 256 to 65535.\n");
#endif    
}

static const real_T RUN_FOREVER = (real_T)-1;
#if INTEGER_CODE == 0
static real_T finaltime = (real_T)-2; /* default to stop time in Sim Params. */
#else
static real_T finaltime = (real_T)-1; /* default to stop time inf */
#endif

void rtERTExtModeSetTFinal(real_T *rtmTFinal)
{
    if (finaltime >= (real_T)0 || finaltime == RUN_FOREVER) {
        *rtmTFinal = finaltime;
    }
}

void rtSetTFinalForExtMode(real_T *rtmTFinal)
{
    rtERTExtModeSetTFinal(rtmTFinal);
}

void rtParseArgsForExtMode(int_T        argc, 
                           const char_T *argv[])
{

#ifdef  EXTMODE_DISABLE_ARGS_PROCESSING
    /* Some targets do not support command line args, so bypass the args 
     * processing and directly call into ExtParseArgsAndInitUD.
     */

    /* initialize external mode */
    ExtParseArgsAndInitUD(0, NULL);

#else    
    /* parse command line args */

    int_T  oldStyle_argc;
    const char_T *oldStyle_argv[5];

    if ((argc > 1) && (argv[1][0] != '-')) {
        /* old style */
        if ( argc > 3 ) {
            displayUsage();
            exit(EXIT_FAILURE);
        }

        oldStyle_argc    = 1;
        oldStyle_argv[0] = argv[0];
    
        if (argc >= 2) {
            oldStyle_argc = 3;

            oldStyle_argv[1] = "-tf";
            oldStyle_argv[2] = argv[1];
        }

        if (argc == 3) {
            oldStyle_argc = 5;

            oldStyle_argv[3] = "-port";
            oldStyle_argv[4] = argv[2];

        }

        argc = oldStyle_argc;
        argv = oldStyle_argv;

    }

    {
        /* new style: */
        double    tmpDouble;
        char_T tmpStr2[200];
        int_T  count      = 1;
        int_T  parseError = false;

        /*
         * Parse the standard RTW parameters.  Let all unrecognized parameters
         * pass through to external mode for parsing.  NULL out all args handled
         * so that the external mode parsing can ignore them.
         */
        while(count < argc) {
            const char_T *option = argv[count++];
            
            /* final time */
            if ((strcmp(option, "-tf") == 0) && (count != argc)) {
                const char_T *tfStr = argv[count++];
                
                sscanf(tfStr, "%200s", tmpStr2);
                if (strcmp(tmpStr2, "inf") == 0) {
                    tmpDouble = RUN_FOREVER;
                } else {
                    char_T tmpstr[2];

#if INTEGER_CODE == 0
                    if ( (sscanf(tmpStr2,"%lf%1s", &tmpDouble, tmpstr) != 1) ||
                         (tmpDouble < (real_T)0) ) {
    #ifndef EXTMODE_DISABLEPRINTF
                        (void)printf("finaltime must be a positive, real value or inf\n");
    #endif
                        parseError = true;
                        break;
                    }
#else
                    if ( (sscanf(tmpStr2,"%d%1s", &tmpDouble, tmpstr) != 1) ||
                         (tmpDouble < (real_T)0) ) {
    #ifndef EXTMODE_DISABLEPRINTF
                        (void)printf("tmpDouble = %d\n", tmpDouble);
                        (void)printf("finaltime must be a positive, integer value or inf\n");
    #endif
                        parseError = true;
                        break;
                    }
#endif
                }
                finaltime = (real_T) tmpDouble;

                argv[count-2] = NULL;
                argv[count-1] = NULL;
            }
        }

        if (parseError) {
    #ifndef EXTMODE_DISABLEPRINTF        
            (void)printf("\nUsage: model_name -option1 val1 -option2 val2 -option3 "
                         "...\n\n");
            (void)printf("\t-tf 20 - sets final time to 20 seconds\n");
    #endif        

            exit(EXIT_FAILURE);
        }

        /*
         * Parse the external mode arguments.
         */
        {
            const char_T *extParseErrorPkt = ExtParseArgsAndInitUD(argc, argv);
            if (extParseErrorPkt != NULL) {
    #ifndef EXTMODE_DISABLEPRINTF           
                printf(
                    "\nError processing External Mode command line arguments:\n");
                printf("\t%s",extParseErrorPkt);
    #endif
                exit(EXIT_FAILURE);
            }
        }

        /*
         * Check for unprocessed ("unhandled") args.
         */
        {
            int i;
            for (i=1; i<argc; i++) {
                if (argv[i] != NULL) {
    #ifndef EXTMODE_DISABLEPRINTF                
                    printf("Unexpected command line argument: %s\n",argv[i]);
    #endif
                    exit(EXIT_FAILURE);
                }
            }
        }
    }

    if (finaltime == RUN_FOREVER) {
        #ifndef EXTMODE_DISABLEPRINTF
                printf ("\n**warning: the simulation will run with no stop time due "
                "to external mode with '-tf inf' argument.\n");
        #endif
    }

#endif /* EXTMODE_DISABLE_ARGS_PROCESSING */
}

void rtERTExtModeStartMsg(void)
{
#ifndef EXTMODE_DISABLEPRINTF    
    (void)printf("\n** starting the model **\n");
#endif
}

/* End of ERT specific functions and data */

/* [EOF] ext_work.c */


