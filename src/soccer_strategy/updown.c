/*
 * Copyright 1994-2014 The MathWorks, Inc.
 */
#include <stdlib.h>
#include <string.h>

/*Real Time Workshop headers*/
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"

#include "ext_types.h"
#include "ext_share.h"
#include "ext_svr.h"
#include "ext_work.h"
#include "updown_util.h"
#include "dt_info.h"

#if defined(VERBOSE) || DUMP_PKT
#include <stdio.h>
#endif

/* 
 * Depending on the target's native word size and pointer size, interrupts
 * might need to be disabled around critical regions when accessing the 
 * the external mode circular buffer. If, for example, reading a pointer 
 * is not guaranteed to be an atomic operation on your target, then you  
 * need to define the preprocessor defines below to prevent interrupts  
 * around the critical regions.
 * 
 * Here is a description of each of these preprocessor defines:
 * EXTMODE_PROTECT_CRITICAL_REGIONS: this should be defined if interrupts 
 *      should be disabled around critical regions
 * EXTMODE_INTERRUPT_INC_HDR: this defines the include header file 
 *      containing the declarations for the enabled/disable interrupts 
 *      functions
 * EXTMODE_ENABLE_INTERRUPTS:  this defines the enable interrupts function
 * EXTMODE_DISABLE_INTERRUPTS: this defines the disable interrupts function
 *
 * If you define EXTMODE_PROTECT_CRITICAL_REGIONS, you need to define the 
 * other above preprocessor defines, otherwise they are not required.
 */
#ifdef EXTMODE_PROTECT_CRITICAL_REGIONS
    #if !defined (EXTMODE_INTERRUPT_INC_HDR) || \
        !defined (EXTMODE_ENABLE_INTERRUPTS) || \
        !defined (EXTMODE_DISABLE_INTERRUPTS)
        #error EXTMODE_INTERRUPT_INC_HDR, EXTMODE_ENABLE_INTERRUPTS and \
               EXTMODE_DISABLE_INTERRUPTS should be defined
    #endif
    /* 
     * include header file containing declarations for interrupt 
     * enable/disable functions 
     */    
    #include EXTMODE_INTERRUPT_INC_HDR
#endif

/**********************
 * External Variables *
 **********************/
#ifdef VXWORKS
# include <sockLib.h>
# include <inetLib.h>
# include <selectLib.h>
extern SEM_ID uploadSem;

#endif

/* Logical definitions */
#if (!defined(__cplusplus))
#  ifndef false
#   define false                       (0U)
#  endif
#  ifndef true
#   define true                        (1U)
#  endif
#endif

boolean_T host_upstatus_is_uploading = false;

/******************************************************************************
 * Parameter Download                                                         *
 ******************************************************************************/

#ifdef VERBOSE
/* Function: DType2Real_T ======================================================
 * Convert a built-in data type to a real_T value.  Return the real_T value
 * as well as a string (by reference) corresponding to the original data type.
 * If the data type is not recognized as a Simulink built-in data type, then
 * NULL is returned for the dTypeName and 0 is returned for the dTypeValue.
 */
PRIVATE double DType2Double(
    const char              *vPtr,
    const int               dTypeIdx,
    const DataTypeTransInfo *dtInfo,
    const char              **dTypeName)
{
    real_T     outVal;
    char * const *dTypeNames = (char * const *)dtGetDataTypeNames(dtInfo);
    const char *thisName     = dTypeNames[dTypeIdx];

    *dTypeName = thisName;

    if (strcmp(thisName, "real_T") == 0) {
        outVal = (real_T) (((const real_T*)vPtr)[0]);
    } else if (strcmp(thisName, "real32_T") == 0) {
        outVal = (real_T) (((const real32_T*)vPtr)[0]);
    } else if (strcmp(thisName, "int8_T") == 0) {
        outVal = (real_T) (((const int8_T*)vPtr)[0]);
    } else if (strcmp(thisName, "uint8_T") == 0) {
        outVal = (real_T) (((const uint8_T*)vPtr)[0]);
    } else if (strcmp(thisName, "int16_T") == 0) {
        outVal = (real_T) (((const int16_T*)vPtr)[0]);
    } else if (strcmp(thisName, "uint16_T") == 0) {
        outVal = (real_T) (((const uint16_T*)vPtr)[0]);
    } else if (strcmp(thisName, "int32_T") == 0) {
        outVal = (real_T) (((const int32_T*)vPtr)[0]);
    } else if (strcmp(thisName, "uint32_T") == 0) {
        outVal = (real_T) (((const uint32_T*)vPtr)[0]);
    } else if (strcmp(thisName, "boolean_T") == 0) {
        outVal = (real_T) (((const boolean_T*)vPtr)[0]);
    } else {
        outVal    = 0;
        dTypeName = NULL;
    }
    return(outVal);
} /* end DType2Double */
#endif


/* Function: SetParam ==========================================================
 * Install new parameters.
 *
 * NOTE: pbuf looks like:
 *  [NPARAMS
 *   B S W DI DATA   % pVal 0
 *   B S W DI DATA   % pVal 1
 *   B S W DI DATA   % pVal N
 *  ]
 *
 *  No assumptions about the alignment of pbuf may be made.
 *
 *  where,
 *      B   : Data type transition index.  Note that on the target the data
 *            type transition index provides both the base address (B)
 *              of the transition as well as whether or not
 *              that section of the array contains complex elements.
 *      S   : starting offset of the data from the DT_TRANS_IDX
 *      W   : number of elements for this param
 *      DI  : index into rtw data type table (inUse idx)
 *      DATA: the param values (in target format).
 *
 *  and,
 *      All values, excluding DATA, are int32_T.
 */
#ifndef EXTMODE_DISABLEPARAMETERTUNING
PUBLIC void SetParam(RTWExtModeInfo  *ei, const char *pbuf)
{
    int        i;
    int32_T    nParams;
    const char *bufPtr    = pbuf;
    const int  B          = 0; /* index into dtype tran table (base address)  */
    const int  SI         = 1; /* starting index - wrt to base address        */
    const int  W          = 2; /* width of section (number of elements)       */
    const int  DI         = 3; /* index into data type tables                 */
    const int  tmpBufSize = sizeof(int32_T) * 4;
    int32_T    tmpBuf[4];

    const DataTypeTransInfo *dtInfo = rteiGetModelMappingInfo(ei);
    const DataTypeTransitionTable *dtTable = dtGetParamDataTypeTrans(dtInfo);
    const uint_T *dtSizes = dtGetDataTypeSizes(dtInfo);

    /* unpack NPARAMS */
    (void)memcpy(&nParams, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);
    
#ifdef VERBOSE
    printf("\nUpdating %d parameters....\n", nParams);
#endif

    /*
     * Unpack the data and install the new parameters.
     */
    for (i=0; i<nParams; i++) {
        int_T   elSize;
        int_T   nBytes;
        char_T  *start;
        char_T  *tranAddress;
        int_T   tranIsComplex;

        /* unpack B SI W DI */
        (void)memcpy(tmpBuf, bufPtr, tmpBufSize);
        bufPtr += tmpBufSize;

        /* 
         * Find starting address and size (nBytes) for this parameters 
         * section of memory.
         */
        tranAddress   = dtTransGetAddress(dtTable, tmpBuf[B]);
        tranIsComplex = dtTransGetComplexFlag(dtTable, tmpBuf[B]);

        elSize = dtSizes[tmpBuf[DI]] * (tranIsComplex ? 2 : 1);
        nBytes = tmpBuf[W] * elSize;
        start = tranAddress + (tmpBuf[SI] * elSize);

        /* Install the params. */
        (void)memcpy(start, bufPtr, nBytes);
        bufPtr += nBytes;

#ifdef VERBOSE
        /*
         * It is safe to assume that once the params are installed into
         * the param vector that they are properly aligned.  So we
         * do our verbosity print-out here.
         */
        {
            double     val;
            const char *dTypeName;
            const char *vPtr = (const char *)start;

            val = DType2Double(start, tmpBuf[DI], dtInfo, &dTypeName);
            printf("\n\tParam| "
                   "DT_Trans: %d, index: %d, nEls: %d, data type: [%s, %d]\n",
                   tmpBuf[B], tmpBuf[SI], tmpBuf[W],
                   (dTypeName != NULL) ? dTypeName : "",tmpBuf[DI]);

            if (!tranIsComplex) {
                int j;
                for (j=0; j<tmpBuf[W]; j++) {
                    val = DType2Double(vPtr, tmpBuf[DI], dtInfo, &dTypeName);
                    printf("\t\t%f\n", val);
                    vPtr += dtSizes[tmpBuf[DI]];
                }
            } else {
                int j;
                for (j=0; j<tmpBuf[W]; j++) {
                    val = DType2Double(vPtr, tmpBuf[DI], dtInfo, &dTypeName);
                    printf("\t\t%f + ", val);
                    vPtr += dtSizes[tmpBuf[DI]];

                    val = DType2Double(vPtr, tmpBuf[DI], dtInfo, &dTypeName);
                    printf("\t\t%fi\n", val);
                    vPtr += dtSizes[tmpBuf[DI]];
                }
            }
        }
#endif
    }
} /* end SetParam */
#endif /* ifndef EXTMODE_DISABLEPARAMETERTUNING */


/******************************************************************************
 * Parameter Upload                                                           *
 ******************************************************************************/
#include "upsup_public.h"
#define DUMP_PKT (0)


/*=============================================================================
 * Circular buffer stuff.
 *============================================================================*/
typedef struct BufMemList_tag {

#if ASSERTS_ON
    int_T  maxBufs;     /* for debugging                       */
#endif
    int_T  nActiveBufs; /* num non-empty bufs                  */
    int_T  *tids;       /* tid associated with each active buf */
    BufMem *bufs;       /* sections of each buffer to upload   */
} BufMemList;


typedef struct CircularBuf_tag {
    volatile int_T    empty;

    int_T    bufSize;
    char_T   *buf;
    
    char_T* volatile head;
    char_T* volatile tail;

    char_T   *newTail;

    struct {
        int_T count;
    } preTrig;
} CircularBuf;


/*==============================================================================
 * Trigger stuff.
 *============================================================================*/
typedef enum {
    TRIGGER_UNARMED,
    TRIGGER_HOLDING_OFF,
    TRIGGER_ARMED,
    TRIGGER_DELAYED,
    TRIGGER_FIRED,
    TRIGGER_TERMINATING
} TriggerState;


/*==============================================================================
 * General stuff.
 *============================================================================*/

/*
 * An UploadSection defines a contiguous section of the blockio or dwork
 * structure.  Each section consists of elements of the same data type and same
 * complexity.
 * 'start' should be a const pointer
 */
typedef struct UploadSection_tag {
    void   *start;
    int_T  nBytes;
} UploadSection;

/*
 * An UploadMap is an array of UploadSections.  Typically a map consists of all
 * of the sections of the blockio or dwork structure that are relevant to a
 * given task.
 */
typedef struct UploadMap_tag {
    int32_T    nSections;
    UploadSection *sections;

    int_T nBytes;  /* total number of bytes in this map */
} UploadMap;


/*
 * Each system contains a table of UploadMap's (one for each tid).  If no data
 * uploading is being done for a given tid, the uploadMap pointer is NULL.
 * The enableState field indicates whether the system in question is active.
 *
 * For a model with 5 total tids, the table looks like:
 *
 *               tid 0    tid 1    tid 2   tid 3   tid 4
 * uploadMap   -------------------------------------------
 *             |  ptr  |  NULL  |  NULL  |  ptr  |  NULL |
 *             ----|-------------------------|------------
 *                 |                         |
 *                 v                         v
 *              UploadMap                 UploadMap
 *              for tid 0                 for tid 3
 */
typedef struct SysUploadTable_tag {
    int8_T    *enableState;
    UploadMap **uploadMap;
} SysUploadTable;


typedef struct TriggerInfo_tag {
    TriggerState            state;
    int_T                   tid;
    int32_T                 duration;
    int32_T                 holdOff;
    int32_T                 delay;
    int_T                   lookForRising;
    int_T                   lookForFalling;
    real_T                  level;
    int_T                   count;
    int_T                   overFlow;

    UploadMap               trigSignals;
    real_T                  *oldTrigSigVals;
    int_T                   haveOldTrigSigVal;

    struct {
        int32_T    duration;
        int32_T    count;

        int_T      checkUnderFlow; /* ??? */
    } preTrig;
} TriggerInfo;


/*
 * The BdUploadInfo contains all information regarding data logging.
 */
struct BdUploadInfo_tag {
    int32_T        upInfoIdx;  /* index of upInfo in the array                 */
    int32_T        nSys;       /* # of sys's for which data logging is active  */
    SysUploadTable *sysTables; /* array of SysUploadTables                     */
    CircularBuf    *circBufs;  /* circular buffers to store upload data        */
    BufMemList     bufMemList; /* list of buffer memory holding data to upload */

    TriggerInfo  trigInfo;
};


/*==============================================================================
 * Global upload data.
 */

/*
 * Definitions that must match Simulink definitions.
 */

#define UPLOAD_RISING_TRIGGER                   ((int32_T)  0)
#define UPLOAD_FALLING_TRIGGER                  ((int32_T)  1)
#define UPLOAD_EITHER_RISING_OR_FALLING_TRIGGER ((int32_T)  2)

/*
 * Definitions.
 */
#define TRIGMODE_ONESHOT (-1)

#define NUM_UPINFOS   2
static  BdUploadInfo  uploadInfoArray[NUM_UPINFOS];


/* Function ====================================================================
 * Dump the signal selection packet (EXT_SELECT_SIGNALS).  The packet looks
 * like:
 *
 * upInfoIdx - Index of the UploadInfo. 
 *
 * nSys - the number of systems that contain upload blocks (length of the
 *        (BdUploadInfo list)
 *
 * enableIdx - the index into the "mode vector" that tells whether or not
 *             a given system is active
 *
 * nTids - the number of tids in a system that contain upload blocks (number
 *         of non NULL entries in a SysUploadTable)
 *
 * nSections - the number of contiguous blockio or dwork sections that
 *             correspond to all blocks within a tid (number of sections in
 *             an UploadMap)
 *
 * B  - Index into data type transition table       - gives base address
 * S  - the starting index of a blockio section     - with respect to B
 * W  - the number of elements in a blockio section
 * DI - data type index - index into the rtw data type table
 *
 * target buf size - size of the upload buffer (to be allocated by target) for
 *                   a given tid
 *
 * Here's the packet format:
 *
 * [upInfoIdx
 *  
 *  nSys
 *
 *  enableIdx                           ---
 *  nTids                                 |
 *  tid nSections B S W DI B S W DI ...   | system
 *  tid nSections B S W DI B S W DI ...   |
 *                                      ---
 *  enableIdx                           ---
 *  nTids                                 |
 *  tid nSections B S W DI B S W DI ...   | system
 *  tid nSections B S W DI B S W DI ...   |
 *                                      ---
 *  target buf size for tid 0
 *  target buf size for tid 1
 *            .
 *            .
 *  target buf size for tid n
 * ]
 *
 * All elements are int32_T.
 */
#if DUMP_PKT
PRIVATE void DumpSelectSignalPkt(const char *pkt, int nRootTids)
{
    int32_T    i,j,k;
    int32_T    upInfoIdx;
    int32_T    nSys;
    const char *bufPtr = pkt;

    printf("Signal Select Pkt-----------\n");

    /* upInfoIdx */
    (void)memcpy(&upInfoIdx, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\nupInfoIdx: %d\n",upInfoIdx);

    /* nSys */
    (void)memcpy(&nSys, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\nnSys: %d\n",nSys);

    for (i=0; i<nSys; i++) {
        int32_T enableIdx, nTids;
        
        /* [enableIdx, nTids] */
        (void)memcpy(&enableIdx, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);

        (void)memcpy(&nTids, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);

        printf("[enableIdx, nTids]: %d %d\n", enableIdx, nTids);

        for (j=0; j<nTids; j++) {
            int32_T tid;
            int32_T nSections;

            /* [tid nSections] */
            (void)memcpy(&tid, bufPtr, sizeof(int32_T));
            bufPtr += sizeof(int32_T);

            (void)memcpy(&nSections, bufPtr, sizeof(int32_T));
            bufPtr += sizeof(int32_T);

            printf("[tid nSects]: %d %d\n", tid, nSections);

            for (k=0; k<nSections; k++) {
                const int B  = 0;
                const int S  = 1;
                const int W  = 2;
                const int DI = 3;
                int32_T   tmpBuf[4];

                /* [B S W DI] */
                (void)memcpy(&tmpBuf, bufPtr, sizeof(int32_T)*4);
                bufPtr += (sizeof(int32_T) * 4);
                
                printf("%d %d %d %d\n",
                    tmpBuf[B], tmpBuf[S], tmpBuf[W], tmpBuf[DI]);
            }
            printf("\n");
        }
        printf("\n");
    }

    /*
     * Now the buffer sizes.
     */
    printf("bufSizes: ");
    for (i=0; i<nRootTids; i++) {
        int32_T bufSize;

        printf("\nbufSize[%d] of %d: ",i, nRootTids);

        (void)memcpy(&bufSize, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);

        printf("%d", bufSize);
    }
    printf("\nEnd of select sigs pkt----\n");
} /* end DumpSelectSignalPkt */
#else
#define DumpSelectSignalPkt(buf, nRootTids) /* do nothing */
#endif


/* Function ====================================================================
 * Dump the trigger selection packet (EXT_SELECT_TRIGGER).  The packet looks
 * like:
 *
 * upInfoIdx - Index of the UploadInfo. 
 *
 * tid       - tid of the trigger signal
 *
 * duration  - The number of base rate steps for which the data logging event
 *             occurs.
 *
 * holdOff   - (-1), signifies that this trigger event is a one_shot, else we 
 *             are in normal mode.  For normal mode the value indicates
 *             the number of base rate steps to wait between the end a data
 *             logging event and the re-arming of the trigger for the next data
 *             logging event.  The end of a data logging event is defined as
 *             when the last bit of data has been sent to the host (i.e.,
 *             immediately after the termination flag has been sent).
 *
 * delay     - The number of base rate steps to wait after the trigger event
 *             occurs and the before starting the data collection.  This can
 *             be either positive or negative (pre-trigger).  This field is
 *             ignored if the trigger source is manual.
 *
 * nsections - The sections of the blockio array to monitor for a trigger event.
 *             If the trigger event is not based on a signal, this is set to
 *             0 (e.g., the signal source is manual).
 *
 * sections  - "B, S, W, DI" description of a signal (see Signal Selection Pkt).
 *             These are the elements of the blockio vector that are monitored
 *             for trigger events when the trigger is based on a signal.  It is
 *             ignored if the trigger event is not based on a signal
 *             (nsections == 0).
 *
 * direction - If the triggering source is a signal, then this specifies the
 *             direction of the crossing (rising, falling or either).  If we
 *             are not triggering on a signal (nsections == 0), then this field
 *             is ignored.
 *
 * level     - If the triggering source is a signal, then this field specifies
 *             the level (value) of the crossing (0 by default).  If we are not
 *             triggering on a signal (nsections == 0), then this field is
 *             ignored.
 *
 * The packet looks like:
 * [tid duration holdOff delay nsections B S W DI B S W DI ... direction level]
 *
 * All fields are int32_T except for level, which is an SL_DOUBLE (real_T
 * on target).
 */
#if DUMP_PKT
PRIVATE void DumpSelectTriggerPkt(const char *pkt)
{
    int32_T    i;
    int32_T    upInfoIdx, tid, duration, holdOff, delay, nSections;
    int32_T    direction;
    real_T     level;
    const char *bufPtr = pkt;

    printf("Trigger Select Pkt-----------\n");

    /* upInfoIdx */
    (void)memcpy(&upInfoIdx, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\nupInfoIdx: %d\n",upInfoIdx);
    
    /* tid */
    (void)memcpy(&tid, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\ntid: %d\n",tid);
    
    /* duration */
    (void)memcpy(&duration, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\nduration: %d\n",duration);

    /* holdOff */
    (void)memcpy(&holdOff, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\nholdOff: %d\n",holdOff);

    /* delay */
    (void)memcpy(&delay, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("\ndelay: %d\n",delay);

    /* nsections */
    (void)memcpy(&nSections, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("nSects: %d\n", nSections);

    /* each section */
    for (i=0; i<nSections; i++) {
        const int B  = 0;
        const int S  = 1;
        const int W  = 2;
        const int DI = 3;
        int32_T   tmpBuf[4];

        (void)memcpy(&tmpBuf, bufPtr, sizeof(int32_T)*4);
        bufPtr += (sizeof(int32_T) * 4);
        
        printf("%d %d %d %d\n", tmpBuf[B], tmpBuf[S], tmpBuf[W], tmpBuf[DI]);
    }
    printf("\n");

    /* direction */
    (void)memcpy(&direction, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    printf("direction: %d\n",direction);

    /* level */
    (void)memcpy(&level, bufPtr, sizeof(real_T));
    bufPtr += sizeof(real_T);

    printf("level: %f\n",level);
} /* end DumpSelectTriggerPkt */
#else
#define DumpSelectTriggerPkt(buf) /* do nothing */
#endif


/* Function ====================================================================
 * Initialize a UploadSection.
 */
#ifndef EXTMODE_DISABLESIGNALMONITORING
PRIVATE void InitUploadSection(
    RTWExtModeInfo *ei,
    const int32_T  *buf,
    UploadSection  *section)   /* out */
{
    int_T                         elSize;
    int_T                         offset;
    int_T                         nBytes;
    char_T                        *tranAddress;
    int_T                         tranIsComplex;
    
    const DataTypeTransInfo       *dtInfo  = rteiGetModelMappingInfo(ei);
    const DataTypeTransitionTable *dtTable = dtGetBIODataTypeTrans(dtInfo);
    const uint_T *dtSizes = dtGetDataTypeSizes(dtInfo);

    const int BI = 0;  /* index into dtype tran table (base address)  */
    const int SI = 1;  /* starting index - wrt to base address        */
    const int W  = 2;  /* width of section (number of elements)       */
    const int DI = 3;  /* index into data type tables                 */

    tranAddress   = dtTransGetAddress(dtTable, buf[BI]);
    tranIsComplex = dtTransGetComplexFlag(dtTable, buf[BI]);

    elSize = dtSizes[buf[DI]] * (tranIsComplex ? 2 : 1);
    nBytes = buf[W] * elSize;
    offset = buf[SI] * elSize;

    section->start  = tranAddress + offset;
    section->nBytes = nBytes;
} /* end InitUploadSection */


/* Function ====================================================================
 * Initialize a SysUploadTable.  The callerBufPtr points to the current place in
 * the EXT_SELECT_SIGNALS pkt which should be the enableIdx field.  This
 * function moves the callerBufPtr to the next unread field of the packet.
 */
PRIVATE boolean_T InitSysUploadTable(
    RTWExtModeInfo *ei,
    int_T          numSampTimes,
    SysUploadTable *sysTable,
    const char     **callerBufPtr) /* in/out */
{
    int_T        i;
    int32_T      nTids;
    const char_T *bufPtr = *callerBufPtr;
    boolean_T    error   = EXT_NO_ERROR;

    /*
     * Set pointer to enable mode.
     */
    {
        int32_T sysIdx;

        /* read sysIdx */
        (void)memcpy(&sysIdx, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);

        sysTable->enableState = rteiGetAddrOfSubSystemActiveVector(ei,sysIdx);
    }

    /*
     * Allocate/initialize each tid's uploadMap.
     */

    /* ...read [nTids] */
    (void)memcpy(&nTids, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    /* Allocate the array of pointers to UploadMaps. */
    sysTable->uploadMap = (UploadMap **)calloc(numSampTimes, sizeof(UploadMap *));
    if (sysTable->uploadMap == NULL) {
        error = EXT_ERROR; goto EXIT_POINT;       
    }

    for (i=0; i<nTids; i++) {
        int32_T   tid;
        int32_T   section;
        UploadMap *map;
        
        /* read tid */
        (void)memcpy(&tid, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);
        
        /* allocate UploadMap */
        assert(sysTable->uploadMap[tid] == NULL);
        sysTable->uploadMap[tid] = (UploadMap *)calloc(1, sizeof(UploadMap));
        if (sysTable->uploadMap[tid] == NULL) {
            error = EXT_ERROR; goto EXIT_POINT;       
        }
        map = sysTable->uploadMap[tid];
        
        /* read nSections */
        (void)memcpy(&map->nSections, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);

        /* Allocate the blockio sections. */
        assert(map->sections == NULL);
        map->sections = (UploadSection *)calloc(map->nSections,sizeof(UploadSection));
        if (map->sections == NULL) {
            error = EXT_ERROR; goto EXIT_POINT;
        }

        /*
         * Init the UploadSections.
         */
        for (section=0; section<map->nSections; section++) {
            int32_T    tmpBuf[4];
            UploadSection *uploadSection = &map->sections[section];

            /* read [B S W DI] */
            (void)memcpy(&tmpBuf, bufPtr, sizeof(int32_T)*4);
            bufPtr += (sizeof(int32_T) * 4);

            InitUploadSection(ei, tmpBuf, uploadSection);

            /* keep track of total number of bytes in this map */
            map->nBytes += uploadSection->nBytes;
        }
    }
    
EXIT_POINT:
    *callerBufPtr = bufPtr;
    return(error);
} /* end InitSysUploadTable */

/* Function ====================================================================
 * Initialize circular buffer fields and allocate required memory.
 */
PRIVATE boolean_T UploadBufInit(CircularBuf *circBuf, int_T size)
{
    boolean_T error = NO_ERR;

    assert(circBuf->buf == NULL);

    /*
     * Size will be negative to indicate an error if host determines too much
     * memory is needed (i.e. the size will not fit in an int32).
     */
    if (size < 0) {
        error = EXT_ERROR; goto EXIT_POINT;
    }

    circBuf->empty = true;
    if (size > 0) {
        assert(circBuf->buf == NULL);
        circBuf->buf = (char_T *)malloc(size);
        if (circBuf->buf == NULL) {
            error = EXT_ERROR; goto EXIT_POINT;
        }
    } else {
        circBuf->buf = NULL;
    }
    circBuf->bufSize = size;
    
    circBuf->head = circBuf->buf;
    circBuf->tail = circBuf->buf;

    circBuf->newTail = NULL;

EXIT_POINT:
    return(error);
} /* end UploadBufInit */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */


/* Function ====================================================================
 * Free all dynamically allocated fields of the trigInfo structure.
 */
PRIVATE void UploadDestroyTrigger(int32_T upInfoIdx)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    TriggerInfo *trigInfo    = &uploadInfo->trigInfo;
    UploadMap   *map         = &trigInfo->trigSignals;

    if (map->sections != NULL) {
        free(map->sections);
        map->sections = NULL;
    }

    if (trigInfo->oldTrigSigVals != NULL) {
        free(trigInfo->oldTrigSigVals);
        trigInfo->oldTrigSigVals = NULL;
    }

    /*
     * Reset trigger info.
     */
    trigInfo->state          = TRIGGER_UNARMED;
    trigInfo->duration       = 0;
    trigInfo->holdOff        = 0;
    trigInfo->delay          = 0;
    trigInfo->lookForRising  = true;
    trigInfo->lookForFalling = false;
    trigInfo->level          = (real_T)0;
    trigInfo->count          = 0;
    trigInfo->overFlow       = false;

    trigInfo->trigSignals.nSections = 0;
    trigInfo->trigSignals.sections  = NULL;
    trigInfo->trigSignals.nBytes    = 0;

    trigInfo->oldTrigSigVals    = NULL;
    trigInfo->haveOldTrigSigVal = false;

    trigInfo->preTrig.duration       = 0;
    trigInfo->preTrig.count          = 0;
    trigInfo->preTrig.checkUnderFlow = false;
} /* end UploadDestroyTrigger */


/* Function ====================================================================
 * Reset fields of the uploadinfo struct.
 */
PUBLIC void UploadLogInfoReset(int32_T upInfoIdx)
{
    static boolean_T firstTime[NUM_UPINFOS] = {true, true};
    BdUploadInfo     *uploadInfo = &uploadInfoArray[upInfoIdx];
    
    /*
     * uploadInfoArray is declared as global static, so most compilers will
     * initialize the memory to zero.  However, the tic6000 assigns this
     * variable into uninitialized memory.  When this function is called for
     * the first time, we attempt to free garbage pointers and crash.  By
     * clearing unloadInfoArray explicitly the first time this function is
     * called, we can ensure that this function will work for all compilers.
     */
    if (firstTime[upInfoIdx]) {
        memset(uploadInfo, 0, sizeof(BdUploadInfo));
        firstTime[upInfoIdx] = false;
    }
   
    /* sysUploadTable */
    uploadInfo->nSys      = 0;
    uploadInfo->sysTables = NULL;

    uploadInfo->circBufs = NULL;

    uploadInfo->bufMemList.bufs = NULL;
    uploadInfo->bufMemList.tids = NULL;

    /* Reset trigger info */
    UploadDestroyTrigger(upInfoIdx);

} /* end UploadLogInfoReset */


/* Function ====================================================================
 * Destroy all data associated with data logging.  Fields are re-initialized
 * and pointers NULL'ed out by UploadLogInfoReset().
 */
PUBLIC void UploadLogInfoTerm(int32_T upInfoIdx, int_T numSampTimes)
{
    int_T i;
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];

    if (uploadInfo->nSys == 0) return; /* Nothing to terminate */

    /*
     * Free fields of the sysUpload tables and then the table itself.
     */
    for (i=0; i<uploadInfo->nSys; i++) {
        int_T     tid;
        UploadMap **uploadMap = uploadInfo->sysTables[i].uploadMap;
        
        for (tid=0; tid<numSampTimes; tid++) {
            if (uploadMap[tid] != NULL) {
                /* Free fields of uploadMap. */
                free(uploadMap[tid]->sections);

                /* Free the uploadMap. */
                free(uploadMap[tid]);
            }
        }

        free(uploadMap);
    }

    free(uploadInfo->sysTables);

    /* Free circular buf fields and bufMemLists. */
    if (uploadInfo->circBufs) {
        for (i=0; i<numSampTimes; i++) {
            free(uploadInfo->circBufs[i].buf);
        }
        free(uploadInfo->circBufs);
    }

    free(uploadInfo->bufMemList.bufs);
    free(uploadInfo->bufMemList.tids);
    
    /*
     * Free trigger info.
     */
    UploadDestroyTrigger(upInfoIdx);

    /*
     * Reset all vals to initial value and NULL out pointers.
     */
    UploadLogInfoReset(upInfoIdx);
} /* end UploadLogInfoTerm */


/* Function ====================================================================
 * Prepare for final flush of buffers.  This involves setting the trigger
 * state to appropriate values.
 */
PUBLIC void UploadPrepareForFinalFlush(int32_T upInfoIdx)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];

    switch(uploadInfo->trigInfo.state) {
    case TRIGGER_FIRED:
    case TRIGGER_TERMINATING:
        /*
         * 1) set trig state to "terminating" so that the eventual call to
         *    UploadBufGetData knows to add the terminator flag to
         *    the data stream.
         * 2) set trig state to "oneshot" to prevent re-arming
         */
        uploadInfo->trigInfo.state   = TRIGGER_TERMINATING; /* 1 */
        uploadInfo->trigInfo.holdOff = TRIGMODE_ONESHOT;    /* 2 */
        break;

    case TRIGGER_UNARMED:
    case TRIGGER_HOLDING_OFF:
    case TRIGGER_ARMED:
    case TRIGGER_DELAYED:
        /* do nothing */
        break;
    }

#ifdef VXWORKS
    /* Let upload server run to ensure that term pkt is sent to host. One
       semGive() is for the background task and the other is for the explicit
       call to rt_UploadServerWork() in DisconnectFromHost(). */
    semGive(uploadSem);
    semGive(uploadSem);
#endif
	
} /* end UploadPrepareForFinalFlush */


/* Function ====================================================================
 * Initialize data uploading by processing the EXT_SELECT_SIGNALS packet
 * (which is passed in).  Return the error status.  See DumpSelectSignalPkt()
 * for a detailed description of the packet.
 */
#ifndef EXTMODE_DISABLESIGNALMONITORING
PUBLIC boolean_T UploadLogInfoInit(RTWExtModeInfo *ei,
                                   int_T          numSampTimes,
                                   const char     *pkt,
                                   int32_T        upInfoIdx)
{
    int          nActiveTids;
    int_T        i;
    boolean_T    error   = EXT_NO_ERROR;
    const char   *bufPtr = pkt;
    BdUploadInfo *uploadInfo;

    DumpSelectSignalPkt(pkt, numSampTimes);

    /* Point to the correct uploadInfo */
    uploadInfo           = &uploadInfoArray[upInfoIdx];
    uploadInfo->upInfoIdx = upInfoIdx;

    /* Free upInfo if fields are already allocated */
    UploadLogInfoTerm(upInfoIdx, numSampTimes);

    /* nSys */
    (void)memcpy(&uploadInfo->nSys, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);
    assert(uploadInfo->sysTables == NULL);
    uploadInfo->sysTables = (SysUploadTable *)
        calloc(uploadInfo->nSys, sizeof(SysUploadTable));
    if (uploadInfo->sysTables == NULL) {
        error = EXT_ERROR; goto EXIT_POINT;
    }

    /*
     * Init each system table.
     */
    for (i=0; i<uploadInfo->nSys; i++) {
        error = InitSysUploadTable(ei, numSampTimes,
                                   &uploadInfo->sysTables[i], &bufPtr);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;
    }

    assert(uploadInfo->circBufs == NULL);
    uploadInfo->circBufs = (CircularBuf *) calloc(numSampTimes,
                                                  sizeof(CircularBuf));

    /*
     * Allocate the circular buffers.
     */
    nActiveTids = 0;
    for (i=0; i<numSampTimes; i++) {
        int32_T size;
        
        (void)memcpy(&size, bufPtr, sizeof(int32_T));
        bufPtr += sizeof(int32_T);
        
        error = UploadBufInit(&uploadInfo->circBufs[i], size);
        if (error != EXT_NO_ERROR) goto EXIT_POINT;

        nActiveTids += (size != 0);
    }

    /*
     * Initialize/Allocate the bufMemLists - these are used by
     * ext_svr to pull the appropriate data out of the buffers and send it
     * to the host.
     */
#if ASSERTS_ON
    uploadInfo->bufMemList.maxBufs = nActiveTids;
#endif
    uploadInfo->bufMemList.nActiveBufs = 0;

    assert(uploadInfo->bufMemList.bufs == NULL);
    uploadInfo->bufMemList.bufs = (BufMem *)malloc(nActiveTids*sizeof(BufMem));
    if (uploadInfo->bufMemList.bufs == NULL) {
        error = EXT_ERROR; goto EXIT_POINT;
    }

    assert(uploadInfo->bufMemList.tids == NULL);
    uploadInfo->bufMemList.tids = (int_T *)malloc(nActiveTids*sizeof(int_T));
    if (uploadInfo->bufMemList.tids == NULL) {
        error = EXT_ERROR; goto EXIT_POINT;
    }

EXIT_POINT:
    if (error != EXT_NO_ERROR) {
        UploadLogInfoTerm(upInfoIdx, numSampTimes);
    }
    return(error);
} /* end UploadLogInfoInit */


/* Function ====================================================================
 * Initialize and configure the trigger attributes.  See DumpSelectTriggerPkt()
 * for a detailed description of the packet.
 */
PUBLIC boolean_T UploadInitTrigger(RTWExtModeInfo *ei,
                                   const char     *pkt,
                                   int32_T         upInfoIdx)
{
    int_T       nSections;
    int32_T     i32_tid;
    int32_T     direction;
    TriggerInfo *trigInfo;
    boolean_T   error     = EXT_NO_ERROR;
    const char  *bufPtr   = pkt;
    
    DumpSelectTriggerPkt(pkt);
    
    /* Select the trig Info */
    trigInfo = &uploadInfoArray[upInfoIdx].trigInfo;

    /* tid, duration, holdOff, delay and nsections */
    (void)memcpy(&i32_tid, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);
    trigInfo->tid = (int_T)i32_tid;

    (void)memcpy(&trigInfo->duration, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    (void)memcpy(&trigInfo->holdOff, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    (void)memcpy(&trigInfo->delay, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    (void)memcpy(&trigInfo->trigSignals.nSections, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);
    
    nSections = trigInfo->trigSignals.nSections;

    /*
     * Init the UploadSections - if the trigger is signal based.
     */
    if (nSections > 0) {
        /* trigger is signal based */
        int       section;
        UploadMap *map = &trigInfo->trigSignals;

        assert(map->nBytes == 0);
        assert(map->sections == NULL);
        map->sections = (UploadSection *)malloc(nSections * sizeof(UploadSection));
        if (map->sections == NULL) {
            error = EXT_ERROR; goto EXIT_POINT;
        }
            
        for (section=0; section<map->nSections; section++) {
            int32_T       tmpBuf[4];
            UploadSection *uploadSection = &map->sections[section];

            /* read [B S W DI] */
            (void)memcpy(&tmpBuf, bufPtr, sizeof(int32_T)*4);
            bufPtr += (sizeof(int32_T) * 4);
            
            InitUploadSection(ei, tmpBuf, uploadSection);

            /* keep track of total number of bytes in this map */
            map->nBytes += uploadSection->nBytes;
        }

        /*
         * Allocate space to hold the old values of the trigger signals.  Note
         * that trigger signals are guaranteed to be of type SL_DOUBLE (real_T)
         * and non-complex.
         */
        assert(trigInfo->oldTrigSigVals == NULL);
        assert(trigInfo->oldTrigSigVals == NULL);
        trigInfo->oldTrigSigVals = (real_T *)malloc(map->nBytes);
        if (trigInfo->oldTrigSigVals == NULL) {
            error = EXT_ERROR; goto EXIT_POINT;
        }
    }
            
    /* Direction. */
    (void)memcpy(&direction, bufPtr, sizeof(int32_T));
    bufPtr += sizeof(int32_T);

    trigInfo->lookForRising  = 
        ((direction == UPLOAD_RISING_TRIGGER)  || 
         (direction == UPLOAD_EITHER_RISING_OR_FALLING_TRIGGER));

    trigInfo->lookForFalling = 
        ((direction == UPLOAD_FALLING_TRIGGER) || 
         (direction == UPLOAD_EITHER_RISING_OR_FALLING_TRIGGER));
    
    /* level */
    (void)memcpy(&trigInfo->level, bufPtr, sizeof(real_T));
    bufPtr += sizeof(real_T);

    /*
     * Convert delay to pre-trigger duration.
     */
    if (trigInfo->delay < 0) {
        trigInfo->preTrig.duration = -trigInfo->delay;
        trigInfo->delay            = 0;
    } else {
        trigInfo->preTrig.duration = 0;
    }

EXIT_POINT:
    if (error != EXT_NO_ERROR) {
        UploadDestroyTrigger(upInfoIdx);
    }
    return(error);
} /* end UploadInitTrigger */


/* Function ====================================================================
 * Arm the trigger.
 */
PUBLIC void UploadArmTrigger(int32_T upInfoIdx, int_T numSampTimes)
{
    int_T   tid;
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];

    assert((uploadInfo->trigInfo.state == TRIGGER_UNARMED) ||
           (uploadInfo->trigInfo.state == TRIGGER_HOLDING_OFF));

    host_upstatus_is_uploading = false;

    /*
     * Re-initialize.
     */
    uploadInfo->trigInfo.overFlow = false;
    for (tid=0; tid<numSampTimes; tid++) {
        CircularBuf *circBuf = &uploadInfo->circBufs[tid];
        if (circBuf->bufSize > 0) {
            circBuf->head = circBuf->buf;
            circBuf->tail = circBuf->buf;

            circBuf->newTail = NULL;
            circBuf->empty   = true;
        }
    }

    /* 
     * Re-initialize trigger fields.
     */
    uploadInfo->trigInfo.count             = 0;
    uploadInfo->trigInfo.haveOldTrigSigVal = false;

    /* 
     * Reset pre-trig counts for normal mode.
     */
    uploadInfo->trigInfo.preTrig.count = 0;

    /* 
     * Re-arm after all initialization.  Make sure that trigInfo.state is
     * set last since this routine may be interrupted.
     */
    uploadInfo->trigInfo.state = TRIGGER_ARMED;

} /* end UploadArmTrigger */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */

/* Function ====================================================================
 * Terminate this data logging session by destroying the uploadInfo and
 * setting the trigger backed to the unarmed state.
 */
PUBLIC void UploadEndLoggingSession(int32_T upInfoIdx, int_T numSampTimes)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];

    uploadInfo->trigInfo.state = TRIGGER_UNARMED;
    UploadLogInfoTerm(upInfoIdx, numSampTimes);
} /* end UploadEndLoggingSession */


/* Function ====================================================================
 * Cancel this data logging session.
 */
PUBLIC void UploadCancelLogging(int32_T upInfoIdx)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];

    switch(uploadInfo->trigInfo.state) {

    case TRIGGER_UNARMED:
        break;

    case TRIGGER_HOLDING_OFF:
    case TRIGGER_ARMED:
    case TRIGGER_DELAYED:
    case TRIGGER_FIRED:
        /*
         * Move to TRIGGER_TERMINATING and ensure that we are no longer in
         * "normal" mode (TRIGMODE_NORMAL) so that the trigger does not get
         * re-armed.
         */
        uploadInfo->trigInfo.holdOff = TRIGMODE_ONESHOT;
        uploadInfo->trigInfo.state   = TRIGGER_TERMINATING;
#ifdef VXWORKS
        /*
         * Let upload server run to ensure that term pkt is sent to host (needed
         * for all but the TRIGGERED_FIRED case since the upload server is
         * inactive).
         */
        semGive(uploadSem);
#endif
        break;
    
    case TRIGGER_TERMINATING:
        /*
         * Ensure that we are no longer in "normal" mode (TRIGMODE_NORMAL) so
         * that the trigger does not get re-armed.
         */
        uploadInfo->trigInfo.holdOff = TRIGMODE_ONESHOT;
        break;
    }
} /* end UploadCancelLogEvent */


/* Function ====================================================================
 * Called by ext_svr (background task), in order to perform tasks that need
 * to be done after each time that data has been sent to the host.  This
 * includes:
 *
 * o move the tail for the specified buffer forward
 * o detect the end of a data logging event so that the trigger state can
 *   be either set to unarmed (for one shot) or backed to armed (for normal
 *   mode).
 * 
 * NOTE:  UploadBufGetData and UploadBufMoveTail must be called in pairs where the
 *        UploadBufGetData call precedes the UploadBufMoveTail call.
 */
#ifndef EXTMODE_DISABLESIGNALMONITORING
PUBLIC void UploadBufDataSent(const int_T tid, int32_T upInfoIdx)
{   
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    CircularBuf  *circBuf    = &uploadInfo->circBufs[tid];            

    host_upstatus_is_uploading = true;
    
    /* Move the tail forward. */
    circBuf->tail = circBuf->newTail;
        
#ifdef EXTMODE_PROTECT_CRITICAL_REGIONS
    /* 
     * disable interrupts around this critical region. We need to 
     * guarantee that reading the head pointer is an atomic 
     * operation.
     */
    EXTMODE_DISABLE_INTERRUPTS;
#endif
    
    /* 
     * Since we are moving the tail forward, we know that head == tail
     * represents an empty buffer and not a full buffer.
     */
    circBuf->empty = (circBuf->tail == circBuf->head);
    
#ifdef EXTMODE_PROTECT_CRITICAL_REGIONS
    /* re-enable interrupts */
    EXTMODE_ENABLE_INTERRUPTS;
#endif

} /* end UploadBufDataSent */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */

/*
 * Macro =======================================================================
 * Move the tail of a circular buffer forward by one time step - accounting for
 * wrapping.
 */
#define MOVE_TAIL_ONESTEP(circBuf, end)                            \
{                                                                  \
    int     nBytesPassedEnd;                                       \
    int     nBytesInStep;                                          \
    int32_T *nBytesPtr = (int32_T *)((circBuf)->tail)+1;           \
                                                                   \
    (void)memcpy(&nBytesInStep, nBytesPtr, sizeof(int32_T));       \
    nBytesInStep += (2*sizeof(int32_T));                           \
    assert(nBytesInStep > 0);                                      \
    (circBuf)->tail += (nBytesInStep);                             \
    nBytesPassedEnd = (int)((circBuf)->tail - (end));              \
    if (nBytesPassedEnd >= 0) {                                    \
        (circBuf)->tail = (circBuf)->buf + nBytesPassedEnd;        \
    }                                                              \
} /* end MOVE_TAIL_ONESTEP */


/*
 * Macro =======================================================================
 * Copy data into the circular buffer.
 */
#define CIRCBUF_COPY_DATA(bufMem, data)                         \
{                                                               \
    (void)memcpy((bufMem).section1, (data), (bufMem).nBytes1);  \
    if ((bufMem).section2 != NULL) {                            \
        char *tmp = ((char *)(data)) + (bufMem).nBytes1;        \
        (void)memcpy((bufMem).section2, tmp, (bufMem).nBytes2); \
    }                                                           \
} /* end CIRCBUF_COPY_DATA */


/* Function ====================================================================
 * Assign sections in the circular buffer for the requested number of bytes
 * (i.e., fill in the bufMem struct).  If there is no room in the circular
 * buffer return an overflow error.
 *
 * NOTE: Do not move the CircularBuffers head forward in this function!  
 *       Only move the tmpHead forward.  The actual head is not advanced
 *       until the entire time point is successfully copied into the buffer.
 *
 *       This function modifies tmpHead to point at the next available 
 *       location.
 *
 *       It is possible for tmpHead to equal the tail upon entry to this
 *       function.  This does not necessarily mean that the buffer is
 *       empty (unwrapped).  It could also mean that the buffer is exactly
 *       full (this is considered as wrapped).
 */
#ifndef EXTMODE_DISABLESIGNALMONITORING
PRIVATE boolean_T UploadBufAssignMem(
    CircularBuf  *circBuf,
    int_T        nBytesToAdd,
    char         **tmpHead,   /* in-out */
    BufMem       *bufMem)     /* out */
{
    int_T       nBytesLeft;
    boolean_T   overFlow  = false;
    char        *end      = circBuf->buf + circBuf->bufSize; /* 1 passed end */

    if ((*tmpHead > circBuf->tail) || circBuf->empty) {
        /* buffer not wrapped */
        nBytesLeft = (int_T)((end - *tmpHead) + (circBuf->tail - circBuf->buf));

        if (nBytesLeft < nBytesToAdd) {
            overFlow = true;
            goto EXIT_POINT;
        }

        if ((*tmpHead + nBytesToAdd) < end) {
            /* still not wrapped */
            bufMem->nBytes1  = nBytesToAdd;
            bufMem->section1 = *tmpHead;

            bufMem->nBytes2  = 0;
            bufMem->section2 = NULL;

            *tmpHead += nBytesToAdd;
        } else {
            /* now we're wrapped */
            bufMem->nBytes1  = (int_T)(end - *tmpHead);
            bufMem->section1 = *tmpHead;

            bufMem->nBytes2  = nBytesToAdd - bufMem->nBytes1;
            bufMem->section2 = (bufMem->nBytes2 > 0) ? circBuf->buf : NULL;

            *tmpHead = circBuf->buf + bufMem->nBytes2;
        }  
    } else {
        /* wrapped */
        nBytesLeft = (int_T)(circBuf->tail - *tmpHead);
        if (nBytesLeft < nBytesToAdd) {
            overFlow = true;
            goto EXIT_POINT;
        }

        bufMem->nBytes1  = nBytesToAdd;
        bufMem->section1 = *tmpHead;

        bufMem->nBytes2  = 0;
        bufMem->section2 = NULL;

        *tmpHead += nBytesToAdd;
    }
    
EXIT_POINT:
    return(overFlow);
} /* end UploadBufAssignMem */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */

/* Function ====================================================================
 * Check the trigger signals for crossings.  Return true if a trigger event is
 * encountered.  It is assumed that the trigger signals are real_T.
 */
#ifndef EXTMODE_DISABLESIGNALMONITORING
PRIVATE boolean_T UploadCheckTriggerSignals(int32_T upInfoIdx)
{
    int          i;
    BdUploadInfo *uploadInfo      = &uploadInfoArray[upInfoIdx];
    TriggerInfo  *trigInfo        = &uploadInfo->trigInfo;
    real_T       *oldTrigSigVals  = trigInfo->oldTrigSigVals;
    real_T       *oldSigPtr       = oldTrigSigVals;
       
    for (i=0; i<trigInfo->trigSignals.nSections; i++) {
        UploadSection *section = &trigInfo->trigSignals.sections[i];
        int_T         nEls     = section->nBytes / sizeof(real_T);

        /*
         * If we have a previous signal value to check, then see if we had
         * a crossing.
         */
        if (trigInfo->haveOldTrigSigVal) {
            int_T   j;
            real_T  level   = trigInfo->level;
            real_T  *rStart = (real_T *)section->start; /* guaranteed by host */
            
            for (j=0; j<nEls; j++) {
                if (trigInfo->lookForRising && 
                    (((rStart[j] >= level) && (oldSigPtr[j] <  level)) ||
                     ((rStart[j] >  level) && (oldSigPtr[j] == level)))) {
                    return(true);
                }
                if (trigInfo->lookForFalling &&
                    (((rStart[j] < level)  && (oldSigPtr[j] >= level)) ||
                     ((rStart[j] == level) && (oldSigPtr[j] >  level)))) {
                    return(true);
                }
            }
        }

        /*
         * Update old signal values.
         */
        (void)memcpy(oldSigPtr, section->start, section->nBytes);
        oldSigPtr += nEls;
    }
    assert(((unsigned char *)oldTrigSigVals) + trigInfo->trigSignals.nBytes == oldSigPtr);
    trigInfo->haveOldTrigSigVal = true;
    return(false);
} /* end UploadCheckTriggerSignals */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */


/* Function ====================================================================
 * If the trigger is in the TRIGGER_FIRED state or we are collecting data for
 * pre-triggering, add data, for each tid with a hit, to the upload buffers.  
 * This function is called from within the appropriate task, once per sample
 * hit.
 *
 * The format of the packet that is sent to the host is as follows:
 *
 * definitions:
 *      pktType - A qualifier indicating any special action that needs to be
 *                taken (e.g., a termination flag following the last data point,
 *                or a flag indicating that it is the first data point after
 *                a trigger event).
 *
 *      nBytes - total number of target bytes for this packet (including the
 *               nBytes field).  nBytes worth of data represents 1 full time
 *               step of the target simulation.
 *
 *      nSys - The number of systems for which this packet contains data.
 *
 *      tid - The tid with which this data is associated.
 *
 *      upInfoIdx - upInfo index
 *
 *      t - simulation time
 *
 *      sysId - The index into the BdUploadInfo.sysTables array so that we can
 *              map the target data back to the appropriate system.  This is
 *              NOT the descendant system index!
 *
 *      data - the target simulation data (in target format)
 *
 * The packet looks like:
 * [nBytes pktType nSys tid upInfoIdx t sysId [data] sysId [data]...]
 *     |                            | |         | |          |
 *     ----------------------------- ----------- ------------
 *          pkt header          sys data     sys data
 *
 * Ints are int32_T.
 */
#ifndef EXTMODE_DISABLESIGNALMONITORING
PUBLIC void UploadBufAddTimePoint(int_T tid, real_T taskTime,
                                  int32_T upInfoIdx)
{
    int_T        preTrig;
    int_T        overFlow;
    TriggerInfo  *trigInfo;
    CircularBuf  *circBuf;
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];

    overFlow   = false;
    trigInfo   = &uploadInfo->trigInfo;
    circBuf    = &uploadInfo->circBufs[tid];
    
    /*
     * Check for transitions from the TRIGGER_ARMED state to either the
     * TRIGGER_FIRED_STATE or the TRIGGER_DELAYED state.  We only do this
     * if it is a sample hit for the trigger signal.  Note that this
     * is the only place in the whole world that the trigger state can
     * move from TRIGGER_ARMED_STATE to TRIGGER_DELAYED or TRIGGER_FIRED.
     */
    if (trigInfo->state == TRIGGER_ARMED) {
        if (trigInfo->trigSignals.nSections == 0) {
            /* short-circuit for manual trigger */
            trigInfo->state = TRIGGER_FIRED;
        } else
            if ((tid == trigInfo->tid) &&
                (UploadCheckTriggerSignals(upInfoIdx))) {
                /* trig signal crossing */
                if (trigInfo->delay == 0) {
                    trigInfo->state = TRIGGER_FIRED;
                    /* 0 unless pre-trig */
                    trigInfo->count = trigInfo->preTrig.count;
                } else {
                    trigInfo->state = TRIGGER_DELAYED;
                    assert(trigInfo->count == 0);
                    
                    /* We will be skipping this step, so the delay count is 1. */
                    trigInfo->count = 1;
                }
            }
    }
    
    preTrig = (trigInfo->state == TRIGGER_ARMED) &&
        (trigInfo->preTrig.duration > 0);
    
    /*
     * Handle adding data to the collection buffers - if needed.
     */
    if (((trigInfo->state == TRIGGER_FIRED) || preTrig) &&
        /* bufSize == 0 means no signals in this tid */
        circBuf->bufSize != 0) {
        
        int32_T     i;
        BufMem      bufMem;
        BufMem      pktStart;
        int_T       size;
        char_T *tmpHead    = circBuf->head;
        const int_T PKT_TYPE_IDX = 0;
        const int_T NBYTES_IDX   = 1;
        const int_T NSYS_IDX     = 2;
        const int_T TID_IDX      = 3;
        const int_T UPINFO_IDX   = 4;

        int32_T intHdr[5] = {0, 0, 0, 0, 0};
        intHdr[UPINFO_IDX] = upInfoIdx;
        
        if (preTrig && (trigInfo->preTrig.count==trigInfo->preTrig.duration)) {
            /* Advance the tail (we don't need the oldest point anymore). */
            char *end = circBuf->buf + circBuf->bufSize;
            MOVE_TAIL_ONESTEP(circBuf, end);
            trigInfo->preTrig.count--;
        }
        
        /*
         * Save some space for the 5 integer values that make up the packet
         * header: [pktType nBytes nSys tid upInfoIdx].
         * The values are filled in later.
         */
        size = 5*sizeof(int32_T);
        overFlow = UploadBufAssignMem(circBuf, size, &tmpHead, &pktStart);
        if (overFlow) goto EXIT_POINT;

        /*
         * We do not want to include the packet type and number of bytes
         * in the size calculation.  Size should represent the payload of
         * this packet.  The packet type and number of bytes represent the
         * packet header and are not included in the payload size.
         */
        size -= 2*sizeof(int32_T);
        intHdr[NBYTES_IDX] += size;
        
        /* time */
        overFlow =
            UploadBufAssignMem(circBuf, sizeof(real_T), &tmpHead, &bufMem);
        if (overFlow) goto EXIT_POINT;
        intHdr[NBYTES_IDX] += sizeof(real_T);
        
        CIRCBUF_COPY_DATA(bufMem, &taskTime);

        /*
         * Check each system for an UploadMap. 
         */
        for (i=0; i<uploadInfo->nSys; i++) {
            const SysUploadTable *sysTable =
                (const SysUploadTable *)&uploadInfo->sysTables[i];
            
            if ( (*sysTable->enableState != SUBSYS_RAN_BC_DISABLE) && 
                 (*sysTable->enableState != SUBSYS_RAN_BC_ENABLE_TO_DISABLE) ) {
                UploadMap *map = sysTable->uploadMap[tid];

                if (map != NULL) {
                    int_T section;
                    intHdr[NSYS_IDX]++;
                    
                    /* Add system index */
                    size = sizeof(int32_T);
                    overFlow =
                        UploadBufAssignMem(circBuf, size, &tmpHead, &bufMem);
                    if (overFlow) goto EXIT_POINT;
                    intHdr[NBYTES_IDX] += size;
                    
                    CIRCBUF_COPY_DATA(bufMem, &i);
                    
                    /* Add data values */
                    for (section=0; section<map->nSections; section++) {
                        UploadSection *sect = &map->sections[section];

                        overFlow = UploadBufAssignMem(
                            circBuf, sect->nBytes, &tmpHead, &bufMem);
                        if (overFlow) goto EXIT_POINT;
                        intHdr[NBYTES_IDX] += sect->nBytes;
                        
                        CIRCBUF_COPY_DATA(bufMem, sect->start);
                    }
                }
            }
        }

        /* If no systems were active then, do nothing. */
        if (intHdr[NSYS_IDX] == 0) goto EXIT_POINT;
        
        /*
         * Go back and finish the header: [nBytes pktType nSys tid]
         */
        
        /* ...pktType */
        intHdr[PKT_TYPE_IDX] = EXT_UPLOAD_LOGGING_DATA;

        /* ...tid */
        intHdr[TID_IDX] = tid;
        CIRCBUF_COPY_DATA(pktStart, intHdr);

        /*
         * Time point successfully added to queue.
         */
        circBuf->head  = tmpHead;
        circBuf->empty = false;
        
        if (preTrig) {
            trigInfo->preTrig.count++;
        }
    }

EXIT_POINT:
    if (!preTrig) {
        if (overFlow) {
            trigInfo->overFlow = true;
            trigInfo->state    = TRIGGER_TERMINATING;
        }
#ifdef VXWORKS
        else if (trigInfo->state == TRIGGER_FIRED) {
            /* allow upload server to run - if data needs to be uploaded */
            semGive(uploadSem);
        }
#endif
    } 
} /* end UploadBufAddTimePoint */


/* Function ====================================================================
 * Called at the base rate, controls the state of data logging including:
 *   - monitoring the trigger signal for a trigger event
 *   - managing transition of most trigger states
 *      o a separate function (UploadCheckEndTrigger manages the duration
 *        count and the transition from fired to terminating at the end
 *        of the data collection event).
 *
 * NOTE:
 *  o This function should be called after mdlOutputs for the base rate
 */
PUBLIC void UploadCheckTrigger(int32_T upInfoIdx, int_T numSampTimes)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    TriggerInfo  *trigInfo   = &uploadInfo->trigInfo;

    if (trigInfo->state == TRIGGER_UNARMED) return;

    if (trigInfo->state == TRIGGER_HOLDING_OFF) {
        if (trigInfo->count++ == trigInfo->holdOff) {
            UploadArmTrigger(upInfoIdx, numSampTimes);
        } else {
            return;
        }
    }

    /*
     * Transitions from the TRIGGER_ARMED_STATE to the TRIGGER_DELAYED
     * state or to the TRIGGER_FIRED_STATE are checked for and realized
     * within the task (tid) associated with the trigger signal.
     * See UploadBufAddTimePoint().
     */ 
    
    /*
     * Look for transitions from the TRIGGER_DELAYED state.  The TRIGGER_FIRED
     * state always follows the TRIGGER_DELAYED state.
     *
     * NOTE: the trigInfo count field is first used to count the trigger delay
     *       and then used to count the trigger duration
     */
    if (trigInfo->state == TRIGGER_DELAYED) {
        if (trigInfo->count++ >= trigInfo->delay) {
            trigInfo->count = trigInfo->preTrig.count; /* 0 unless pre-trig */
            trigInfo->state = TRIGGER_FIRED;
            if (trigInfo->preTrig.duration > 0) {
                trigInfo->preTrig.checkUnderFlow = true;
            }
#ifdef VERBOSE
            printf("\nTrigger fired!\n");
#endif
        }
    }
} /* end UploadCheckTrigger */


/* Function ====================================================================
 * Called at the base rate, controls the state of data logging wrt
 *  o incrementing the duration count
 *  o managing the transition to the trigger terminating state
 *
 * NOTES:
 *  o Call this function at the very end of a step.
 *  o Also see UploadCheckTrigger()
 */
PUBLIC void UploadCheckEndTrigger(int32_T upInfoIdx)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    TriggerInfo *trigInfo    = &uploadInfo->trigInfo;

    if (trigInfo->state == TRIGGER_UNARMED) return;

    /*
     * Increment duration count and terminate the data logging event if
     * the duration has been met.
     */
    if (trigInfo->state == TRIGGER_FIRED) {
        trigInfo->count++;
        if (trigInfo->count == trigInfo->duration) {
            trigInfo->state = TRIGGER_TERMINATING;
        }
    }

#ifdef VXWORKS
    if (trigInfo->state == TRIGGER_TERMINATING) {
        /* Let upload server run to ensure that term pkt is sent to host. */
        semGive(uploadSem);
    }
#endif
} /* end UploadCheckEndTrigger */


/* Function ===================================================================
 * Search through the upload buffers and fill out the internal copy of the
 * buffer list.  It contains a list of all buffer memory (1 entry per non-empty
 * tid buffer) that needs to be sent to the host.  Fill out the fields of the
 * specified ExtBufMemList (passed in by ext_svr) to provide public, read-only
 * access.
 */
PRIVATE void SetExtBufListFields(ExtBufMemList *extBufList,
                                 int32_T       upInfoIdx,
                                 int_T         numSampTimes)
{
    int_T       tid;
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    BufMemList   *bufList    = &uploadInfo->bufMemList;

    bufList->nActiveBufs = 0;

    for (tid=0; tid<numSampTimes; tid++) {
        CircularBuf *circBuf = &uploadInfo->circBufs[tid];

        if (!circBuf->empty) {
            BufMem  *bufMem;
            char_T  *head;
            char_T  *tail   = circBuf->tail;
            int_T   size    = circBuf->bufSize;

            
#ifdef EXTMODE_PROTECT_CRITICAL_REGIONS
            /* 
             * disable interrupts around this critical region. We need to 
             * guarantee that reading the head pointer is an atomic 
             * operation.
             */
            EXTMODE_DISABLE_INTERRUPTS;
#endif

            head = circBuf->head;

            
#ifdef EXTMODE_PROTECT_CRITICAL_REGIONS
            /* re-enable interrupts */
            EXTMODE_ENABLE_INTERRUPTS;
#endif

            /* Validate that head/tail ptrs are within allocated range. */
            assert((head >= circBuf->buf) && (tail >= circBuf->buf));
            assert((head < circBuf->buf + circBuf->bufSize) &&
                   (tail < circBuf->buf + circBuf->bufSize));

            bufMem = &bufList->bufs[bufList->nActiveBufs];
            bufList->tids[bufList->nActiveBufs] = tid;
            assert(bufList->nActiveBufs < bufList->maxBufs);
            bufList->nActiveBufs++;

            bufMem->section1 = tail;
            circBuf->newTail = head;

            if (head > tail) {
                /* not wrapped - only one section required */
                bufMem->nBytes1  = (int_T)(head - tail);

                bufMem->nBytes2  = 0;
                bufMem->section2 = NULL;
            } else {
                /* wrapped - 2 sections required */
                bufMem->nBytes1 = (int_T)(circBuf->buf + size - tail);

                bufMem->nBytes2  = (int_T)(head - circBuf->buf);
                bufMem->section2 = circBuf->buf;
            }
        }
    }

    /*
     * Provide ext_svr with readonly access to the bufMemList.
     */
    extBufList->nActiveBufs = bufList->nActiveBufs;
    extBufList->bufs        = (const BufMem *)bufList->bufs;
    extBufList->tids        = (const int_T *)bufList->tids;
} /* end SetExtBufListFields */


/* Function ===================================================================
 * Set the internal copy of the buffer list to "empty" & fill out the fields
 * of the specified ExtBufMemList (passed in by ext_svr) to provide public,
 * read only access.
 */
PRIVATE void SetExtBufListFieldsForEmptyList(ExtBufMemList *extBufList,
                                             int32_T       upInfoIdx)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    BufMemList   *bufList    = &uploadInfo->bufMemList;

    bufList->nActiveBufs = 0;

    extBufList->nActiveBufs = bufList->nActiveBufs;
    extBufList->bufs        = (const BufMem *)NULL;
} /* end SetExtBufListFieldsForEmptyList */


/* Function ====================================================================
 * Called by ext_svr (background task), this function checks all buffers for
 * data and returns a list of buffer memory to be sent to the host.
 */
PUBLIC void UploadBufGetData(ExtBufMemList *extBufList,
                             int32_T       upInfoIdx,
                             int_T         numSampTimes)
{
    BdUploadInfo *uploadInfo = &uploadInfoArray[upInfoIdx];
    TriggerInfo  *trigInfo   = &uploadInfo->trigInfo;
    TriggerState trigState = trigInfo->state;
 
    if ((trigState == TRIGGER_FIRED) ||
        (trigState == TRIGGER_TERMINATING)) {

        /* Make sure we start with an empty list */
        SetExtBufListFieldsForEmptyList(extBufList, upInfoIdx);
        SetExtBufListFields(extBufList, upInfoIdx, numSampTimes);

        /*
         * If all bufs are empty and we are terminating then we're now done!
         */
        if ((extBufList->nActiveBufs == 0) &&
            (trigState == TRIGGER_TERMINATING)) {

            host_upstatus_is_uploading = false;

            if (trigInfo->holdOff == TRIGMODE_ONESHOT) {
                SendPktToHost(EXT_TERMINATE_LOG_SESSION, sizeof(int32_T),
                              (char *)&upInfoIdx);
                UploadEndLoggingSession(upInfoIdx, numSampTimes);
            } else {
                SendPktToHost(EXT_TERMINATE_LOG_EVENT, sizeof(int32_T),
                              (char *)&upInfoIdx);
                trigInfo->count = 0;
                trigInfo->state = TRIGGER_HOLDING_OFF;
            }
        }
    } else {
        SetExtBufListFieldsForEmptyList(extBufList, upInfoIdx);
    }
} /* end UploadBufGetData */
#endif /* ifndef EXTMODE_DISABLESIGNALMONITORING */


/* [EOF] updown.c */

/* LocalWords:  DType pbuf NPARAMS dtype tran Els EXTMODE
 * LocalWords:  DISABLEPARAMETERTUNING bufs buf blockio tids sys's nbuf sigs
 * LocalWords:  tid's DISABLESIGNALMONITORING uploadinfo NULL'ed vals oneshot
 * LocalWords:  sem svr TRIGMODE ONESTEP CIRCBUF tmp
 */
