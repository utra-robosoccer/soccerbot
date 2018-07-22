/* File: ext_share.h
 * Absract:
 *	External mode shared data structures used by the external communication
 *	mex link, the generated code, and Simulink.
 *
 * Copyright 1994-2005 The MathWorks, Inc.
 *
 */

#ifndef EXTSHARE
#define EXTSHARE

typedef enum { 
    /*================================
     * Packets/actions to target.
     *==============================*/

    /* connection actions */
    EXT_CONNECT,
    EXT_DISCONNECT_REQUEST,
    EXT_DISCONNECT_REQUEST_NO_FINAL_UPLOAD,
    EXT_DISCONNECT_CONFIRMED,

    /* parameter upload/download actions */
    EXT_SETPARAM,
    EXT_GETPARAMS,

    /* data upload actions */
    EXT_SELECT_SIGNALS,
    EXT_SELECT_TRIGGER,
    EXT_ARM_TRIGGER,
    EXT_CANCEL_LOGGING,

    /* model control actions */
    EXT_MODEL_START,
    EXT_MODEL_STOP,
    EXT_MODEL_PAUSE,
    EXT_MODEL_STEP,
    EXT_MODEL_CONTINUE,

    /* data request actions */
    EXT_GET_TIME,

    /*================================
     * Packets/actions from target.
     *==============================*/
    
    /* responses */
    EXT_CONNECT_RESPONSE,   /* must not be 0! */
    EXT_DISCONNECT_REQUEST_RESPONSE,
    EXT_SETPARAM_RESPONSE,
    EXT_GETPARAMS_RESPONSE,
    EXT_MODEL_SHUTDOWN,
    EXT_GET_TIME_RESPONSE,
    EXT_MODEL_START_RESPONSE,
    EXT_MODEL_PAUSE_RESPONSE,
    EXT_MODEL_STEP_RESPONSE,
    EXT_MODEL_CONTINUE_RESPONSE,
    EXT_UPLOAD_LOGGING_DATA,
    EXT_SELECT_SIGNALS_RESPONSE,
    EXT_SELECT_TRIGGER_RESPONSE,
    EXT_ARM_TRIGGER_RESPONSE,
    EXT_CANCEL_LOGGING_RESPONSE,

    /*
     * This packet is sent from the target to signal the end of a data
     * collection event (e.g., each time that the duration is reached).
     * This packet only applies to normal mode (see
     * EXT_TERMINATE_LOG_SESSION)
     */
    EXT_TERMINATE_LOG_EVENT,     

    /*
     * This packet is sent from the target at the end of each data logging
     * session.  This occurs either at the end of a oneshot or at the end
     * of normal mode (i.e., the last in a series of oneshots).
     */
    EXT_TERMINATE_LOG_SESSION,

    /*
     * This packet header is used only in DAEMON_MODE.
     * This is sent from the target to the host after receiving a chunk of
     * data. The host pauses sending data until it receives this
     * acknowledgement packet.
     */
    EXT_DAEMON_ACK,

    EXTENDED = 255          /* reserved for extending beyond 254 ID's */
} ExtModeAction;


typedef enum {
  STATUS_OK,
  NOT_ENOUGH_MEMORY
} ResponseStatus;

typedef enum {
  LittleEndian,
  BigEndian
} MachByteOrder;

#ifndef TARGETSIMSTATUS_DEFINED
#define TARGETSIMSTATUS_DEFINED
typedef enum {
    TARGET_STATUS_NOT_CONNECTED,
    TARGET_STATUS_WAITING_TO_START,
    TARGET_STATUS_STARTING, /* in the process of starting - host waiting 
                               for confirmation */

    TARGET_STATUS_RUNNING,
    TARGET_STATUS_PAUSED
} TargetSimStatus;
#endif

/*
 * The packet header consists of 2 32 bit unsigned ints [type, size].  size
 * is the number of bytes coming after the header.  It is always expressed
 * in target bytes.
 */
typedef struct PktHeader_tag {
    uint32_T type;  /* packet type */
    uint32_T size;  /* number of bytes to follow */
} PktHeader;
#define NUM_HDR_ELS (2)

#ifndef FALSE
enum {FALSE, TRUE};
#endif

#define NO_ERR (0)

#define EXT_NO_ERROR ((boolean_T)(0))
#define EXT_ERROR ((boolean_T)(1))

#define UNKNOWN_BYTES_NEEDED (-1)

#define PRIVATE static
#define PUBLIC

#endif /* __EXTSHARE__ */
