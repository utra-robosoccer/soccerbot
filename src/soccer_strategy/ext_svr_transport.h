/*
 * Copyright 1994-2002 The MathWorks, Inc.
 *
 * File: ext_svr_transport.h     
 *
 * Abstract:
 *  PUBLIC interface for ext_svr_transport.c.
 */

/********************* DO NOT CHANGE BELOW THIS LINE **************************
 *                                                                            *
 * The function prototypes below define the interface between the target-     *
 * side external mode and ext_svr_transport.  They should not need to be      *
 * changed.  Only the implementation of these functions in ext_svr_transport.c*
 * should need to be modified.                                                *
 *                                                                            *
 ******************************************************************************/

/*
 * Export the user data as an 'opaque' or 'incomplete' data type.  ext_svr
 * may reference it (i.e., pass pointers to it, but it can not dereference
 * the pointer).
 */
typedef struct ExtUserData_tag ExtUserData;


/*
 * Define the transport interface.
 */

extern boolean_T ExtInit(ExtUserData *UD);

extern boolean_T ExtOpenConnection(
    ExtUserData *UD,
    boolean_T   *outConnectionMade);

extern void ExtCloseConnection(ExtUserData *UD);

extern void ExtShutDown(ExtUserData *UD);

extern const char_T *ExtProcessArgs(
    ExtUserData   *UD,
    const int_T   argc,
    const char_T  *argv[]);

extern boolean_T ExtWaitForStartPktFromHost(ExtUserData *UD);

extern ExtUserData *ExtUserDataCreate(void);

extern void ExtUserDataDestroy(ExtUserData *userData);

#ifdef VXWORKS
extern void ExtUserDataSetPort(ExtUserData *UD, const int_T port);
#endif

extern boolean_T ExtGetHostPkt(
    const ExtUserData *UD,
    const int         nBytesToGet,
    int               *nBytesGot,
    char              *dst);

extern boolean_T ExtSetHostPkt(
    const ExtUserData *UD,
    const int         nBytesToSet,
    const char        *src,
    int               *nBytesSet);

extern void ExtModeSleep(
    const ExtUserData *UD,
    const long        sec,  
    const long        usec);

extern void ExtForceDisconnect(ExtUserData *UD);

