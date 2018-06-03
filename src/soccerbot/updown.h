/* 
 * Copyright 1994-2008 The MathWorks, Inc.
 */
#ifndef updown_h
#define updown_h

#include "upsup_public.h"

#define NUM_UPINFOS   2 /* Number of UploadLogInfos in use */

#ifdef __cplusplus

extern "C" {

#endif

extern void      SetParam(RTWExtModeInfo  *ei,
                          const char      *pbuf);

extern void      UploadLogInfoReset(int32_T upInfoIdx);

extern void      UploadPrepareForFinalFlush(int32_T upInfoIdx);

extern void      UploadLogInfoTerm(int32_T upInfoIdx,
                                   int_T   numSampTimes);

extern boolean_T UploadLogInfoInit(RTWExtModeInfo *ei,
                                   int_T          numSampTimes,
                                   const char     *pkt,
                                   int32_T        upInfoIdx);

extern boolean_T UploadInitTrigger(RTWExtModeInfo *ei, 
                                   const char     *pkt,
                                   int32_T        upInfoIdx);

extern void      UploadArmTrigger(int32_T upInfoIdx,
                                  int_T   numSampTimes);

extern void      UploadEndLoggingSession(int32_T upInfoIdx,
                                         int_T   numSampTimes);

extern void      UploadCancelLogging(int32_T upInfoIdx);

extern void      UploadBufDataSent(const int_T tid,
                                   int32_T     upInfoIdx);

extern void      UploadBufAddTimePoint(int_T   tid,
                                       real_T  taskTime,
                                       int32_T upInfoIdx);

extern void      UploadCheckTrigger(int32_T upInfoIdx,
                                    int_T   numSampTimes);

extern void      UploadCheckEndTrigger(int32_T upInfoIdx);

extern void      UploadBufGetData(ExtBufMemList *extBufList,
                                  int32_T       upInfoIdx,
                                  int_T         numSampTimes);

extern boolean_T IsAnyDataReadyForUpload(int32_T upInfoIdx);

#ifdef __cplusplus

}
#endif

#endif /* updown_h */
