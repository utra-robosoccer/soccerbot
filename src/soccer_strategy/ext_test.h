/*
 * Copyright 1994-2002 The MathWorks, Inc.
 *
 * File: ext_test.h     
 *
 * Abstract:
 *   
 */

#ifndef __EXT_TEST_OBJECT__
#define __EXT_TEST_OBJECT__

#ifdef TMW_EXTMODE_TESTING
extern void rtExtModeTestingInit(void);
extern void rtExtModeTestingKillIfOrphaned(boolean_T pktReceived);
extern void rtExtModeTestingRemoveBatMarker(void);
#else
#define rtExtModeTestingInit()                      /* do nothing */
#define rtExtModeTestingKillIfOrphaned(pktReceived) /* do nothing */
#define rtExtModeTestingRemoveBatMarker()           /* do nothing */
#endif

#endif /* __EXT_TEST_OBJECT__ */
