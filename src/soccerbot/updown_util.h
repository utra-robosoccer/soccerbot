#ifndef __UPDOWN_UTIL__
#define __UPDOWN_UTIL__

/*
 * Copyright 1994-2002 The MathWorks, Inc.
 *
 * File    : updown_util.h
 * Abstract:
 *  'assert' support for external MATLAB files.
 */

/*
 * Set ASSERTS_ON to 1 turn asserts on, 0 otherwise.
 */
#define ASSERTS_ON (0)

/*------------------------------------------------------------------------------
 * Do not modify below this line.
 *----------------------------------------------------------------------------*/
#if ASSERTS_ON
#include <assert.h>
#else
#define assert(dum) /* do nothing */
#endif

#ifdef VERBOSE
# define PRINT_VERBOSE(args) (void)printf args
#else
#define PRINT_VERBOSE(args) /* do nothing */
#endif

#endif /* __UPDOWN_UTIL__ */
