/*
 * Copyright 1994-2004 The MathWorks, Inc.
 *
 * File: ext_types.h     
 *
 * Abstract:
 */

#ifndef __EXT_TYPES__
#define __EXT_TYPES__

/* Additional types required for Simulink External Mode */
#ifndef fcn_call_T
# define fcn_call_T real_T
#endif
#ifndef action_T
# define action_T real_T
#endif

/****************************************
 * Dynamic vs. Static memory allocation *
 ****************************************/
#ifdef EXTMODE_STATIC
#  include "mem_mgr.h"
#  define malloc ExtModeMalloc
#  define calloc ExtModeCalloc
#  define free   ExtModeFree
#endif

/****************************************
 * Integer only code                    *
 ****************************************/
#ifdef EXT_MODE
#  if INTEGER_CODE == 1
#    undef  real_T
#    define real_T int32_T

#    undef  real32_T
#    define real32_T int32_T

#    undef  double
#    define double int32_T

#    undef  time_T
#    define time_T int32_T
#  endif /* INTEGER_CODE == 1 */
#endif /* #ifdef EXTMODE */


#endif /* __EXT_TYPES__ */

/* [EOF] ext_types.h */
