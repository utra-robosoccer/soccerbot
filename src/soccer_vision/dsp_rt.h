/*
 *  dsp_rt.h
 *
 *   Only contains run-time functions suitable for inclusion in
 *   generated code.  No _S_i_m_S_t_r_u_c_t_ code is allowed in
 *   this file. (The _'s are present because we have a test in
 *   place to prohibit files containing that string from being
 *   submitted to the src/rt directory!)
 *
 *  Copyright 1995-2015 The MathWorks, Inc.
 */
#ifndef dsp_rt_h
#define dsp_rt_h

#ifndef DSPBLKS_SIM_SFCN
  #ifndef MWDSPSIMONLY_DO_NOT_INCLUDE_RTWTYPES_H
    #ifndef __TMWTYPES__
      /* The following file must be included
       * for RTW code generation targets ONLY.
       * It CANNOT be mixed with tmwtypes.h
       */
      #include "rtwtypes.h"
    #endif /* __TMWTYPES__ */
  #endif /* MWDSPSIMONLY_DO_NOT_INCLUDE_RTWTYPES_H */
#else
  /* DSPBLKS_SIM_SFCN is defined
   *
   * tmwtypes.h is needed to build sim-support libraries
   */
  #include "version.h" /* includes tmwtypes.h after compiler switch magic */
#endif /* DSPBLKS_SIM_SFCN */

#include <float.h>
#include "dsp_iso_math_rt.h"
#include <string.h>

/* MWDSP_DYNAMIC_RTLIBS #defined by individual RT lib module makefiles */
#ifdef MWDSP_DYNAMIC_RTLIBS
    #include "package.h"
#endif

/*
 * ----------------------------------------------------------
 * Common (platform-independent) constants
 * ----------------------------------------------------------
 */

#define DSP_TWO_PI     6.283185307179586476925286766559005768394
#define DSP_PI         3.1415926535897932384626433832795
#define DSP_PI_OVER_2  1.5707963267948966192313216916398
#define DSP_PI_OVER_4  0.7853981633974483096156608458199
#define DSP_PI_OVER_8  0.39269908169872415480783042290995
#define MIN_real64_T   DBL_MIN
#define MIN_real32_T   FLT_MIN
#define EPS_real64_T   DBL_EPSILON
#define EPS_real32_T   FLT_EPSILON

/* LCC uses special values for the following two constants.
 * These values are redefined (from float.h) in
 * $matlabroot/toolbox/stateflow/internal/lcc/devel/src/lcc/c.h
 * and are used in
 * $matlabroot/toolbox/stateflow/internal/lcc/devel/src/lcc/lex.c
 * to check the ranges of floating-point constants.
 */
#ifdef __LCC__
  #define MAX_real64_T 1e37
  #define MAX_real32_T 1e37
#else
  #define MAX_real64_T DBL_MAX
  #define MAX_real32_T FLT_MAX
#endif

#define MIN_real_T MIN_real64_T
#define EPS_real_T EPS_real64_T
#define MAX_real_T MAX_real64_T

/*
 * ----------------------------------------------------------
 * Common macros
 * ----------------------------------------------------------
 */

#ifndef MAX
#define MAX(a,b) ((a)>(b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#endif

/* Clip value to lie between lower Bound and upper Bound  */
#ifndef CLIP
#define CLIP(val,low,up)                     \
   ((val)>(up)) ? (up)                       \
                : ( ((val)<(low)) ? (low)    \
                                  : (val))
#endif

/*
 *  4 forms of complex multiply:
 */
#define CMULT_RE(X,Y)        ( (X).re * (Y).re - (X).im * (Y).im)
#define CMULT_IM(X,Y)        ( (X).re * (Y).im + (X).im * (Y).re)

#define CMULT_XCONJ_RE(X,Y)  ( (X).re * (Y).re + (X).im * (Y).im)
#define CMULT_XCONJ_IM(X,Y)  ( (X).re * (Y).im - (X).im * (Y).re)

#define CMULT_YCONJ_RE(X,Y)  ( (X).re * (Y).re + (X).im * (Y).im)
#define CMULT_YCONJ_IM(X,Y)  (-(X).re * (Y).im + (X).im * (Y).re)

#define CMULT_XYCONJ_RE(X,Y) ( (X).re * (Y).re - (X).im * (Y).im)
#define CMULT_XYCONJ_IM(X,Y) (-(X).re * (Y).im - (X).im * (Y).re)

/* Complex conjugate: */
#define CCONJ(X,Y)     \
{                      \
   (Y).re =  (X).re;   \
   (Y).im = -((X).im); \
}

/*
 *  Complex magnitude squared ( X * conj(X), or |X|^2 )
 *   CMAGSQ: arg is a complex struct
 *   CMAGSQp: arg is a pointer to a complex struct
 */
#define CMAGSQ(X)   ((X).re   * (X).re   + (X).im   * (X).im)
#define CMAGSQp(pX) ((pX)->re * (pX)->re + (pX)->im * (pX)->im)

/*
 * Quick-and-dirty (approximate) complex absolute value:
 */
#define CQABS(X) (fabs((X).re) + fabs((X).im))

/*
 *  Complex reciprocal: C = 1 / B  (A=1)
 */
#define CRECIP(B,C)                     \
{                                       \
    const real_T _s = 1.0 / CQABS(B);   \
    real_T  _d;                         \
    creal_T _bs;                        \
    _bs.re = (B).re * _s;               \
    _bs.im = (B).im * _s;               \
    _d = 1.0 / CMAGSQ(_bs);             \
    (C).re = ( _s * _bs.re) * _d;       \
    (C).im = (-_s * _bs.im) * _d;       \
}

/*
 * Complex division: C = A / B
 */
#define CDIV(A,B,C)                             \
{                                               \
    if ((B).im == 0.0) {                        \
	(C).re = (A).re / (B).re;               \
	(C).im = (A).im / (B).re;               \
    } else {                                    \
        const real_T _s = 1.0 / CQABS(B);       \
        real_T  _d;                             \
        creal_T _as, _bs;                       \
        _as.re = (A).re * _s;                   \
        _as.im = (A).im * _s;                   \
        _bs.re = (B).re * _s;                   \
        _bs.im = (B).im * _s;                   \
        _d = 1.0 / CMAGSQ(_bs);                 \
        (C).re = CMULT_YCONJ_RE(_as, _bs) * _d; \
        (C).im = CMULT_YCONJ_IM(_as, _bs) * _d; \
    }                                           \
}

/*
 *  Hypotenuse: c = sqrt(a^2 + b^2)
 */
#define CHYPOT(A,B,C)                          \
{                                              \
    if (fabs(A) > fabs(B)) {                   \
        real_T _tmp = (B)/(A);                 \
        (C) = (fabs(A)*sqrt(1+_tmp*_tmp));     \
    } else {                                   \
    if ((B) == 0.0) {                          \
            (C) = 0.0;                         \
        } else {                               \
            real_T _tmp = (A)/(B);             \
            (C) = (fabs(B)*sqrt(1+_tmp*_tmp)); \
        }                                      \
    }                                          \
}

/*
 *  Complex modulus: Y = abs(X)
 */
#define CABS(X,Y) CHYPOT((X).re, (X).im, (Y))

/*
 *  Single precision macros for complex arithmetic
 */

/*
 * Find absolute value for single precision - Single precision
 *************************************************************************
 * NOTE: this definition will be removed in the near future.
 *       all current instances of "FABS32" in dspblks were repalced with "fabsf".
 *       please don't use FABS32 in your code development!!! 
 *   Ref. Gecko Record 232828
 */
#define FABS32(x)    (((x) > 0.0F) ? (x) : (-(x)))


/*
 * Quick-and-dirty (approximate) complex absolute value: Single precision
 */
#define CQABS32(X) (fabsf((X).re) + fabsf((X).im))

/*
 *  Complex reciprocal: C = 1 / B  (A=1) : Single precision
 */
#define CRECIP32(B,C)                       \
{                                           \
    const real32_T _s = 1.0F / CQABS32(B);  \
    real32_T  _d;                           \
    creal32_T _bs;                          \
    _bs.re = (B).re * _s;                   \
    _bs.im = (B).im * _s;                   \
    _d = 1.0F / CMAGSQ(_bs);                \
    (C).re = ( _s * _bs.re) * _d;           \
    (C).im = (-_s * _bs.im) * _d;           \
}

/*
 * Complex division: C = A / B : Single Precision
 */
#define CDIV32(A,B,C)                            \
{                                                \
    if ((B).im == 0.0F) {                        \
	(C).re = (A).re / (B).re;                    \
	(C).im = (A).im / (B).re;                    \
    } else {                                     \
        const real32_T _s = 1.0F / CQABS32(B);   \
        real32_T  _d;                            \
        creal32_T _as, _bs;                      \
        _as.re = (A).re * _s;                    \
        _as.im = (A).im * _s;                    \
        _bs.re = (B).re * _s;                    \
        _bs.im = (B).im * _s;                    \
        _d = 1.0F / CMAGSQ(_bs);                 \
        (C).re = CMULT_YCONJ_RE(_as, _bs) * _d;  \
        (C).im = CMULT_YCONJ_IM(_as, _bs) * _d;  \
    }                                            \
}

/*
 *  Hypotenuse: c = sqrt(a^2 + b^2) : Single Precision
 */
#define CHYPOT32(A,B,C)                                                \
{                                                                      \
    if (fabsf(A) > fabsf(B)) {                                       \
        real32_T _tmp = (B)/(A);                                       \
        (C) = (fabsf(A)*(real32_T)sqrtf((real_T)(1.0F+_tmp*_tmp)));    \
    } else {                                                           \
    if ((B) == 0.0F) {                                                 \
            (C) = 0.0F;                                                \
        } else {                                                       \
            real32_T _tmp = (A)/(B);                                   \
            (C) = (fabsf(B)*(real32_T)sqrtf((real_T)(1.0F+_tmp*_tmp)));\
        }                                                              \
    }                                                                  \
}

/*
 *  Complex modulus: Y = abs(X) : Single Precision
 */
#define CABS32(X,Y) CHYPOT32((X).re, (X).im, (Y))


/*
 * ----------------------------------------------------------
 * Definitions used for the inlining mechanism
 * for the DSP RunTime C Target Library
 * ----------------------------------------------------------
 */

#define MWDSP_IDECL extern
#define MWSP_IDECL extern

/*
  DSPBLKS_SIM_SFCN is defined in dsp_bld_defs.gnu when compiling DSPBLKS S-Functions
  and the dsp_sim simulation library and is used here to #define EXPORT_FCN or not,
  depending on whether or not an RT lib shared object module is currently being built...
*/
#ifndef DSPBLKS_SIM_SFCN
  #ifndef EXPORT_FCN
    #define EXPORT_FCN MWDSP_IDECL
  #endif
#endif

#define NONINLINED_EXPORT_FCN EXPORT_FCN


#endif  /* dsp_rt_h */
