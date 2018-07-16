/*
 * dsp_iso_math_rt.h - DSP Blockset ISO C math run time library support
 *
 *  Provides (actual or emulated) support for ISO C math library functions.
 *
 *  Copyright 1995-2009 The MathWorks, Inc.
 */

#ifndef dsp_iso_math_rt_h
#define dsp_iso_math_rt_h


/* This is a common #include for both
 * ANSI and ISO math library functions.
 *
 * Ordinarily, if the ISO C math library
 * is supported for this particular target
 * plaform's C compiler, these functions
 * are defined in the standard math.h header.
 */
#include <math.h>


/* _SP_ISO_MATH_NOT_SUPPORTED_ used below to indicate that
 * single-precision floating point ISO C math.h functions
 * are not defined for this particular target platform's
 * C compiler.  Put all logic to determine this here
 * in this one location (for readability/maintenance).
 *
 * Details:
 *
 * "_ISOC_" is sometimes defined if ISO C is used (but don't count on it...) ;-)
 *
 * "__STRICT_ANSI__" is defined in versions of GCC requiring strict ANSI C compliance
 * "__USE_ISOC9X"    is defined in versions of GCC supporting single precision math
 * "_ISOC9X_SOURCE"  is defined in versions of GCC supporting single precision math
 *
 * "__mathf__" is defined for certain TI C6000 family DSP embedded targets,
 *             when single precision floating-point math is supported
 */

#if (  defined(__STRICT_ANSI__) || ( (!defined(_ISOC_)) && (!defined(__USE_ISOC9X)) && (!defined(_ISOC9X_SOURCE)) && (!defined(__mathf__)) )  )
  /* Single-precision ISO C floating point math
   * library fcns are NOT supported on this platform.
   *
   * Instead, define equivalent ANSI C function calls
   * (using supported ANSI C library functions).
   *
   * These definitions come from a subset of functions from
   * the ISO (1995 and 1999) C math library standards.
   *
   * One free example of this library (the GNU C Library) can be found at:
   *
   *   http://www.gnu.org/manual/glibc-2.2.3/html_node/libc_toc.html
   *
   * or on Linux/UNIX platforms that support the ISO math library:
   *
   *   /usr/include/math.h
   */

  /* TRIGONOMETRIC FUNCTIONS */
  #ifndef sinf
  #define sinf(X)     (float)( sin((double)(X)) )
  #endif
  #ifndef cosf
  #define cosf(X)     (float)( cos((double)(X)) )
  #endif
  #ifndef tanf
  #define tanf(X)     (float)( tan((double)(X)) )
  #endif


  /* INVERSE TRIGONOMETRIC FUNCTIONS */
  #ifndef asinf
  #define asinf(X)    (float)( asin( (double)(X)) )
  #endif
  #ifndef acosf
  #define acosf(X)    (float)( acos( (double)(X)) )
  #endif
  #ifndef atanf
  #define atanf(X)    (float)( atan( (double)(X)) )
  #endif
  #ifndef atan2f
  #define atan2f(Y,X) (float)( atan2((double)(Y), (double)(X)) )
  #endif


  /* HYPERBOLIC FUNCTIONS */
  #ifndef sinhf
  #define sinhf(X)     (float)( sinh((double)(X)) )
  #endif
  #ifndef coshf
  #define coshf(X)     (float)( cosh((double)(X)) )
  #endif
  #ifndef tanhf
  #define tanhf(X)     (float)( tanh((double)(X)) )
  #endif


  /* INVERSE HYPERBOLIC FUNCTIONS */
  #ifndef asinhf
  #define asinhf(X)    (float)( asinh((double)(X)) )
  #endif
  #ifndef acoshf
  #define acoshf(X)    (float)( acosh((double)(X)) )
  #endif
  #ifndef atanhf
  #define atanhf(X)    (float)( atanh((double)(X)) )
  #endif


  /* EXPONENTIATION AND LOGARITHMS */
  #ifndef expf
  #define expf(X)      (float)( exp(  (double)(X)) )
  #endif
  #ifndef exp2f
  #define exp2f(X)     (float)( exp2( (double)(X)) )
  #endif
  #ifndef logf
  #define logf(X)      (float)( log(  (double)(X)) )
  #endif
  #ifndef log10f
  #define log10f(X)    (float)( log10((double)(X)) )
  #endif
  #ifndef log2f
  #define log2f(X)     (float)( log2( (double)(X)) )
  #endif
  #ifndef logbf
  #define logbf(X)     (float)( logb( (double)(X)) )
  #endif
  #ifndef ilogbf
  #define ilogbf(X)    (float)( ilogb((double)(X)) )
  #endif
  #ifndef powf
  #define powf(X,Y)    (float)( pow(  (double)(X), (double)(Y)) )
  #endif
  #ifndef sqrtf
  #define sqrtf(X)     (float)( sqrt( (double)(X)) )
  #endif
  #ifndef cbrtf
  #define cbrtf(X)     (float)( cbrt( (double)(X)) )
  #endif
  #ifndef hypotf
  #define hypotf(X,Y)  (float)( hypot((double)(X), (double)(Y)) )
  #endif
  #ifndef expmlf
  #define expmlf(X)    (float)( expml((double)(X)) )
  #endif
  #ifndef log1pf
  #define log1pf(X)    (float)( log1p((double)(X)) )
  #endif


  /* GENERAL FLOATING POINT MATH FUNCTIONS */
  #ifndef ceilf
  #define ceilf(X)      (float)( ceil( (double)(X)) )
  #endif
  #ifndef fabsf
  #define fabsf(X)      (float)( fabs( (double)(X)) )
  #endif
  #ifndef floorf
  #define floorf(X)     (float)( floor((double)(X)) )
  #endif
  #ifndef fmodf
  #define fmodf(X,Y)    (float)( fmod( (double)(X), (double)(Y)) )
  #endif
  #ifndef frexpf
  #define frexpf(X,EXP) (float)( frexp((double)(X), EXP        ) )
  #endif
  #ifndef ldexpf
  #define ldexpf(X,N)   (float)( ldexp((double)(X), N          ) )
  #endif
  /* NOTE modff(X,IP)
   * not yet implemented - since it
   * requires a true C function call
   * to handle single and double ptr
   * in ANSI C modf library function.
   * Add fcn to RT C library if needed.
   */


  /* SPECIAL FUNCTIONS */
  #ifndef erff
  #define erff(X)      (float)( erf(   (double)(X)) )
  #endif
  #ifndef erfcf
  #define erfcf(X)     (float)( erfc(  (double)(X)) )
  #endif
  #ifndef lgammaf
  #define lgammaf(X)   (float)( lgamma((double)(X)) )
  #endif
  #ifndef tgammaf
  #define tgammaf(X)   (float)( tgamma((double)(X)) )
  #endif


  /* SPECIAL PLATFORM-DEPENDENT FUNCTION EXTENSIONS THAT WERE
   * NOT PART OF THE ORIGINAL ISO STDS (USE AT YOUR OWN RISK)
   */
  #ifndef lgammaf_r
  #define lgammaf_r(X) (float)( lgamma_r((double)(X)) )
  #endif
  #ifndef gammaf
  #define gammaf(X)    (float)( gamma(   (double)(X)) )
  #endif
  #ifndef j0f
  #define j0f(X)       (float)( j0(      (double)(X)) )
  #endif
  #ifndef j1f
  #define j1f(X)       (float)( j1(      (double)(X)) )
  #endif
  #ifndef jnf
  #define jnf(N,X)     (float)( jn( (N), (double)(X)) )
  #endif
  #ifndef y0f
  #define y0f(X)       (float)( y0(      (double)(X)) )
  #endif
  #ifndef y1f
  #define y1f(X)       (float)( y1(      (double)(X)) )
  #endif
  #ifndef ynf
  #define ynf(N,X)     (float)( yn( (N), (double)(X)) )
  #endif


#endif


#endif /* dsp_iso_math_rt_h */

/* [EOF] dsp_iso_math_rt.h */
