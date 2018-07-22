//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_vision_types.h
//
// Code generated for Simulink model 'soccer_vision'.
//
// Model version                  : 1.759
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sat Jul 14 20:07:55 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_soccer_vision_types_h_
#define RTW_HEADER_soccer_vision_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef typedef_map2Dto3D_soccer_vision_T
#define typedef_map2Dto3D_soccer_vision_T

typedef struct {
  int32_T isInitialized;
  real_T ticksUntilNextHit;
} map2Dto3D_soccer_vision_T;

#endif                                 //typedef_map2Dto3D_soccer_vision_T

#ifndef typedef_Line2f_soccer_vision_T
#define typedef_Line2f_soccer_vision_T

typedef struct {
  real_T rho;
  real_T theta;
} Line2f_soccer_vision_T;

#endif                                 //typedef_Line2f_soccer_vision_T

#ifndef typedef_Point2f_soccer_vision_T
#define typedef_Point2f_soccer_vision_T

typedef struct {
  real_T x;
  real_T y;
} Point2f_soccer_vision_T;

#endif                                 //typedef_Point2f_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S2>/Filter'
#ifndef struct_tag_sYtAINW4VTWzUrJuBkN3Z7
#define struct_tag_sYtAINW4VTWzUrJuBkN3Z7

struct tag_sYtAINW4VTWzUrJuBkN3Z7
{
  uint32_T distance;
  uint32_T start;
  uint32_T replicates;
  uint32_T emptyaction;
  uint32_T onlinephase;
  uint32_T options;
  uint32_T maxiter;
  uint32_T display;
};

#endif                                 //struct_tag_sYtAINW4VTWzUrJuBkN3Z7

#ifndef typedef_sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T
#define typedef_sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T

typedef struct tag_sYtAINW4VTWzUrJuBkN3Z7 sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T;

#endif                                 //typedef_sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T

#ifndef struct_tag_spmDXMrbi73sp7C198PnRNF
#define struct_tag_spmDXMrbi73sp7C198PnRNF

struct tag_spmDXMrbi73sp7C198PnRNF
{
  uint32_T Display;
  uint32_T MaxFunEvals;
  uint32_T MaxIter;
  uint32_T TolBnd;
  uint32_T TolFun;
  uint32_T TolTypeFun;
  uint32_T TolX;
  uint32_T TolTypeX;
  uint32_T GradObj;
  uint32_T Jacobian;
  uint32_T DerivStep;
  uint32_T FunValCheck;
  uint32_T Robust;
  uint32_T RobustWgtFun;
  uint32_T WgtFun;
  uint32_T Tune;
  uint32_T UseParallel;
  uint32_T UseSubstreams;
  uint32_T OutputFcn;
};

#endif                                 //struct_tag_spmDXMrbi73sp7C198PnRNF

#ifndef typedef_spmDXMrbi73sp7C198PnRNF_soccer_vision_T
#define typedef_spmDXMrbi73sp7C198PnRNF_soccer_vision_T

typedef struct tag_spmDXMrbi73sp7C198PnRNF
  spmDXMrbi73sp7C198PnRNF_soccer_vision_T;

#endif                                 //typedef_spmDXMrbi73sp7C198PnRNF_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S12>/Label Field Lines'
#ifndef struct_tag_spGKsvEVm7uA89hv31XX4LH
#define struct_tag_spGKsvEVm7uA89hv31XX4LH

struct tag_spGKsvEVm7uA89hv31XX4LH
{
  uint32_T MissingPlacement;
  uint32_T ComparisonMethod;
};

#endif                                 //struct_tag_spGKsvEVm7uA89hv31XX4LH

#ifndef typedef_spGKsvEVm7uA89hv31XX4LH_soccer_vision_T
#define typedef_spGKsvEVm7uA89hv31XX4LH_soccer_vision_T

typedef struct tag_spGKsvEVm7uA89hv31XX4LH
  spGKsvEVm7uA89hv31XX4LH_soccer_vision_T;

#endif                                 //typedef_spGKsvEVm7uA89hv31XX4LH_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S2>/Filter'
#ifndef struct_tag_spLjAHi9hGjlscmJkpZ9eWC
#define struct_tag_spLjAHi9hGjlscmJkpZ9eWC

struct tag_spLjAHi9hGjlscmJkpZ9eWC
{
  real_T MaxIter;
  boolean_T UseParallel;
  boolean_T UseSubstreams;
};

#endif                                 //struct_tag_spLjAHi9hGjlscmJkpZ9eWC

#ifndef typedef_spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T
#define typedef_spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T

typedef struct tag_spLjAHi9hGjlscmJkpZ9eWC
  spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T;

#endif                                 //typedef_spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S12>/Label Field Lines'
#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 //struct_tag_sJCxfmxS8gBOONUZjbjUd9E

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E
  sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T;

#endif                                 //typedef_sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T

// Parameters (default storage)
typedef struct P_soccer_vision_T_ P_soccer_vision_T;

// Forward declaration for rtModel
typedef struct tag_RTM_soccer_vision_T RT_MODEL_soccer_vision_T;

#endif                                 // RTW_HEADER_soccer_vision_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
