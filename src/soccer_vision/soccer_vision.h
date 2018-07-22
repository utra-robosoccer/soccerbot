//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_vision.h
//
// Code generated for Simulink model 'soccer_vision'.
//
<<<<<<< HEAD
// Model version                  : 1.759
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sat Jul 14 20:07:55 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
=======
// Model version                  : 1.770
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sun Jul 15 23:50:31 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_soccer_vision_h_
#define RTW_HEADER_soccer_vision_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef soccer_vision_COMMON_INCLUDES_
# define soccer_vision_COMMON_INCLUDES_
#include <math.h>
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
<<<<<<< HEAD
=======
#include "slros_initialize.h"
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
#include "viphough_rt.h"
#endif                                 // soccer_vision_COMMON_INCLUDES_

#include "soccer_vision_types.h"

// Shared type includes
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include "rt_defines.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
# define rtmGetRTWExtModeInfo(rtm)     ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               (&(rtm)->Timing.taskTime0)
#endif

// Block signals (default storage)
typedef struct {
<<<<<<< HEAD
  real_T HoughTransform_o1[71820];     // '<S10>/Hough Transform'
  uint8_T CoverAboveHorizon[307200];   // '<S2>/Cover Above Horizon'
  real_T HoughTransform_o3[399];       // '<S10>/Hough Transform'
  real_T MatrixConcatenate[399];       // '<S12>/Matrix Concatenate'
  real_T HoughTransform_o2[180];       // '<S10>/Hough Transform'
  real_T D_data[120];
=======
  uint32_T b_index_data[921603];
  uint32_T indexBase_data[307201];
  uint32_T y_data[307201];
  uint8_T fullImage[921600];
  uint8_T rawImage_data[921600];
  uint8_T data_data[921600];
  uint8_T tmp_data[921600];
  uint8_T CoverAboveHorizon[307200];
  uint8_T Resize[19200];               // '<S10>/Resize'
  boolean_T EdgeDetection[19200];      // '<S10>/Edge Detection'
  real_T Gain[399];                    // '<S10>/Gain'
  int32_T j_data[640];
  int32_T i_data[480];
  real_T D_data[120];
  SL_Bus_soccer_vision_sensor_msgs_Image In1;// '<S6>/In1'
  real_T HoughTransform_o1[71820];     // '<S10>/Hough Transform'
  real_T HoughTransform_o2[180];       // '<S10>/Hough Transform'
  real_T HoughTransform_o3[399];       // '<S10>/Hough Transform'
  SL_Bus_soccer_vision_sensor_msgs_Image b_varargout_2;
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T lines[40];
  real_T sampleDist[21];
  uint32_T FindLocalMaxima_o1[40];     // '<S10>/Find Local Maxima'
  real_T rho[20];                      // '<S10>/Variable Selector'
  real_T VariableSelector1[20];        // '<S10>/Variable Selector1'
<<<<<<< HEAD
  uint8_T TmpSignalConversionAtCoverAboveHorizonInport1[307200];
  uint8_T Resize[19200];               // '<S10>/Resize'
  boolean_T EdgeDetection[19200];      // '<S10>/Edge Detection'
  real_T unusedU0[20];
  real_T d[20];
  real_T d_m[20];
  real_T rtb_netLinesNew_c[18];
  int32_T HoughLines[36];              // '<S2>/Hough Lines'
=======
  real_T unusedU0[20];
  real_T d[20];
  real_T d_m[20];
  real_T MatrixConcatenate[18];        // '<S12>/Matrix Concatenate'
  char_T tmp_data_c[128];
  uint8_T busstruct_Encoding_data[128];
  uint8_T data_data_k[128];
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T linesFilteredBest_data[12];
  real_T C_data[12];
  real_T fieldLinesNew[10];            // '<S12>/Label Field Lines'
  int32_T idx[20];
  int32_T previdx[20];
  int32_T moved[20];
  int32_T nidx[20];
<<<<<<< HEAD
  real_T Selector[9];                  // '<S2>/Selector'
  real_T netLinesNew[8];               // '<S12>/Label Net Lines'
  real_T rhoField[6];
  real_T thetaField[6];
  real_T rhoNet[6];
  real_T thetaNet[6];
=======
  real_T netLinesNew[8];               // '<S12>/Label Net Lines'
  real_T thetaField[6];                // '<S2>/Filter'
  real_T rhoField[6];                  // '<S2>/Filter'
  real_T thetaNet[6];                  // '<S2>/Filter'
  real_T rhoNet[6];                    // '<S2>/Filter'
  int32_T HoughLines[36];              // '<S2>/Hough Lines'
  real_T sumd_data[6];
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T x[5];
  real_T xwork[5];
  real_T x4[4];
  real_T A[4];
  int32_T crows_data[6];
  int32_T nonEmpties_data[6];
  int32_T empties_data[6];
  int32_T changed_data[6];
  boolean_T b[20];
  int32_T iidx[5];
  int32_T iwork[5];
  int32_T tmpOutRC[4];
<<<<<<< HEAD
  real_T thetaNet_k[2];
  real_T thetaNet_c[2];
=======
  real_T dv0[2];
  real_T dv1[2];
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  Line2f_soccer_vision_T lobj_7;
  Line2f_soccer_vision_T horizonLine;
  Point2f_soccer_vision_T inter;
  Point2f_soccer_vision_T b_inter;
<<<<<<< HEAD
=======
  int32_T rawImage_size[3];
  int32_T data_size[3];
  int32_T tmp_size[3];
  int64_T i0;
  uint64_T u0;
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T sumdbest;
  real_T c_y;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T totalLineCount;
<<<<<<< HEAD
=======
  real_T d0;
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T b_index;
  real_T prevtotsumD;
  real_T totsumD;
  real_T denom;
  real_T pt;
  real_T u;
<<<<<<< HEAD
  int32_T C_size[2];
  int32_T D_size[2];
=======
  uint32_T u_c[2];
  int32_T C_size[2];
  int32_T busstruct_Encoding_size[2];
  int32_T tmp_size_b[2];
  int32_T D_size[2];
  int8_T idx4[4];
  int32_T q;
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  int32_T imgCol;
  int32_T imgRow;
  int32_T accumOne;
  int32_T accumTwo;
  int32_T accumFour;
  int32_T imgIdx_d;
  int32_T imgIdx;
<<<<<<< HEAD
  int32_T i;
  int32_T sumd_size;
  int32_T q0;
  int32_T nNaNs;
  int32_T pidx;
  int32_T nNonEmpty;
  uint32_T horizonY;
=======
  int32_T sumd_size;
  int32_T q0;
  int32_T c;
  int32_T h;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T busstruct_Data_size;
  int32_T nNaNs;
  int32_T ib;
  int32_T pidx;
  int32_T nNonEmpty;
  int32_T nchanged;
  int32_T lonely;
  uint32_T horizonY;
  uint8_T ReadImage_o2;                // '<S4>/Read Image'
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
} B_soccer_vision_T;

// Block states (default storage) for system '<Root>'
typedef struct {
<<<<<<< HEAD
  real_T FindLocalMaxima_TEMP_IN_DWORKS[71820];// '<S10>/Find Local Maxima'
  uint8_T Resize_IntBuffer[76800];     // '<S10>/Resize'
  int32_T EdgeDetection_GV_SQUARED_DW[19200];// '<S10>/Edge Detection'
  int32_T EdgeDetection_GH_SQUARED_DW[19200];// '<S10>/Edge Detection'
  int32_T EdgeDetection_GRAD_SUM_DW[19200];// '<S10>/Edge Detection'
  uint8_T Resize_LineBuffer[480];      // '<S10>/Resize'
=======
  robotics_slros_internal_block_ReadImage_soccer_vision_T obj;// '<S4>/Read Image' 
  real_T FindLocalMaxima_TEMP_IN_DWORKS[71820];// '<S10>/Find Local Maxima'
  uint8_T Resize_IntBuffer[76800];     // '<S10>/Resize'
  int32_T EdgeDetection_GH_SQUARED_DW[19200];// '<S10>/Edge Detection'
  int32_T EdgeDetection_GRAD_SUM_DW[19200];// '<S10>/Edge Detection'
  uint8_T Resize_LineBuffer[480];      // '<S10>/Resize'
  robotics_slros_internal_block_Subscriber_soccer_vision_T obj_h;// '<S5>/SourceBlock' 
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T Memory_PreviousInput[8];      // '<S12>/Memory'
  real_T Memory1_PreviousInput[10];    // '<S12>/Memory1'
  int32_T EdgeDetection_VO_DW[6];      // '<S10>/Edge Detection'
  int32_T EdgeDetection_HO_DW[6];      // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOU_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOD_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOL_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOR_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOU_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOD_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOL_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOR_DW[6];     // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOUL_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOLL_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOUR_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_VOLR_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOUL_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOLL_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOUR_DW[6];    // '<S10>/Edge Detection'
  int32_T EdgeDetection_HOLR_DW[6];    // '<S10>/Edge Detection'
  uint32_T state[625];                 // '<S2>/Filter'
  int32_T EdgeDetection_MEAN_FACTOR_DW;// '<S10>/Edge Detection'
<<<<<<< HEAD
=======
  int8_T EnabledSubsystem_SubsysRanBC; // '<S5>/Enabled Subsystem'
  uint8_T ColorSpaceConversion_DWORK1[921600];// '<S4>/Color Space  Conversion'
  int32_T EdgeDetection_GV_SQUARED_DW[19200];// '<S10>/Edge Detection'
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
} DW_soccer_vision_T;

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: HoughTransform_SINE_TABLE_RTP
  //  Referenced by: '<S10>/Hough Transform'

  real_T HoughTransform_SINE_TABLE_RTP[91];

  // Computed Parameter: HoughTransform_FIRSTRHO_RTP
  //  Referenced by: '<S10>/Hough Transform'

  real_T HoughTransform_FIRSTRHO_RTP;

  // Computed Parameter: EdgeDetection_VRO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  int32_T EdgeDetection_VRO_RTP[6];

  // Computed Parameter: EdgeDetection_VCO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  int32_T EdgeDetection_VCO_RTP[6];

  // Computed Parameter: EdgeDetection_HRO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  int32_T EdgeDetection_HRO_RTP[6];

  // Computed Parameter: EdgeDetection_HCO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  int32_T EdgeDetection_HCO_RTP[6];

  // Computed Parameter: EdgeDetection_VC_RTP
  //  Referenced by: '<S10>/Edge Detection'

  int32_T EdgeDetection_VC_RTP[6];

  // Computed Parameter: EdgeDetection_HC_RTP
  //  Referenced by: '<S10>/Edge Detection'

  int32_T EdgeDetection_HC_RTP[6];
} ConstP_soccer_vision_T;

// Parameters (default storage)
struct P_soccer_vision_T_ {
  real_T clusterProximityThreshold;    // Variable: clusterProximityThreshold
                                       //  Referenced by: '<S2>/Filter'

  real_T lineTrackingThreshold;        // Variable: lineTrackingThreshold
                                       //  Referenced by: '<S12>/Label Field Lines'

  real_T netAngleThreshold;            // Variable: netAngleThreshold
                                       //  Referenced by: '<S2>/Filter'

  real_T scalerho;                     // Variable: scalerho
                                       //  Referenced by:
                                       //    '<S2>/Filter'
                                       //    '<S12>/Label Field Lines'

  real_T FindLocalMaxima_threshold;    // Mask Parameter: FindLocalMaxima_threshold
                                       //  Referenced by: '<S10>/Find Local Maxima'

  int32_T EdgeDetection_thresholdTuning;// Mask Parameter: EdgeDetection_thresholdTuning
                                        //  Referenced by: '<S10>/Edge Detection'

<<<<<<< HEAD
=======
  SL_Bus_soccer_vision_sensor_msgs_Image Out1_Y0;// Computed Parameter: Out1_Y0
                                                 //  Referenced by: '<S6>/Out1'

  SL_Bus_soccer_vision_sensor_msgs_Image Constant_Value;// Computed Parameter: Constant_Value
                                                        //  Referenced by: '<S5>/Constant'

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  real_T Constant6_Value;              // Expression: 700
                                       //  Referenced by: '<S2>/Constant6'

  real_T Gain_Gain;                    // Expression: 4
                                       //  Referenced by: '<S10>/Gain'

  real_T Constant2_Value;              // Expression: 500
                                       //  Referenced by: '<S2>/Constant2'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S2>/Constant3'

  real_T Constant4_Value;              // Expression: 0
                                       //  Referenced by: '<S2>/Constant4'

  real_T Constant1_Value;              // Expression: 0
                                       //  Referenced by: '<S2>/Constant1'

  real_T Memory_InitialCondition[8];   // Expression: [0,0;0,0;0,0;0,0]
                                       //  Referenced by: '<S12>/Memory'

  real_T Memory1_InitialCondition[10]; // Expression: [0,0;0,0;0,0;0,0;0,0]
                                       //  Referenced by: '<S12>/Memory1'

  int16_T CoverAboveHorizon_RTP_OPACITY;// Computed Parameter: CoverAboveHorizon_RTP_OPACITY
                                        //  Referenced by: '<S2>/Cover Above Horizon'

};

// Real-time Model Data Structure
struct tag_RTM_soccer_vision_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
<<<<<<< HEAD
=======
    uint32_T clockTick1;
    struct {
      uint8_T TID[2];
    } TaskCounters;

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_soccer_vision_T soccer_vision_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
extern B_soccer_vision_T soccer_vision_B;

// Block states (default storage)
extern DW_soccer_vision_T soccer_vision_DW;

// Constant parameters (default storage)
extern const ConstP_soccer_vision_T soccer_vision_ConstP;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void soccer_vision_initialize(void);
  extern void soccer_vision_step(void);
  extern void soccer_vision_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_soccer_vision_T *const soccer_vision_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/Draw Lines' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'soccer_vision'
//  '<S1>'   : 'soccer_vision/From File'
//  '<S2>'   : 'soccer_vision/Line And Edge Detection'
//  '<S3>'   : 'soccer_vision/From File/From File'
//  '<S4>'   : 'soccer_vision/From File/From Gazebo'
//  '<S5>'   : 'soccer_vision/From File/From Gazebo/Subscribe'
//  '<S6>'   : 'soccer_vision/From File/From Gazebo/Subscribe/Enabled Subsystem'
//  '<S7>'   : 'soccer_vision/Line And Edge Detection/Cover Edge'
//  '<S8>'   : 'soccer_vision/Line And Edge Detection/DrawLines'
//  '<S9>'   : 'soccer_vision/Line And Edge Detection/Filter'
//  '<S10>'  : 'soccer_vision/Line And Edge Detection/Line Detection'
//  '<S11>'  : 'soccer_vision/Line And Edge Detection/Plot Field Intersection1'
//  '<S12>'  : 'soccer_vision/Line And Edge Detection/Track Lines'
//  '<S13>'  : 'soccer_vision/Line And Edge Detection/Track Lines/Label Field Lines'
//  '<S14>'  : 'soccer_vision/Line And Edge Detection/Track Lines/Label Net Lines'

#endif                                 // RTW_HEADER_soccer_vision_h_

//
// File trailer for generated code.
//
// [EOF]
//
