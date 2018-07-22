//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_vision_data.cpp
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
#include "soccer_vision.h"
#include "soccer_vision_private.h"

// Block parameters (default storage)
P_soccer_vision_T soccer_vision_P = {
  // Variable: clusterProximityThreshold
  //  Referenced by: '<S2>/Filter'

  0.05,

  // Variable: lineTrackingThreshold
  //  Referenced by: '<S12>/Label Field Lines'

  0.2,

  // Variable: netAngleThreshold
  //  Referenced by: '<S2>/Filter'

  0.31415926535897931,

  // Variable: scalerho
  //  Referenced by:
  //    '<S2>/Filter'
  //    '<S12>/Label Field Lines'

  0.002,

  // Mask Parameter: FindLocalMaxima_threshold
  //  Referenced by: '<S10>/Find Local Maxima'

  27.0,

  // Mask Parameter: EdgeDetection_thresholdTuning
  //  Referenced by: '<S10>/Edge Detection'

  1536,

<<<<<<< HEAD
=======
  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S6>/Out1'

  {
    0U,                                // Height
    0U,                                // Width

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Encoding

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Encoding_SL_Info
    0U,                                // IsBigendian
    0U,                                // Step

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Data

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Data_SL_Info

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S5>/Constant'

  {
    0U,                                // Height
    0U,                                // Width

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Encoding

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Encoding_SL_Info
    0U,                                // IsBigendian
    0U,                                // Step

    {
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }
    ,                                  // Data

    {
      0U,                              // CurrentLength
      0U                               // ReceivedLength
    },                                 // Data_SL_Info

    {
      0U,                              // Seq

      {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
        0U, 0U }
      ,                                // FrameId

      {
        0U,                            // CurrentLength
        0U                             // ReceivedLength
      },                               // FrameId_SL_Info

      {
        0.0,                           // Sec
        0.0                            // Nsec
      }                                // Stamp
    }                                  // Header
  },

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  // Expression: 700
  //  Referenced by: '<S2>/Constant6'

  700.0,

  // Expression: 4
  //  Referenced by: '<S10>/Gain'

  4.0,

  // Expression: 500
  //  Referenced by: '<S2>/Constant2'

  500.0,

  // Expression: 0
  //  Referenced by: '<S2>/Constant3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S2>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<S2>/Constant1'

  0.0,

  // Expression: [0,0;0,0;0,0;0,0]
  //  Referenced by: '<S12>/Memory'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: [0,0;0,0;0,0;0,0;0,0]
  //  Referenced by: '<S12>/Memory1'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Computed Parameter: CoverAboveHorizon_RTP_OPACITY
  //  Referenced by: '<S2>/Cover Above Horizon'

  13107
};

// Constant parameters (default storage)
const ConstP_soccer_vision_T soccer_vision_ConstP = {
  // Computed Parameter: HoughTransform_SINE_TABLE_RTP
  //  Referenced by: '<S10>/Hough Transform'

  { -1.0, -0.99984769515639127, -0.99939082701909576, -0.99862953475457383,
    -0.9975640502598242, -0.99619469809174555, -0.99452189536827329,
    -0.992546151641322, -0.99026806874157036, -0.98768834059513777,
    -0.984807753012208, -0.981627183447664, -0.97814760073380558,
    -0.97437006478523525, -0.97029572627599647, -0.96592582628906831,
    -0.96126169593831889, -0.95630475596303544, -0.95105651629515353,
    -0.94551857559931674, -0.93969262078590832, -0.93358042649720174,
    -0.92718385456678742, -0.92050485345244037, -0.91354545764260087,
    -0.90630778703664994, -0.898794046299167, -0.89100652418836779,
    -0.88294759285892688, -0.87461970713939574, -0.8660254037844386,
    -0.85716730070211233, -0.848048096156426, -0.838670567945424,
    -0.82903757255504162, -0.81915204428899169, -0.80901699437494734,
    -0.79863551004729283, -0.7880107536067219, -0.77714596145697079,
    -0.766044443118978, -0.7547095802227719, -0.74314482547739424,
    -0.73135370161917046, -0.71933980033865108, -0.70710678118654746,
    -0.69465837045899725, -0.68199836006249848, -0.66913060635885824,
    -0.65605902899050728, -0.64278760968653925, -0.62932039104983739,
    -0.61566147532565829, -0.60181502315204827, -0.58778525229247314,
    -0.573576436351046, -0.5591929034707469, -0.54463903501502708,
    -0.5299192642332049, -0.51503807491005416, -0.49999999999999994,
    -0.48480962024633706, -0.46947156278589081, -0.45399049973954675,
    -0.4383711467890774, -0.42261826174069944, -0.40673664307580021,
    -0.39073112848927377, -0.374606593415912, -0.35836794954530027,
    -0.34202014332566871, -0.3255681544571567, -0.3090169943749474,
    -0.29237170472273677, -0.27563735581699916, -0.25881904510252074,
    -0.24192189559966773, -0.224951054343865, -0.20791169081775934,
    -0.1908089953765448, -0.17364817766693033, -0.15643446504023087,
    -0.13917310096006544, -0.12186934340514748, -0.10452846326765347,
    -0.087155742747658166, -0.0697564737441253, -0.052335956242943835,
    -0.034899496702500969, -0.017452406437283512, 0.0 },

  // Computed Parameter: HoughTransform_FIRSTRHO_RTP
  //  Referenced by: '<S10>/Hough Transform'

  -199.0,

  // Computed Parameter: EdgeDetection_VRO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  { -1, -1, -1, 1, 1, 1 },

  // Computed Parameter: EdgeDetection_VCO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  { -1, 0, 1, -1, 0, 1 },

  // Computed Parameter: EdgeDetection_HRO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  { -1, -1, 0, 0, 1, 1 },

  // Computed Parameter: EdgeDetection_HCO_RTP
  //  Referenced by: '<S10>/Edge Detection'

  { -1, 1, -1, 1, -1, 1 },

  // Computed Parameter: EdgeDetection_VC_RTP
  //  Referenced by: '<S10>/Edge Detection'

  { 357913941, 357913941, 357913941, -357913941, -357913941, -357913941 },

  // Computed Parameter: EdgeDetection_HC_RTP
  //  Referenced by: '<S10>/Edge Detection'

  { 357913941, -357913941, 357913941, -357913941, 357913941, -357913941 }
};

//
// File trailer for generated code.
//
// [EOF]
//
