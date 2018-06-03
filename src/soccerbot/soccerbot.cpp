//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccerbot.cpp
//
// Code generated for Simulink model 'soccerbot'.
//
// Model version                  : 1.19
// Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
// C/C++ source code generated on : Sun Jun  3 00:59:48 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "soccerbot.h"
#include "soccerbot_private.h"
#include "soccerbot_dt.h"
#define soccerbot_MessageQueueLen      (1)

// Block signals (auto storage)
B_soccerbot_T soccerbot_B;

// Block states (auto storage)
DW_soccerbot_T soccerbot_DW;

// Real-time model
RT_MODEL_soccerbot_T soccerbot_M_;
RT_MODEL_soccerbot_T *const soccerbot_M = &soccerbot_M_;

// Forward declaration for local functions
static void soccerbot_char(const uint8_T varargin_1_data[], const int32_T
  varargin_1_size[2], char_T y_data[], int32_T y_size[2]);
static boolean_T soccerbot_strcmp(const char_T a_data[], const int32_T a_size[2]);
static boolean_T soccerbot_any(const boolean_T x[2]);
static void socce_ReadImage_decompressImage(const uint8_T busstruct_Data[128],
  uint32_T busstruct_Data_SL_Info_CurrentL, uint8_T image_data[], int32_T
  image_size[3], uint8_T *errorCode);
static uint8_T soccerbot_ReadImage_update(robotics_slros_internal_blo_b_T *obj,
  const uint8_T busstruct_Format[128], uint32_T busstruct_Format_SL_Info_Curren,
  const uint8_T busstruct_Data[128], uint32_T busstruct_Data_SL_Info_CurrentL,
  uint32_T busstruct_Data_SL_Info_Received);
static void soccerbot_char(const uint8_T varargin_1_data[], const int32_T
  varargin_1_size[2], char_T y_data[], int32_T y_size[2])
{
  int32_T i;
  int32_T loop_ub;
  y_size[0] = 1;
  y_size[1] = varargin_1_size[1];
  loop_ub = varargin_1_size[1];
  for (i = 0; i <= (int32_T)(loop_ub - 1); i++) {
    y_data[i] = (char_T)(int8_T)varargin_1_data[i];
  }
}

static boolean_T soccerbot_strcmp(const char_T a_data[], const int32_T a_size[2])
{
  boolean_T b_bool;
  int32_T minnanb;
  char_T b[5];
  static const char_T tmp[5] = { 'r', 'g', 'b', '8', ';' };

  static const char_T tmp_0[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d',
    '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16',
    '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',
    '!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>',
    '?', '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\',
    ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '{', '|', '}', '~', '\x7f' };

  int32_T exitg1;
  for (minnanb = 0; minnanb < 5; minnanb++) {
    b[minnanb] = tmp[minnanb];
  }

  b_bool = false;
  if (a_size[1] < 5) {
    minnanb = a_size[1];
  } else {
    minnanb = 5;
  }

  if ((5 <= minnanb) || (a_size[1] == 5)) {
    minnanb = 0;
    do {
      exitg1 = 0;
      if ((int32_T)(minnanb + 1) <= 5) {
        if (tmp_0[(int32_T)((int32_T)(uint8_T)a_data[minnanb] & 127)] != tmp_0
            [(int32_T)b[minnanb]]) {
          exitg1 = 1;
        } else {
          minnanb++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static boolean_T soccerbot_any(const boolean_T x[2])
{
  boolean_T y;
  int32_T b_k;
  boolean_T exitg1;
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (x[b_k]) {
      y = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  return y;
}

static void socce_ReadImage_decompressImage(const uint8_T busstruct_Data[128],
  uint32_T busstruct_Data_SL_Info_CurrentL, uint8_T image_data[], int32_T
  image_size[3], uint8_T *errorCode)
{
  int32_T height;
  int32_T width;
  int32_T b;
  int32_T c;
  boolean_T height_0[2];
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  memset(&soccerbot_B.fullImage[0], 0, (uint32_T)(230400U * sizeof(uint8_T)));
  width = 320;
  height = 240;
  decompressImage<3>(busstruct_Data, busstruct_Data_SL_Info_CurrentL,
                     soccerbot_B.fullImage, &width, &height);
  if (1 > height) {
    b = 0;
  } else {
    b = height;
  }

  if (1 > width) {
    c = 0;
  } else {
    c = width;
  }

  image_size[0] = b;
  image_size[1] = c;
  image_size[2] = 3;
  for (i_1 = 0; i_1 < 3; i_1++) {
    for (i_0 = 0; i_0 <= (int32_T)(c - 1); i_0++) {
      for (i = 0; i <= (int32_T)(b - 1); i++) {
        image_data[(int32_T)((int32_T)(i + (int32_T)(b * i_0)) + (int32_T)
                             ((int32_T)(b * c) * i_1))] = soccerbot_B.fullImage
          [(int32_T)((int32_T)((int32_T)(240 * i_0) + i) + (int32_T)(76800 * i_1))];
      }
    }
  }

  *errorCode = 0U;
  height_0[0] = (height > 240);
  height_0[1] = (width > 320);
  if (soccerbot_any(height_0)) {
    *errorCode = 2U;
  }
}

static uint8_T soccerbot_ReadImage_update(robotics_slros_internal_blo_b_T *obj,
  const uint8_T busstruct_Format[128], uint32_T busstruct_Format_SL_Info_Curren,
  const uint8_T busstruct_Data[128], uint32_T busstruct_Data_SL_Info_CurrentL,
  uint32_T busstruct_Data_SL_Info_Received)
{
  uint8_T errorCode;
  int32_T c;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T loop_ub;
  int32_T tmp_size[2];
  if (busstruct_Data_SL_Info_CurrentL < busstruct_Data_SL_Info_Received) {
    errorCode = 3U;
  } else {
    if (1U > busstruct_Format_SL_Info_Curren) {
      c = 0;
    } else {
      c = (int32_T)busstruct_Format_SL_Info_Curren;
    }

    soccerbot_B.busstruct_Format_size[0] = 1;
    soccerbot_B.busstruct_Format_size[1] = c;
    for (i = 0; i <= (int32_T)(c - 1); i++) {
      soccerbot_B.busstruct_Format_data[i] = busstruct_Format[i];
    }

    soccerbot_char(soccerbot_B.busstruct_Format_data,
                   soccerbot_B.busstruct_Format_size, soccerbot_B.tmp_data,
                   tmp_size);
    if (!soccerbot_strcmp(soccerbot_B.tmp_data, tmp_size)) {
      errorCode = 1U;
    } else {
      socce_ReadImage_decompressImage(busstruct_Data,
        busstruct_Data_SL_Info_CurrentL, soccerbot_B.rawImage_data,
        soccerbot_B.rawImage_size, &errorCode);
      if ((int32_T)errorCode == 0) {
        memset(&obj->Image[0], 0, (uint32_T)(230400U * sizeof(uint8_T)));
        c = soccerbot_B.rawImage_size[1];
        for (i = 0; i < 3; i++) {
          for (i_1 = 0; i_1 <= (int32_T)(c - 1); i_1++) {
            loop_ub = soccerbot_B.rawImage_size[0];
            for (i_0 = 0; i_0 <= (int32_T)(loop_ub - 1); i_0++) {
              obj->Image[(int32_T)((int32_T)(i_0 + (int32_T)(240 * i_1)) +
                                   (int32_T)(76800 * i))] =
                soccerbot_B.rawImage_data[(int32_T)((int32_T)((int32_T)
                (soccerbot_B.rawImage_size[0] * i_1) + i_0) + (int32_T)((int32_T)
                (soccerbot_B.rawImage_size[0] * soccerbot_B.rawImage_size[1]) *
                i))];
            }
          }
        }

        i = soccerbot_B.rawImage_size[0];
        if (soccerbot_B.rawImage_size[0] < 0) {
          i = 0;
        }

        i_1 = soccerbot_B.rawImage_size[1];
        if (soccerbot_B.rawImage_size[1] < 0) {
          i_1 = 0;
        }

        obj->ImageSize[0] = (uint32_T)i;
        obj->ImageSize[1] = (uint32_T)i_1;
      }
    }
  }

  return errorCode;
}

// Model step function
void soccerbot_step(void)
{
  boolean_T b_varargout_1;

  // Reset subsysRan breadcrumbs
  srClearBC(soccerbot_DW.SelectImages_SubsysRanBC);

  // Reset subsysRan breadcrumbs
  srClearBC(soccerbot_DW.EnabledSubsystem_SubsysRanBC);

  // Gain: '<S2>/Gain'
  soccerbot_B.Gain = soccerbot_P.ctrlParam1 * 0.0;

  // Gain: '<S2>/Gain1'
  soccerbot_B.Gain1 = soccerbot_P.ctrlParam2 * 0.0;

  // Outputs for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S6>/SourceBlock'
  b_varargout_1 = Sub_soccerbot_97.getLatestMessage(&soccerbot_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<Root>/Select Images' incorporates:
  //   EnablePort: '<S5>/Enable'

  // Outputs for Enabled SubSystem: '<S6>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S8>/Enable'

  if (b_varargout_1) {
    srUpdateBC(soccerbot_DW.EnabledSubsystem_SubsysRanBC);

    // MATLABSystem: '<S5>/Read Image'
    soccerbot_ReadImage_update(&soccerbot_DW.obj,
      soccerbot_B.b_varargout_2.Format,
      soccerbot_B.b_varargout_2.Format_SL_Info.CurrentLength,
      soccerbot_B.b_varargout_2.Data,
      soccerbot_B.b_varargout_2.Data_SL_Info.CurrentLength,
      soccerbot_B.b_varargout_2.Data_SL_Info.ReceivedLength);
    srUpdateBC(soccerbot_DW.SelectImages_SubsysRanBC);
  }

  // End of Start for MATLABSystem: '<S6>/SourceBlock'
  // End of Outputs for SubSystem: '<S6>/Enabled Subsystem'
  // End of Outputs for SubSystem: '<Root>/Select Images'
  // End of Outputs for SubSystem: '<Root>/Subscribe'

  // External mode
  rtExtModeUploadCheckTrigger(1);

  {                                    // Sample time: [0.2s, 0.0s]
    rtExtModeUpload(0, soccerbot_M->Timing.taskTime0);
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.2s, 0.0s]
    if ((rtmGetTFinal(soccerbot_M)!=-1) &&
        !((rtmGetTFinal(soccerbot_M)-soccerbot_M->Timing.taskTime0) >
          soccerbot_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(soccerbot_M, "Simulation finished");
    }

    if (rtmGetStopRequested(soccerbot_M)) {
      rtmSetErrorStatus(soccerbot_M, "Simulation finished");
    }
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  soccerbot_M->Timing.taskTime0 =
    (++soccerbot_M->Timing.clockTick0) * soccerbot_M->Timing.stepSize0;
}

// Model initialize function
void soccerbot_initialize(void)
{
  // Registration code

  // initialize real-time model
  (void) memset((void *)soccerbot_M, 0,
                sizeof(RT_MODEL_soccerbot_T));
  rtmSetTFinal(soccerbot_M, -1);
  soccerbot_M->Timing.stepSize0 = 0.2;

  // External mode info
  soccerbot_M->Sizes.checksums[0] = (3498812502U);
  soccerbot_M->Sizes.checksums[1] = (1874584249U);
  soccerbot_M->Sizes.checksums[2] = (2566056049U);
  soccerbot_M->Sizes.checksums[3] = (3380035101U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[8];
    soccerbot_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = (sysRanDType *)&soccerbot_DW.SelectImages_SubsysRanBC;
    systemRan[4] = (sysRanDType *)&soccerbot_DW.SelectImages_SubsysRanBC;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = (sysRanDType *)&soccerbot_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[7] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(soccerbot_M->extModeInfo,
      &soccerbot_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(soccerbot_M->extModeInfo, soccerbot_M->Sizes.checksums);
    rteiSetTPtr(soccerbot_M->extModeInfo, rtmGetTPtr(soccerbot_M));
  }

  // block I/O
  (void) memset(((void *) &soccerbot_B), 0,
                sizeof(B_soccerbot_T));

  // states (dwork)
  (void) memset((void *)&soccerbot_DW, 0,
                sizeof(DW_soccerbot_T));

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    soccerbot_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 22;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    static const char_T tmp[39] = { '/', 's', 'o', 'c', 'c', 'e', 'r', 'b', 'o',
      't', '/', 'c', 'a', 'm', 'e', 'r', 'a', '1', '/', 'i', 'm', 'a', 'g', 'e',
      '_', 'r', 'a', 'w', '/', 'c', 'o', 'm', 'p', 'r', 'e', 's', 's', 'e', 'd'
    };

    char_T tmp_0[40];
    int32_T i;

    // Start for Atomic SubSystem: '<Root>/Subscribe'
    // Start for MATLABSystem: '<S6>/SourceBlock'
    soccerbot_DW.obj_e.isInitialized = 0;
    soccerbot_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 39; i++) {
      tmp_0[i] = tmp[i];
    }

    tmp_0[39] = '\x00';
    Sub_soccerbot_97.createSubscriber(tmp_0, soccerbot_MessageQueueLen);

    // End of Start for MATLABSystem: '<S6>/SourceBlock'
    // End of Start for SubSystem: '<Root>/Subscribe'

    // Start for Enabled SubSystem: '<Root>/Select Images'
    // Start for MATLABSystem: '<S5>/Read Image'
    soccerbot_DW.obj.isInitialized = 0;
    soccerbot_DW.obj.isInitialized = 1;

    // End of Start for SubSystem: '<Root>/Select Images'

    // Start for MATLABSystem: '<Root>/MATLAB System'
    soccerbot_DW.obj_d.isInitialized = 0;
    soccerbot_DW.obj_d.isInitialized = 1;

    // SystemInitialize for Enabled SubSystem: '<Root>/Select Images'
    // InitializeConditions for MATLABSystem: '<S5>/Read Image'
    memset(&soccerbot_DW.obj.Image[0], 0, (uint32_T)(230400U * sizeof(uint8_T)));
    soccerbot_DW.obj.ImageSize[0] = 240U;
    soccerbot_DW.obj.ImageSize[1] = 320U;

    // End of SystemInitialize for SubSystem: '<Root>/Select Images'
  }
}

// Model terminate function
void soccerbot_terminate(void)
{
  // Terminate for Atomic SubSystem: '<Root>/Subscribe'
  // Start for MATLABSystem: '<S6>/SourceBlock'
  if (soccerbot_DW.obj_e.isInitialized == 1) {
    soccerbot_DW.obj_e.isInitialized = 2;
  }

  // End of Start for MATLABSystem: '<S6>/SourceBlock'
  // End of Terminate for SubSystem: '<Root>/Subscribe'

  // Terminate for Enabled SubSystem: '<Root>/Select Images'
  // Terminate for MATLABSystem: '<S5>/Read Image'
  if (soccerbot_DW.obj.isInitialized == 1) {
    soccerbot_DW.obj.isInitialized = 2;
  }

  // End of Terminate for MATLABSystem: '<S5>/Read Image'
  // End of Terminate for SubSystem: '<Root>/Select Images'

  // Terminate for MATLABSystem: '<Root>/MATLAB System'
  if (soccerbot_DW.obj_d.isInitialized == 1) {
    soccerbot_DW.obj_d.isInitialized = 2;
  }

  // End of Terminate for MATLABSystem: '<Root>/MATLAB System'
}

//
// File trailer for generated code.
//
// [EOF]
//
