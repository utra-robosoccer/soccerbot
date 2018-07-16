//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_vision.cpp
//
// Code generated for Simulink model 'soccer_vision'.
//
// Model version                  : 1.770
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sun Jul 15 23:50:31 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "soccer_vision.h"
#include "soccer_vision_private.h"
#include "soccer_vision_dt.h"

// Named constants for MATLAB Function: '<S12>/Label Net Lines'
#define soccer_vision_scalerho         (0.002)
#define soccer_vision_MessageQueueLen  (1)

// Block signals (default storage)
B_soccer_vision_T soccer_vision_B;

// Block states (default storage)
DW_soccer_vision_T soccer_vision_DW;

// Real-time model
RT_MODEL_soccer_vision_T soccer_vision_M_;
RT_MODEL_soccer_vision_T *const soccer_vision_M = &soccer_vision_M_;

// Forward declaration for local functions
static void soccer_vision_char(const uint8_T varargin_1_data[], const int32_T
  varargin_1_size[2], char_T y_data[], int32_T y_size[2]);
static boolean_T soccer_vision_strcmp(const char_T a_data[], const int32_T
  a_size[2]);
static void soccer_vision_typecast(const uint8_T x_data[], const int32_T *x_size,
  uint8_T y_data[], int32_T *y_size);
static void soccer_vision_permute(const uint8_T a_data[], const int32_T a_size[3],
  uint8_T b_data[], int32_T b_size[3]);
static void soccer_vision_ImageReader_readImage(const uint8_T data_data[], const
  int32_T *data_size, uint32_T width, uint32_T height, uint8_T img_data[],
  int32_T img_size[3]);
static void soccer_vision_ReadImage_decodeImage(const
  robotics_slros_internal_block_ReadImage_soccer_vision_T *obj, const uint8_T
  data_data[], const int32_T *data_size, uint32_T width, uint32_T height,
  uint8_T image_data[], int32_T image_size[3], uint8_T *errorCode);
static uint8_T soccer_vision_ReadImage_update
  (robotics_slros_internal_block_ReadImage_soccer_vision_T *obj, uint32_T
   busstruct_Height, uint32_T busstruct_Width, const uint8_T busstruct_Encoding
   [128], uint32_T busstruct_Encoding_SL_Info_CurrentLength, const uint8_T
   busstruct_Data[128], uint32_T busstruct_Data_SL_Info_CurrentLength, uint32_T
   busstruct_Data_SL_Info_ReceivedLength);
static real_T soccer_vision_rand(void);
static void soccer_vision_distfun(real_T D_data[], int32_T D_size[2], const
  real_T X[40], const real_T C_data[], const int32_T C_size[2], int32_T crows);
static boolean_T soccer_vision_isfinite(real_T x);
static int32_T soccer_vision_bsearch(const real_T x[21], real_T xi);
static void soccer_vision_simpleRandperm(int32_T k, int32_T idx[20]);
static void soccer_vision_distfun_k(real_T D_data[], int32_T D_size[2], const
  real_T X[40], const real_T C_data[], const int32_T C_size[2], const int32_T
  crows_data[], int32_T ncrows);
static void soccer_vision_mindim2(const real_T D_data[], const int32_T D_size[2],
  real_T d[20], int32_T idx[20]);
static void soccer_vision_gcentroids(real_T C_data[], int32_T C_size[2], int32_T
  counts_data[], const real_T X[40], const int32_T idx[20], const int32_T
  clusters_data[], int32_T nclusters);
static int32_T soccer_vision_findchanged(int32_T changed_data[], const int32_T
  idx[20], const int32_T previdx[20], const int32_T moved[20], int32_T nmoved);
static void soccer_vision_gcentroids_f(real_T C_data[], int32_T C_size[2],
  int32_T counts_data[], const real_T X[40], const int32_T idx[20], int32_T
  clusters);
static void soccer_vision_batchUpdate(const real_T X[40], int32_T k, int32_T
  idx[20], real_T C_data[], int32_T C_size[2], real_T D_data[], int32_T D_size[2],
  int32_T counts_data[], boolean_T *converged, int32_T *iter);
static void soccer_vision_kmeans(const real_T X[40], real_T kin, real_T idxbest
  [20], real_T Cbest_data[], int32_T Cbest_size[2], real_T varargout_1_data[],
  int32_T *varargout_1_size);
static real_T soccer_vision_norm(const real_T x[2]);
static void soccer_vision_merge(int32_T idx[5], real_T x[5], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork[5], real_T xwork[5]);
static void soccer_vision_sort(real_T x[5], int32_T idx[5]);
static Point2f_soccer_vision_T *soccer_vision_Line2f_screenIntersection(const
  Line2f_soccer_vision_T *obj1, const Line2f_soccer_vision_T *obj2,
  Point2f_soccer_vision_T *iobj_0);
static void soccer_vision_SystemProp_matlabCodegenSetAnyProp
  (robotics_slros_internal_block_Subscriber_soccer_vision_T *obj, boolean_T
   value);
static void soccer_vision_matlabCodegenHandle_matlabCodegenDestructor
  (robotics_slros_internal_block_Subscriber_soccer_vision_T *obj);
static void rate_scheduler(void);

//
//   This function updates active task flag for each subrate.
// The function is called at model base rate, hence the
// generated code self-manages all its subrates.
//
static void rate_scheduler(void)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (soccer_vision_M->Timing.TaskCounters.TID[1])++;
  if ((soccer_vision_M->Timing.TaskCounters.TID[1]) > 9) {// Sample time: [0.1s, 0.0s] 
    soccer_vision_M->Timing.TaskCounters.TID[1] = 0;
  }
}

static void soccer_vision_char(const uint8_T varargin_1_data[], const int32_T
  varargin_1_size[2], char_T y_data[], int32_T y_size[2])
{
  int32_T loop_ub;
  int32_T i;
  y_size[0] = 1;
  y_size[1] = varargin_1_size[1];
  loop_ub = varargin_1_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    y_data[i] = (int8_T)varargin_1_data[i];
  }
}

static boolean_T soccer_vision_strcmp(const char_T a_data[], const int32_T
  a_size[2])
{
  boolean_T b_bool;
  int32_T kstr;
  char_T b[4];
  static const char_T tmp[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
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
  b[0] = 'r';
  b[1] = 'g';
  b[2] = 'b';
  b[3] = '8';
  b_bool = false;
  if (a_size[1] == 4) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr + 1 < 5) {
        if (tmp[(uint8_T)a_data[kstr] & 127] != tmp[(int32_T)b[kstr]]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static void soccer_vision_typecast(const uint8_T x_data[], const int32_T *x_size,
  uint8_T y_data[], int32_T *y_size)
{
  *y_size = *x_size;
  memcpy((void *)&y_data[0], (void *)&x_data[0], (uint32_T)((size_t)*x_size *
          sizeof(uint8_T)));
}

static void soccer_vision_permute(const uint8_T a_data[], const int32_T a_size[3],
  uint8_T b_data[], int32_T b_size[3])
{
  boolean_T b;
  int8_T perm[3];
  int32_T plast;
  int32_T k;
  boolean_T exitg1;
  perm[0] = 2;
  perm[1] = 1;
  perm[2] = 3;
  b = true;
  if (!((a_size[0] == 0) || (a_size[1] == 0))) {
    plast = 0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 4)) {
      if (a_size[perm[k] - 1] != 1) {
        if (plast > perm[k]) {
          b = false;
          exitg1 = true;
        } else {
          plast = perm[k];
          k++;
        }
      } else {
        k++;
      }
    }
  }

  if (b) {
    b_size[0] = a_size[1];
    b_size[1] = a_size[0];
    b_size[2] = 3;
    k = a_size[1] * a_size[0] * 3 - 1;
    if (0 <= k) {
      memcpy(&b_data[0], &a_data[0], (k + 1) * sizeof(uint8_T));
    }
  } else {
    b_size[0] = a_size[1];
    b_size[1] = a_size[0];
    b_size[2] = 3;
    for (k = 1; k <= a_size[1]; k++) {
      for (plast = 1; plast <= a_size[0]; plast++) {
        b_data[(k + b_size[0] * (plast - 1)) - 1] = a_data[((k - 1) * a_size[0]
          + plast) - 1];
      }
    }

    for (k = 1; k <= a_size[1]; k++) {
      for (plast = 1; plast <= a_size[0]; plast++) {
        b_data[((k + b_size[0] * (plast - 1)) + b_size[0] * b_size[1]) - 1] =
          a_data[(((k - 1) * a_size[0] + plast) + a_size[0] * a_size[1]) - 1];
      }
    }

    for (k = 1; k <= a_size[1]; k++) {
      for (plast = 1; plast <= a_size[0]; plast++) {
        b_data[((k + b_size[0] * (plast - 1)) + ((b_size[0] * b_size[1]) << 1))
          - 1] = a_data[(((k - 1) * a_size[0] + plast) + ((a_size[0] * a_size[1])
          << 1)) - 1];
      }
    }
  }
}

static void soccer_vision_ImageReader_readImage(const uint8_T data_data[], const
  int32_T *data_size, uint32_T width, uint32_T height, uint8_T img_data[],
  int32_T img_size[3])
{
  uint32_T y;
  int32_T n;
  int32_T k;
  int32_T loop_ub;
  int32_T i;
  uint32_T qY;
  if (*data_size == 0) {
    img_size[0] = 0;
    img_size[1] = 1;
    img_size[2] = 1;
  } else {
    soccer_vision_typecast(data_data, data_size, soccer_vision_B.data_data_k, &n);
    soccer_vision_B.u0 = (uint64_T)width * height;
    if (soccer_vision_B.u0 > 4294967295UL) {
      soccer_vision_B.u0 = 4294967295UL;
    }

    soccer_vision_B.u0 = (uint32_T)soccer_vision_B.u0 * 3UL;
    if (soccer_vision_B.u0 > 4294967295UL) {
      soccer_vision_B.u0 = 4294967295UL;
    }

    y = (uint32_T)soccer_vision_B.u0;
    if (y < 1U) {
      n = 0;
    } else {
      n = (int32_T)((y - 1U) / 3U) + 1;
    }

    for (k = 0; k < n; k++) {
      soccer_vision_B.y_data[k] = k * 3U + 1U;
    }

    if (0 <= n - 1) {
      memcpy(&soccer_vision_B.indexBase_data[0], &soccer_vision_B.y_data[0], n *
             sizeof(uint32_T));
    }

    loop_ub = n * 3 - 1;
    if (0 <= loop_ub) {
      memset(&soccer_vision_B.b_index_data[0], 0, (loop_ub + 1) * sizeof
             (uint32_T));
    }

    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      k = 1 + loop_ub;
      for (i = 0; i < n; i++) {
        soccer_vision_B.d0 = (real_T)soccer_vision_B.indexBase_data[i] + (real_T)
          k;
        if (soccer_vision_B.d0 < 4.294967296E+9) {
          y = (uint32_T)soccer_vision_B.d0;
        } else {
          y = MAX_uint32_T;
        }

        qY = y - /*MW:OvSatOk*/ 1U;
        if (qY > y) {
          qY = 0U;
        }

        soccer_vision_B.b_index_data[i + n * loop_ub] = qY;
      }
    }

    soccer_vision_B.data_size[0] = (int32_T)width;
    soccer_vision_B.data_size[1] = (int32_T)height;
    soccer_vision_B.data_size[2] = 3;
    loop_ub = (int32_T)width * (int32_T)height * 3;
    for (k = 0; k < loop_ub; k++) {
      soccer_vision_B.data_data[k] = soccer_vision_B.data_data_k[(int32_T)
        soccer_vision_B.b_index_data[k] - 1];
    }

    soccer_vision_permute(soccer_vision_B.data_data, soccer_vision_B.data_size,
                          soccer_vision_B.tmp_data, soccer_vision_B.tmp_size);
    img_size[0] = soccer_vision_B.tmp_size[0];
    img_size[1] = soccer_vision_B.tmp_size[1];
    img_size[2] = 3;
    loop_ub = soccer_vision_B.tmp_size[0] * soccer_vision_B.tmp_size[1] *
      soccer_vision_B.tmp_size[2];
    if (0 <= loop_ub - 1) {
      memcpy(&img_data[0], &soccer_vision_B.tmp_data[0], loop_ub * sizeof
             (uint8_T));
    }
  }
}

static void soccer_vision_ReadImage_decodeImage(const
  robotics_slros_internal_block_ReadImage_soccer_vision_T *obj, const uint8_T
  data_data[], const int32_T *data_size, uint32_T width, uint32_T height,
  uint8_T image_data[], int32_T image_size[3], uint8_T *errorCode)
{
  if ((height > 480U) || (width > 640U)) {
    *errorCode = 2U;
    image_size[0] = 480;
    image_size[1] = 640;
    image_size[2] = 3;
    memcpy(&image_data[0], &obj->Image[0], 921600U * sizeof(uint8_T));
  } else {
    soccer_vision_ImageReader_readImage(data_data, data_size, width, height,
      image_data, image_size);
    *errorCode = 0U;
  }
}

static uint8_T soccer_vision_ReadImage_update
  (robotics_slros_internal_block_ReadImage_soccer_vision_T *obj, uint32_T
   busstruct_Height, uint32_T busstruct_Width, const uint8_T busstruct_Encoding
   [128], uint32_T busstruct_Encoding_SL_Info_CurrentLength, const uint8_T
   busstruct_Data[128], uint32_T busstruct_Data_SL_Info_CurrentLength, uint32_T
   busstruct_Data_SL_Info_ReceivedLength)
{
  uint8_T errorCode;
  if (busstruct_Data_SL_Info_CurrentLength <
      busstruct_Data_SL_Info_ReceivedLength) {
    errorCode = 3U;
  } else {
    if (1U > busstruct_Encoding_SL_Info_CurrentLength) {
      soccer_vision_B.c = 0;
    } else {
      soccer_vision_B.c = (int32_T)busstruct_Encoding_SL_Info_CurrentLength;
    }

    soccer_vision_B.busstruct_Encoding_size[0] = 1;
    soccer_vision_B.busstruct_Encoding_size[1] = soccer_vision_B.c;
    if (0 <= soccer_vision_B.c - 1) {
      memcpy(&soccer_vision_B.busstruct_Encoding_data[0], &busstruct_Encoding[0],
             soccer_vision_B.c * sizeof(uint8_T));
    }

    soccer_vision_char(soccer_vision_B.busstruct_Encoding_data,
                       soccer_vision_B.busstruct_Encoding_size,
                       soccer_vision_B.tmp_data_c, soccer_vision_B.tmp_size_b);
    if (!soccer_vision_strcmp(soccer_vision_B.tmp_data_c,
         soccer_vision_B.tmp_size_b)) {
      errorCode = 1U;
    } else {
      if (1U > busstruct_Data_SL_Info_CurrentLength) {
        soccer_vision_B.c = 0;
      } else {
        soccer_vision_B.c = (int32_T)busstruct_Data_SL_Info_CurrentLength;
      }

      soccer_vision_B.busstruct_Data_size = soccer_vision_B.c;
      if (0 <= soccer_vision_B.c - 1) {
        memcpy(&soccer_vision_B.busstruct_Encoding_data[0], &busstruct_Data[0],
               soccer_vision_B.c * sizeof(uint8_T));
      }

      soccer_vision_ReadImage_decodeImage(obj,
        soccer_vision_B.busstruct_Encoding_data,
        &soccer_vision_B.busstruct_Data_size, busstruct_Width, busstruct_Height,
        soccer_vision_B.rawImage_data, soccer_vision_B.rawImage_size, &errorCode);
      if (errorCode == 0) {
        memset(&obj->Image[0], 0, 921600U * sizeof(uint8_T));
        if (1 > soccer_vision_B.rawImage_size[0]) {
          soccer_vision_B.c = 0;
        } else {
          soccer_vision_B.c = soccer_vision_B.rawImage_size[0];
        }

        if (1 > soccer_vision_B.rawImage_size[1]) {
          soccer_vision_B.h = 0;
        } else {
          soccer_vision_B.h = soccer_vision_B.rawImage_size[1];
        }

        for (soccer_vision_B.i1 = 0; soccer_vision_B.i1 < soccer_vision_B.c;
             soccer_vision_B.i1++) {
          soccer_vision_B.i_data[soccer_vision_B.i1] = soccer_vision_B.i1;
        }

        for (soccer_vision_B.i1 = 0; soccer_vision_B.i1 < soccer_vision_B.h;
             soccer_vision_B.i1++) {
          soccer_vision_B.j_data[soccer_vision_B.i1] = soccer_vision_B.i1;
        }

        for (soccer_vision_B.i1 = 0; soccer_vision_B.i1 < 3; soccer_vision_B.i1
             ++) {
          for (soccer_vision_B.i3 = 0; soccer_vision_B.i3 < soccer_vision_B.h;
               soccer_vision_B.i3++) {
            for (soccer_vision_B.i2 = 0; soccer_vision_B.i2 < soccer_vision_B.c;
                 soccer_vision_B.i2++) {
              obj->Image[(soccer_vision_B.i_data[soccer_vision_B.i2] + 480 *
                          soccer_vision_B.j_data[soccer_vision_B.i3]) + 307200 *
                soccer_vision_B.i1] = soccer_vision_B.rawImage_data
                [(soccer_vision_B.c * soccer_vision_B.i3 + soccer_vision_B.i2) +
                soccer_vision_B.c * soccer_vision_B.h * soccer_vision_B.i1];
            }
          }
        }

        soccer_vision_B.i1 = soccer_vision_B.rawImage_size[0];
        if (soccer_vision_B.rawImage_size[0] < 0) {
          soccer_vision_B.i1 = 0;
        }

        soccer_vision_B.i3 = soccer_vision_B.rawImage_size[1];
        if (soccer_vision_B.rawImage_size[1] < 0) {
          soccer_vision_B.i3 = 0;
        }

        obj->ImageSize[0] = (uint32_T)soccer_vision_B.i1;
        obj->ImageSize[1] = (uint32_T)soccer_vision_B.i3;
        errorCode = 0U;
      }
    }
  }

  return errorCode;
}

// Function for MATLAB Function: '<S2>/Filter'
static real_T soccer_vision_rand(void)
{
  real_T r;
  uint32_T mti;
  uint32_T y;
  int32_T j;
  int32_T kk;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    for (j = 0; j < 2; j++) {
      mti = soccer_vision_DW.state[624] + 1U;
      if (mti >= 625U) {
        for (kk = 0; kk < 227; kk++) {
          y = (soccer_vision_DW.state[kk + 1] & 2147483647U) |
            (soccer_vision_DW.state[kk] & 2147483648U);
          if ((int32_T)(y & 1U) == 0) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          soccer_vision_DW.state[kk] = soccer_vision_DW.state[kk + 397] ^ y;
        }

        for (kk = 0; kk < 396; kk++) {
          y = (soccer_vision_DW.state[kk + 227] & 2147483648U) |
            (soccer_vision_DW.state[kk + 228] & 2147483647U);
          if ((int32_T)(y & 1U) == 0) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          soccer_vision_DW.state[kk + 227] = soccer_vision_DW.state[kk] ^ y;
        }

        y = (soccer_vision_DW.state[623] & 2147483648U) |
          (soccer_vision_DW.state[0] & 2147483647U);
        if ((int32_T)(y & 1U) == 0) {
          y >>= 1U;
        } else {
          y = y >> 1U ^ 2567483615U;
        }

        soccer_vision_DW.state[623] = soccer_vision_DW.state[396] ^ y;
        mti = 1U;
      }

      y = soccer_vision_DW.state[(int32_T)mti - 1];
      soccer_vision_DW.state[624] = mti;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      soccer_vision_B.u_c[j] = y >> 18U ^ y;
    }

    r = ((real_T)(soccer_vision_B.u_c[0] >> 5U) * 6.7108864E+7 + (real_T)
         (soccer_vision_B.u_c[1] >> 6U)) * 1.1102230246251565E-16;
  } while (r == 0.0);

  return r;
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_distfun(real_T D_data[], int32_T D_size[2], const
  real_T X[40], const real_T C_data[], const int32_T C_size[2], int32_T crows)
{
  int32_T r;
  real_T b_a;
  for (r = 0; r < 20; r++) {
    b_a = X[r] - C_data[crows - 1];
    D_data[r + D_size[0] * (crows - 1)] = b_a * b_a;
  }

  for (r = 0; r < 20; r++) {
    b_a = X[20 + r] - C_data[(crows + C_size[0]) - 1];
    D_data[r + D_size[0] * (crows - 1)] += b_a * b_a;
  }
}

// Function for MATLAB Function: '<S2>/Filter'
static boolean_T soccer_vision_isfinite(real_T x)
{
  return (!rtIsInf(x)) && (!rtIsNaN(x));
}

// Function for MATLAB Function: '<S2>/Filter'
static int32_T soccer_vision_bsearch(const real_T x[21], real_T xi)
{
  int32_T n;
  int32_T low_ip1;
  int32_T high_i;
  int32_T mid_i;
  n = 1;
  low_ip1 = 2;
  high_i = 21;
  while (high_i > low_ip1) {
    mid_i = (n + high_i) >> 1;
    if (xi >= x[mid_i - 1]) {
      n = mid_i;
      low_ip1 = mid_i + 1;
    } else {
      high_i = mid_i;
    }
  }

  return n;
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_simpleRandperm(int32_T k, int32_T idx[20])
{
  int32_T t;
  int32_T numer;
  int32_T m;
  t = 0;
  memset(&idx[0], 0, 20U * sizeof(int32_T));
  for (m = 0; m < k; m++) {
    numer = k - m;
    soccer_vision_B.denom = 20 - t;
    soccer_vision_B.pt = (real_T)numer / (real_T)(20 - t);
    soccer_vision_B.u = soccer_vision_rand();
    while (soccer_vision_B.u > soccer_vision_B.pt) {
      t++;
      soccer_vision_B.denom--;
      soccer_vision_B.pt += (1.0 - soccer_vision_B.pt) * ((real_T)numer /
        soccer_vision_B.denom);
    }

    t++;
    soccer_vision_B.denom = (real_T)(m + 1) * soccer_vision_rand();
    soccer_vision_B.denom = floor(soccer_vision_B.denom);
    idx[m] = idx[(int32_T)soccer_vision_B.denom];
    idx[(int32_T)soccer_vision_B.denom] = t;
  }
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_distfun_k(real_T D_data[], int32_T D_size[2], const
  real_T X[40], const real_T C_data[], const int32_T C_size[2], const int32_T
  crows_data[], int32_T ncrows)
{
  int32_T cr;
  int32_T i;
  int32_T r;
  real_T b_a;
  for (i = 1; i <= ncrows; i++) {
    cr = crows_data[i - 1] - 1;
    for (r = 0; r < 20; r++) {
      b_a = X[r] - C_data[cr];
      D_data[r + D_size[0] * cr] = b_a * b_a;
    }

    for (r = 0; r < 20; r++) {
      b_a = X[20 + r] - C_data[cr + C_size[0]];
      D_data[r + D_size[0] * cr] += b_a * b_a;
    }
  }
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_mindim2(const real_T D_data[], const int32_T D_size[2],
  real_T d[20], int32_T idx[20])
{
  int32_T j;
  int32_T i;
  real_T d_0;
  for (i = 0; i < 20; i++) {
    d[i] = (rtInf);
    idx[i] = 1;
  }

  for (j = 1; j <= D_size[1]; j++) {
    for (i = 0; i < 20; i++) {
      d_0 = d[i];
      if (D_data[(j - 1) * D_size[0] + i] < d[i]) {
        idx[i] = j;
        d_0 = D_data[(j - 1) * D_size[0] + i];
      }

      d[i] = d_0;
    }
  }
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_gcentroids(real_T C_data[], int32_T C_size[2], int32_T
  counts_data[], const real_T X[40], const int32_T idx[20], const int32_T
  clusters_data[], int32_T nclusters)
{
  int32_T clic;
  int32_T cc;
  int32_T ic;
  int32_T i;
  int32_T tmp;
  for (ic = 0; ic < nclusters; ic++) {
    counts_data[clusters_data[ic] - 1] = 0;
    C_data[clusters_data[ic] - 1] = (rtNaN);
    C_data[(clusters_data[ic] + C_size[0]) - 1] = (rtNaN);
  }

  for (ic = 0; ic < nclusters; ic++) {
    clic = clusters_data[ic] - 1;
    cc = 0;
    C_data[clic] = 0.0;
    tmp = clic + C_size[0];
    C_data[tmp] = 0.0;
    for (i = 0; i < 20; i++) {
      if (clic + 1 == idx[i]) {
        cc++;
        C_data[clic] += X[i];
        C_data[tmp] += X[i + 20];
      }
    }

    counts_data[clusters_data[ic] - 1] = cc;
    C_data[clic] /= (real_T)cc;
    C_data[tmp] /= (real_T)cc;
  }
}

// Function for MATLAB Function: '<S2>/Filter'
static int32_T soccer_vision_findchanged(int32_T changed_data[], const int32_T
  idx[20], const int32_T previdx[20], const int32_T moved[20], int32_T nmoved)
{
  int32_T nchanged;
  int32_T i;
  for (i = 0; i < 20; i++) {
    soccer_vision_B.b[i] = false;
  }

  for (i = 0; i < nmoved; i++) {
    soccer_vision_B.b[idx[moved[i] - 1] - 1] = true;
    soccer_vision_B.b[previdx[moved[i] - 1] - 1] = true;
  }

  nchanged = 0;
  for (i = 0; i < 20; i++) {
    if (soccer_vision_B.b[i]) {
      nchanged++;
      changed_data[nchanged - 1] = i + 1;
    }
  }

  return nchanged;
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_gcentroids_f(real_T C_data[], int32_T C_size[2],
  int32_T counts_data[], const real_T X[40], const int32_T idx[20], int32_T
  clusters)
{
  int32_T cc;
  int32_T i;
  int32_T tmp;
  counts_data[clusters - 1] = 0;
  C_data[clusters - 1] = (rtNaN);
  tmp = (clusters + C_size[0]) - 1;
  C_data[tmp] = (rtNaN);
  cc = 0;
  C_data[clusters - 1] = 0.0;
  C_data[tmp] = 0.0;
  for (i = 0; i < 20; i++) {
    if (idx[i] == clusters) {
      cc++;
      C_data[clusters - 1] += X[i];
      C_data[tmp] += X[i + 20];
    }
  }

  counts_data[clusters - 1] = cc;
  C_data[clusters - 1] /= (real_T)cc;
  C_data[tmp] /= (real_T)cc;
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_batchUpdate(const real_T X[40], int32_T k, int32_T
  idx[20], real_T C_data[], int32_T C_size[2], real_T D_data[], int32_T D_size[2],
  int32_T counts_data[], boolean_T *converged, int32_T *iter)
{
  int32_T from;
  int32_T nempty;
  int32_T c_j;
  int32_T f_j;
  int32_T exitg1;
  boolean_T exitg2;
  if (0 <= k - 1) {
    memset(&soccer_vision_B.empties_data[0], 0, k * sizeof(int32_T));
  }

  memset(&soccer_vision_B.previdx[0], 0, 20U * sizeof(int32_T));
  memset(&soccer_vision_B.moved[0], 0, 20U * sizeof(int32_T));
  soccer_vision_B.nchanged = 1;
  while (soccer_vision_B.nchanged <= k) {
    soccer_vision_B.changed_data[soccer_vision_B.nchanged - 1] =
      soccer_vision_B.nchanged;
    soccer_vision_B.nchanged++;
  }

  soccer_vision_B.nchanged = k;
  soccer_vision_B.prevtotsumD = (rtInf);
  *iter = 0;
  *converged = false;
  do {
    exitg1 = 0;
    (*iter)++;
    soccer_vision_gcentroids(C_data, C_size, counts_data, X, idx,
      soccer_vision_B.changed_data, soccer_vision_B.nchanged);
    soccer_vision_distfun_k(D_data, D_size, X, C_data, C_size,
      soccer_vision_B.changed_data, soccer_vision_B.nchanged);
    nempty = 0;
    for (f_j = 0; f_j < soccer_vision_B.nchanged; f_j++) {
      if (counts_data[soccer_vision_B.changed_data[f_j] - 1] == 0) {
        nempty++;
        soccer_vision_B.empties_data[nempty - 1] =
          soccer_vision_B.changed_data[f_j];
      }
    }

    if (nempty > 0) {
      for (f_j = 0; f_j < nempty; f_j++) {
        soccer_vision_B.totsumD = D_data[(idx[0] - 1) * D_size[0]];
        soccer_vision_B.lonely = 0;
        for (from = 0; from < 20; from++) {
          if (D_data[(idx[from] - 1) * D_size[0] + from] >
              soccer_vision_B.totsumD) {
            soccer_vision_B.totsumD = D_data[(idx[from] - 1) * D_size[0] + from];
            soccer_vision_B.lonely = from;
          }
        }

        from = idx[soccer_vision_B.lonely];
        if (counts_data[idx[soccer_vision_B.lonely] - 1] < 2) {
          c_j = 1;
          exitg2 = false;
          while ((!exitg2) && (c_j < 21)) {
            if (counts_data[c_j - 1] > 1) {
              from = c_j;
              exitg2 = true;
            } else {
              c_j++;
            }
          }

          c_j = 0;
          exitg2 = false;
          while ((!exitg2) && (c_j + 1 < 21)) {
            if (idx[c_j] == from) {
              soccer_vision_B.lonely = c_j;
              exitg2 = true;
            } else {
              c_j++;
            }
          }
        }

        C_data[soccer_vision_B.empties_data[f_j] - 1] = X[soccer_vision_B.lonely];
        C_data[(soccer_vision_B.empties_data[f_j] + C_size[0]) - 1] =
          X[soccer_vision_B.lonely + 20];
        counts_data[soccer_vision_B.empties_data[f_j] - 1] = 1;
        idx[soccer_vision_B.lonely] = soccer_vision_B.empties_data[f_j];
        soccer_vision_distfun(D_data, D_size, X, C_data, C_size,
                              soccer_vision_B.empties_data[f_j]);
        soccer_vision_gcentroids_f(C_data, C_size, counts_data, X, idx, from);
        soccer_vision_distfun(D_data, D_size, X, C_data, C_size, from);
        if (soccer_vision_B.nchanged < k) {
          soccer_vision_B.lonely = 0;
          exitg2 = false;
          while ((!exitg2) && ((soccer_vision_B.lonely + 1 <=
                                soccer_vision_B.nchanged) && (!(from ==
                    soccer_vision_B.changed_data[soccer_vision_B.lonely])))) {
            if (from > soccer_vision_B.changed_data[soccer_vision_B.lonely]) {
              for (c_j = soccer_vision_B.nchanged; c_j >= soccer_vision_B.lonely
                   + 1; c_j--) {
                soccer_vision_B.changed_data[c_j] =
                  soccer_vision_B.changed_data[c_j - 1];
              }

              soccer_vision_B.changed_data[soccer_vision_B.lonely] = from;
              soccer_vision_B.nchanged++;
              exitg2 = true;
            } else {
              soccer_vision_B.lonely++;
            }
          }
        }
      }
    }

    soccer_vision_B.totsumD = 0.0;
    for (nempty = 0; nempty < 20; nempty++) {
      soccer_vision_B.totsumD += D_data[(idx[nempty] - 1) * D_size[0] + nempty];
    }

    if (soccer_vision_B.prevtotsumD <= soccer_vision_B.totsumD) {
      memcpy(&idx[0], &soccer_vision_B.previdx[0], 20U * sizeof(int32_T));
      soccer_vision_gcentroids(C_data, C_size, counts_data, X,
        soccer_vision_B.previdx, soccer_vision_B.changed_data,
        soccer_vision_B.nchanged);
      (*iter)--;
      exitg1 = 1;
    } else if (*iter >= 15) {
      exitg1 = 1;
    } else {
      memcpy(&soccer_vision_B.previdx[0], &idx[0], 20U * sizeof(int32_T));
      soccer_vision_B.prevtotsumD = soccer_vision_B.totsumD;
      soccer_vision_mindim2(D_data, D_size, soccer_vision_B.d_m,
                            soccer_vision_B.nidx);
      soccer_vision_B.nchanged = 0;
      for (nempty = 0; nempty < 20; nempty++) {
        if ((soccer_vision_B.nidx[nempty] != soccer_vision_B.previdx[nempty]) &&
            (D_data[(soccer_vision_B.previdx[nempty] - 1) * D_size[0] + nempty] >
             soccer_vision_B.d_m[nempty])) {
          soccer_vision_B.nchanged++;
          soccer_vision_B.moved[soccer_vision_B.nchanged - 1] = nempty + 1;
          idx[nempty] = soccer_vision_B.nidx[nempty];
        }
      }

      if (soccer_vision_B.nchanged == 0) {
        *converged = true;
        exitg1 = 1;
      } else {
        soccer_vision_B.nchanged = soccer_vision_findchanged
          (soccer_vision_B.changed_data, idx, soccer_vision_B.previdx,
           soccer_vision_B.moved, soccer_vision_B.nchanged);
      }
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S2>/Filter'
static void soccer_vision_kmeans(const real_T X[40], real_T kin, real_T idxbest
  [20], real_T Cbest_data[], int32_T Cbest_size[2], real_T varargout_1_data[],
  int32_T *varargout_1_size)
{
  boolean_T DNeedsComputing;
  boolean_T exitg1;
  soccer_vision_B.b_index = soccer_vision_rand();
  Cbest_size[0] = (int32_T)kin;
  Cbest_size[1] = 2;
  soccer_vision_B.nNonEmpty = ((int32_T)kin << 1) - 1;
  if (0 <= soccer_vision_B.nNonEmpty) {
    memset(&Cbest_data[0], 0, (soccer_vision_B.nNonEmpty + 1) * sizeof(real_T));
  }

  soccer_vision_B.pidx = (int32_T)(floor(soccer_vision_B.b_index * 20.0) + 1.0);
  Cbest_data[0] = X[soccer_vision_B.pidx - 1];
  Cbest_data[Cbest_size[0]] = X[soccer_vision_B.pidx + 19];
  soccer_vision_B.D_size[0] = 20;
  soccer_vision_B.D_size[1] = (int32_T)kin;
  soccer_vision_B.nNonEmpty = 20 * (int32_T)kin - 1;
  if (0 <= soccer_vision_B.nNonEmpty) {
    memset(&soccer_vision_B.D_data[0], 0, (soccer_vision_B.nNonEmpty + 1) *
           sizeof(real_T));
  }

  soccer_vision_distfun(soccer_vision_B.D_data, soccer_vision_B.D_size, X,
                        Cbest_data, Cbest_size, 1);
  for (soccer_vision_B.pidx = 0; soccer_vision_B.pidx < 20; soccer_vision_B.pidx
       ++) {
    soccer_vision_B.d[soccer_vision_B.pidx] =
      soccer_vision_B.D_data[soccer_vision_B.pidx];
    soccer_vision_B.idx[soccer_vision_B.pidx] = 1;
  }

  memset(&soccer_vision_B.sampleDist[0], 0, 21U * sizeof(real_T));
  DNeedsComputing = false;
  soccer_vision_B.nNonEmpty = 2;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.nNonEmpty <= (int32_T)kin)) {
    soccer_vision_B.b_index = 0.0;
    soccer_vision_B.sampleDist[0] = 0.0;
    for (soccer_vision_B.pidx = 0; soccer_vision_B.pidx < 20;
         soccer_vision_B.pidx++) {
      soccer_vision_B.sampleDist[soccer_vision_B.pidx + 1] =
        soccer_vision_B.sampleDist[soccer_vision_B.pidx] +
        soccer_vision_B.d[soccer_vision_B.pidx];
      soccer_vision_B.b_index += soccer_vision_B.d[soccer_vision_B.pidx];
    }

    if ((soccer_vision_B.b_index == 0.0) || (!soccer_vision_isfinite
         (soccer_vision_B.b_index))) {
      soccer_vision_simpleRandperm(((int32_T)kin - soccer_vision_B.nNonEmpty) +
        1, soccer_vision_B.idx);
      soccer_vision_B.pidx = soccer_vision_B.nNonEmpty;
      while (soccer_vision_B.pidx <= (int32_T)kin) {
        Cbest_data[soccer_vision_B.pidx - 1] =
          X[soccer_vision_B.idx[soccer_vision_B.pidx - soccer_vision_B.nNonEmpty]
          - 1];
        soccer_vision_B.pidx++;
      }

      soccer_vision_B.pidx = soccer_vision_B.nNonEmpty;
      while (soccer_vision_B.pidx <= (int32_T)kin) {
        Cbest_data[(soccer_vision_B.pidx + Cbest_size[0]) - 1] =
          X[soccer_vision_B.idx[soccer_vision_B.pidx - soccer_vision_B.nNonEmpty]
          + 19];
        soccer_vision_B.pidx++;
      }

      DNeedsComputing = true;
      exitg1 = true;
    } else {
      for (soccer_vision_B.pidx = 0; soccer_vision_B.pidx < 21;
           soccer_vision_B.pidx++) {
        soccer_vision_B.sampleDist[soccer_vision_B.pidx] /=
          soccer_vision_B.b_index;
      }

      soccer_vision_B.pidx = soccer_vision_bsearch(soccer_vision_B.sampleDist,
        soccer_vision_rand());
      soccer_vision_B.b_index = soccer_vision_B.sampleDist[soccer_vision_B.pidx
        - 1];
      if (soccer_vision_B.sampleDist[soccer_vision_B.pidx - 1] < 1.0) {
        while ((soccer_vision_B.pidx <= 20) &&
               (soccer_vision_B.sampleDist[soccer_vision_B.pidx] <=
                soccer_vision_B.b_index)) {
          soccer_vision_B.pidx++;
        }
      } else {
        while ((soccer_vision_B.pidx >= 2) &&
               (soccer_vision_B.sampleDist[soccer_vision_B.pidx - 2] >=
                soccer_vision_B.b_index)) {
          soccer_vision_B.pidx--;
        }
      }

      Cbest_data[soccer_vision_B.nNonEmpty - 1] = X[soccer_vision_B.pidx - 1];
      Cbest_data[(soccer_vision_B.nNonEmpty + Cbest_size[0]) - 1] =
        X[soccer_vision_B.pidx + 19];
      soccer_vision_distfun(soccer_vision_B.D_data, soccer_vision_B.D_size, X,
                            Cbest_data, Cbest_size, soccer_vision_B.nNonEmpty);
      for (soccer_vision_B.pidx = 0; soccer_vision_B.pidx < 20;
           soccer_vision_B.pidx++) {
        soccer_vision_B.b_index = soccer_vision_B.d[soccer_vision_B.pidx];
        if (soccer_vision_B.D_data[(soccer_vision_B.nNonEmpty - 1) *
            soccer_vision_B.D_size[0] + soccer_vision_B.pidx] <
            soccer_vision_B.d[soccer_vision_B.pidx]) {
          soccer_vision_B.b_index = soccer_vision_B.D_data
            [(soccer_vision_B.nNonEmpty - 1) * soccer_vision_B.D_size[0] +
            soccer_vision_B.pidx];
          soccer_vision_B.idx[soccer_vision_B.pidx] = soccer_vision_B.nNonEmpty;
        }

        soccer_vision_B.d[soccer_vision_B.pidx] = soccer_vision_B.b_index;
      }

      soccer_vision_B.nNonEmpty++;
    }
  }

  if (DNeedsComputing) {
    soccer_vision_B.nNonEmpty = 1;
    while (soccer_vision_B.nNonEmpty <= (int32_T)kin) {
      soccer_vision_B.crows_data[soccer_vision_B.nNonEmpty - 1] =
        soccer_vision_B.nNonEmpty;
      soccer_vision_B.nNonEmpty++;
    }

    soccer_vision_distfun_k(soccer_vision_B.D_data, soccer_vision_B.D_size, X,
      Cbest_data, Cbest_size, soccer_vision_B.crows_data, (int32_T)kin);
    soccer_vision_mindim2(soccer_vision_B.D_data, soccer_vision_B.D_size,
                          soccer_vision_B.d, soccer_vision_B.idx);
  }

  soccer_vision_B.nNonEmpty = (int32_T)kin;
  if (0 <= soccer_vision_B.nNonEmpty - 1) {
    memset(&soccer_vision_B.crows_data[0], 0, soccer_vision_B.nNonEmpty * sizeof
           (int32_T));
  }

  for (soccer_vision_B.nNonEmpty = 0; soccer_vision_B.nNonEmpty < 20;
       soccer_vision_B.nNonEmpty++) {
    soccer_vision_B.crows_data[soccer_vision_B.idx[soccer_vision_B.nNonEmpty] -
      1]++;
  }

  soccer_vision_B.nNonEmpty = (int32_T)kin;
  if (0 <= soccer_vision_B.nNonEmpty - 1) {
    memset(&soccer_vision_B.nonEmpties_data[0], 0, soccer_vision_B.nNonEmpty *
           sizeof(int32_T));
  }

  soccer_vision_batchUpdate(X, (int32_T)kin, soccer_vision_B.idx, Cbest_data,
    Cbest_size, soccer_vision_B.D_data, soccer_vision_B.D_size,
    soccer_vision_B.crows_data, &DNeedsComputing, &soccer_vision_B.nNonEmpty);
  soccer_vision_B.nNonEmpty = 0;
  soccer_vision_B.pidx = 1;
  while (soccer_vision_B.pidx <= (int32_T)kin) {
    if (soccer_vision_B.crows_data[soccer_vision_B.pidx - 1] > 0) {
      soccer_vision_B.nNonEmpty++;
      soccer_vision_B.nonEmpties_data[soccer_vision_B.nNonEmpty - 1] =
        soccer_vision_B.pidx;
    }

    soccer_vision_B.pidx++;
  }

  soccer_vision_distfun_k(soccer_vision_B.D_data, soccer_vision_B.D_size, X,
    Cbest_data, Cbest_size, soccer_vision_B.nonEmpties_data,
    soccer_vision_B.nNonEmpty);
  for (soccer_vision_B.nNonEmpty = 0; soccer_vision_B.nNonEmpty < 20;
       soccer_vision_B.nNonEmpty++) {
    soccer_vision_B.d[soccer_vision_B.nNonEmpty] = soccer_vision_B.D_data
      [(soccer_vision_B.idx[soccer_vision_B.nNonEmpty] - 1) *
      soccer_vision_B.D_size[0] + soccer_vision_B.nNonEmpty];
  }

  *varargout_1_size = (int32_T)kin;
  soccer_vision_B.nNonEmpty = (int32_T)kin;
  if (0 <= soccer_vision_B.nNonEmpty - 1) {
    memset(&varargout_1_data[0], 0, soccer_vision_B.nNonEmpty * sizeof(real_T));
  }

  for (soccer_vision_B.nNonEmpty = 0; soccer_vision_B.nNonEmpty < 20;
       soccer_vision_B.nNonEmpty++) {
    varargout_1_data[soccer_vision_B.idx[soccer_vision_B.nNonEmpty] - 1] +=
      soccer_vision_B.d[soccer_vision_B.nNonEmpty];
    idxbest[soccer_vision_B.nNonEmpty] =
      soccer_vision_B.idx[soccer_vision_B.nNonEmpty];
  }
}

// Function for MATLAB Function: '<S12>/Label Net Lines'
static real_T soccer_vision_norm(const real_T x[2])
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

// Function for MATLAB Function: '<S12>/Label Field Lines'
static void soccer_vision_merge(int32_T idx[5], real_T x[5], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork[5], real_T xwork[5])
{
  int32_T p;
  int32_T q;
  int32_T iout;
  int32_T offset1;
  int32_T exitg1;
  if (!((np == 0) || (nq == 0))) {
    offset1 = np + nq;
    for (p = 0; p < offset1; p++) {
      q = offset + p;
      iwork[p] = idx[q];
      xwork[p] = x[q];
    }

    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork[p] <= xwork[q]) {
        idx[iout] = iwork[p];
        x[iout] = xwork[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if (q + 1 < offset1) {
          q++;
        } else {
          offset1 = iout - p;
          while (p + 1 <= np) {
            q = (offset1 + p) + 1;
            idx[q] = iwork[p];
            x[q] = xwork[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

// Function for MATLAB Function: '<S12>/Label Field Lines'
static void soccer_vision_sort(real_T x[5], int32_T idx[5])
{
  int32_T m;
  int8_T perm[4];
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  soccer_vision_B.x4[0] = 0.0;
  soccer_vision_B.idx4[0] = 0;
  soccer_vision_B.x4[1] = 0.0;
  soccer_vision_B.idx4[1] = 0;
  soccer_vision_B.x4[2] = 0.0;
  soccer_vision_B.idx4[2] = 0;
  soccer_vision_B.x4[3] = 0.0;
  soccer_vision_B.idx4[3] = 0;
  for (m = 0; m < 5; m++) {
    idx[m] = 0;
    soccer_vision_B.xwork[m] = 0.0;
  }

  soccer_vision_B.nNaNs = 0;
  soccer_vision_B.ib = 0;
  for (m = 0; m < 5; m++) {
    if (rtIsNaN(x[m])) {
      idx[4 - soccer_vision_B.nNaNs] = m + 1;
      soccer_vision_B.xwork[4 - soccer_vision_B.nNaNs] = x[m];
      soccer_vision_B.nNaNs++;
    } else {
      soccer_vision_B.ib++;
      soccer_vision_B.idx4[soccer_vision_B.ib - 1] = (int8_T)(m + 1);
      soccer_vision_B.x4[soccer_vision_B.ib - 1] = x[m];
      if (soccer_vision_B.ib == 4) {
        soccer_vision_B.ib = m - soccer_vision_B.nNaNs;
        if (soccer_vision_B.x4[0] <= soccer_vision_B.x4[1]) {
          i1 = 1;
          i2 = 2;
        } else {
          i1 = 2;
          i2 = 1;
        }

        if (soccer_vision_B.x4[2] <= soccer_vision_B.x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        if (soccer_vision_B.x4[i1 - 1] <= soccer_vision_B.x4[i3 - 1]) {
          if (soccer_vision_B.x4[i2 - 1] <= soccer_vision_B.x4[i3 - 1]) {
            perm[0] = (int8_T)i1;
            perm[1] = (int8_T)i2;
            perm[2] = (int8_T)i3;
            perm[3] = (int8_T)i4;
          } else if (soccer_vision_B.x4[i2 - 1] <= soccer_vision_B.x4[i4 - 1]) {
            perm[0] = (int8_T)i1;
            perm[1] = (int8_T)i3;
            perm[2] = (int8_T)i2;
            perm[3] = (int8_T)i4;
          } else {
            perm[0] = (int8_T)i1;
            perm[1] = (int8_T)i3;
            perm[2] = (int8_T)i4;
            perm[3] = (int8_T)i2;
          }
        } else if (soccer_vision_B.x4[i1 - 1] <= soccer_vision_B.x4[i4 - 1]) {
          if (soccer_vision_B.x4[i2 - 1] <= soccer_vision_B.x4[i4 - 1]) {
            perm[0] = (int8_T)i3;
            perm[1] = (int8_T)i1;
            perm[2] = (int8_T)i2;
            perm[3] = (int8_T)i4;
          } else {
            perm[0] = (int8_T)i3;
            perm[1] = (int8_T)i1;
            perm[2] = (int8_T)i4;
            perm[3] = (int8_T)i2;
          }
        } else {
          perm[0] = (int8_T)i3;
          perm[1] = (int8_T)i4;
          perm[2] = (int8_T)i1;
          perm[3] = (int8_T)i2;
        }

        i1 = perm[0] - 1;
        idx[soccer_vision_B.ib - 3] = soccer_vision_B.idx4[i1];
        i2 = perm[1] - 1;
        idx[soccer_vision_B.ib - 2] = soccer_vision_B.idx4[i2];
        i3 = perm[2] - 1;
        idx[soccer_vision_B.ib - 1] = soccer_vision_B.idx4[i3];
        i4 = perm[3] - 1;
        idx[soccer_vision_B.ib] = soccer_vision_B.idx4[i4];
        x[soccer_vision_B.ib - 3] = soccer_vision_B.x4[i1];
        x[soccer_vision_B.ib - 2] = soccer_vision_B.x4[i2];
        x[soccer_vision_B.ib - 1] = soccer_vision_B.x4[i3];
        x[soccer_vision_B.ib] = soccer_vision_B.x4[i4];
        soccer_vision_B.ib = 0;
      }
    }
  }

  if (soccer_vision_B.ib > 0) {
    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    switch (soccer_vision_B.ib) {
     case 1:
      perm[0] = 1;
      break;

     case 2:
      if (soccer_vision_B.x4[0] <= soccer_vision_B.x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
      break;

     default:
      if (soccer_vision_B.x4[0] <= soccer_vision_B.x4[1]) {
        if (soccer_vision_B.x4[1] <= soccer_vision_B.x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (soccer_vision_B.x4[0] <= soccer_vision_B.x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (soccer_vision_B.x4[0] <= soccer_vision_B.x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (soccer_vision_B.x4[1] <= soccer_vision_B.x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }
      break;
    }

    for (m = 5; m - 4 <= soccer_vision_B.ib; m++) {
      i1 = perm[m - 5] - 1;
      i2 = (m - soccer_vision_B.nNaNs) - soccer_vision_B.ib;
      idx[i2] = soccer_vision_B.idx4[i1];
      x[i2] = soccer_vision_B.x4[i1];
    }
  }

  m = soccer_vision_B.nNaNs >> 1;
  soccer_vision_B.ib = 1;
  while (soccer_vision_B.ib <= m) {
    i1 = (soccer_vision_B.ib - soccer_vision_B.nNaNs) + 4;
    i2 = idx[i1];
    idx[i1] = idx[5 - soccer_vision_B.ib];
    idx[5 - soccer_vision_B.ib] = i2;
    x[i1] = soccer_vision_B.xwork[5 - soccer_vision_B.ib];
    x[5 - soccer_vision_B.ib] = soccer_vision_B.xwork[i1];
    soccer_vision_B.ib++;
  }

  if ((soccer_vision_B.nNaNs & 1U) != 0U) {
    x[(m - soccer_vision_B.nNaNs) + 5] = soccer_vision_B.xwork[(m -
      soccer_vision_B.nNaNs) + 5];
  }

  if (5 - soccer_vision_B.nNaNs > 1) {
    for (m = 0; m < 5; m++) {
      soccer_vision_B.iwork[m] = 0;
    }

    soccer_vision_B.ib = (5 - soccer_vision_B.nNaNs) >> 2;
    m = 4;
    while (soccer_vision_B.ib > 1) {
      if ((soccer_vision_B.ib & 1U) != 0U) {
        soccer_vision_B.ib--;
        i1 = m * soccer_vision_B.ib;
        i2 = 5 - (soccer_vision_B.nNaNs + i1);
        if (i2 > m) {
          soccer_vision_merge(idx, x, i1, m, i2 - m, soccer_vision_B.iwork,
                              soccer_vision_B.xwork);
        }
      }

      i1 = m << 1;
      soccer_vision_B.ib >>= 1;
      for (i2 = 1; i2 <= soccer_vision_B.ib; i2++) {
        soccer_vision_merge(idx, x, (i2 - 1) * i1, m, m, soccer_vision_B.iwork,
                            soccer_vision_B.xwork);
      }

      m = i1;
    }

    if (5 - soccer_vision_B.nNaNs > m) {
      soccer_vision_merge(idx, x, 0, m, 5 - (soccer_vision_B.nNaNs + m),
                          soccer_vision_B.iwork, soccer_vision_B.xwork);
    }
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2((real_T)u0_0, (real_T)u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static Point2f_soccer_vision_T *soccer_vision_Line2f_screenIntersection(const
  Line2f_soccer_vision_T *obj1, const Line2f_soccer_vision_T *obj2,
  Point2f_soccer_vision_T *iobj_0)
{
  Point2f_soccer_vision_T *intersect;
  real_T theta1;
  real_T theta2;
  real_T b[2];
  int32_T r1;
  int32_T r2;

  // LINE Summary of this class goes here
  //    Detailed explanation goes here
  // LINE Construct an instance of this class
  //    Detailed explanation goes here
  // METHOD1 Finds the screen intersection between 2 points
  b[0] = obj1->rho;
  theta1 = obj1->theta;
  b[1] = obj2->rho;
  theta2 = obj2->theta;
  soccer_vision_B.A[0] = cos(theta1);
  soccer_vision_B.A[2] = sin(theta1);
  soccer_vision_B.A[1] = cos(theta2);
  soccer_vision_B.A[3] = sin(theta2);
  if (fabs(soccer_vision_B.A[1]) > fabs(soccer_vision_B.A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  theta1 = soccer_vision_B.A[r2] / soccer_vision_B.A[r1];
  theta1 = (b[r2] - b[r1] * theta1) / (soccer_vision_B.A[2 + r2] -
    soccer_vision_B.A[2 + r1] * theta1);

  //  Create the intersection
  // POINT2F Summary of this class goes here
  //    Detailed explanation goes here
  intersect = iobj_0;
  iobj_0->x = (b[r1] - soccer_vision_B.A[2 + r1] * theta1) /
    soccer_vision_B.A[r1];
  iobj_0->y = theta1;
  return intersect;
}

static void soccer_vision_SystemProp_matlabCodegenSetAnyProp
  (robotics_slros_internal_block_Subscriber_soccer_vision_T *obj, boolean_T
   value)
{
  obj->matlabCodegenIsDeleted = value;
}

static void soccer_vision_matlabCodegenHandle_matlabCodegenDestructor
  (robotics_slros_internal_block_Subscriber_soccer_vision_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    soccer_vision_SystemProp_matlabCodegenSetAnyProp(obj, true);
  }
}

// Model step function
void soccer_vision_step(void)
{
  boolean_T y[2];
  robotics_slros_internal_block_ReadImage_soccer_vision_T *obj;
  boolean_T b4;
  boolean_T b3;
  boolean_T b2;
  boolean_T b1;
  int16_T CoverAboveHorizon_DW_OneMOpacity;
  int32_T *lines;
  boolean_T exitg1;

  // Reset subsysRan breadcrumbs
  srClearBC(soccer_vision_DW.EnabledSubsystem_SubsysRanBC);
  if (soccer_vision_M->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<S4>/Subscribe'
    // MATLABSystem: '<S5>/SourceBlock' incorporates:
    //   Inport: '<S6>/In1'

    b1 = Sub_soccer_vision_197.getLatestMessage(&soccer_vision_B.b_varargout_2);

    // Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S6>/Enable'

    if (b1) {
      soccer_vision_B.In1 = soccer_vision_B.b_varargout_2;
      srUpdateBC(soccer_vision_DW.EnabledSubsystem_SubsysRanBC);
    }

    // End of MATLABSystem: '<S5>/SourceBlock'
    // End of Outputs for SubSystem: '<S5>/Enabled Subsystem'
    // End of Outputs for SubSystem: '<S4>/Subscribe'

    // MATLABSystem: '<S4>/Read Image'
    obj = &soccer_vision_DW.obj;
    soccer_vision_B.ReadImage_o2 = soccer_vision_ReadImage_update
      (&soccer_vision_DW.obj, soccer_vision_B.In1.Height,
       soccer_vision_B.In1.Width, soccer_vision_B.In1.Encoding,
       soccer_vision_B.In1.Encoding_SL_Info.CurrentLength,
       soccer_vision_B.In1.Data, soccer_vision_B.In1.Data_SL_Info.CurrentLength,
       soccer_vision_B.In1.Data_SL_Info.ReceivedLength);
    for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 3;
         soccer_vision_B.imgCol++) {
      for (soccer_vision_B.q = 0; soccer_vision_B.q < 640; soccer_vision_B.q++)
      {
        for (soccer_vision_B.accumOne = 0; soccer_vision_B.accumOne < 480;
             soccer_vision_B.accumOne++) {
          soccer_vision_B.fullImage[(soccer_vision_B.accumOne + 480 *
            soccer_vision_B.q) + 307200 * soccer_vision_B.imgCol] = obj->Image
            [(((soccer_vision_B.imgCol + 1) - 1) * 307200 + 480 *
              soccer_vision_B.q) + soccer_vision_B.accumOne];
        }
      }
    }

    // MATLAB Function: '<S2>/Cover Edge' incorporates:
    //   Constant: '<S2>/Constant6'

    soccer_vision_B.totalLineCount = 240.0 - 0.0 *
      soccer_vision_P.Constant6_Value;

    // Copy the image from input to output.
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 307200; soccer_vision_B.q++)
    {
      // S-Function (svipdrawshapes): '<S2>/Cover Above Horizon' incorporates:
      //   MATLABSystem: '<S4>/Read Image'
      //   S-Function (svipcolorconv): '<S4>/Color Space  Conversion'

      soccer_vision_B.CoverAboveHorizon[soccer_vision_B.q] = (uint8_T)
        ((((soccer_vision_B.fullImage[307200 + soccer_vision_B.q] * 38470U +
            soccer_vision_B.fullImage[soccer_vision_B.q] * 19595U) +
           soccer_vision_B.fullImage[614400 + soccer_vision_B.q] * 7471U) +
          32768U) >> 16);
    }

    // S-Function (svipdrawshapes): '<S2>/Cover Above Horizon'
    // Calculate FillColor times Opacity.
    // Calculate One minus Opacity.
    CoverAboveHorizon_DW_OneMOpacity = (int16_T)(16384 -
      soccer_vision_P.CoverAboveHorizon_RTP_OPACITY);

    // MATLAB Function: '<S2>/Cover Edge'
    // Update view port.
    // Draw all filled rectangles.
    // Find the overlapping area of the rectangle and the view port.
    if (soccer_vision_B.totalLineCount < 4.294967296E+9) {
      soccer_vision_B.horizonY = 240U;
    } else {
      soccer_vision_B.horizonY = MAX_uint32_T;
    }

    // S-Function (svipdrawshapes): '<S2>/Cover Above Horizon'
    soccer_vision_B.imgCol = (int32_T)soccer_vision_B.horizonY - 2;
    if (479 <= soccer_vision_B.imgCol) {
      soccer_vision_B.imgCol = 479;
    }

    // Draw a filled rectangle.
    if (0 <= soccer_vision_B.imgCol) {
      soccer_vision_B.accumOne = 0;
      while (soccer_vision_B.accumOne <= 638) {
        soccer_vision_B.accumTwo = soccer_vision_B.accumOne * 480;
        soccer_vision_B.q = 0;
        while (soccer_vision_B.q <= soccer_vision_B.imgCol) {
          soccer_vision_B.CoverAboveHorizon[soccer_vision_B.accumTwo] = (uint8_T)
            ((soccer_vision_B.CoverAboveHorizon[soccer_vision_B.accumTwo] *
              CoverAboveHorizon_DW_OneMOpacity) >> 14);
          soccer_vision_B.accumTwo++;
          soccer_vision_B.q++;
        }

        soccer_vision_B.accumOne++;
      }
    }

    // S-Function (svipresize): '<S10>/Resize'
    // this algorithm computes interpolation weights on demand as oppose to using a lookup table 
    // first resize along X-axis direction
    soccer_vision_B.accumFour = 0;
    for (soccer_vision_B.accumOne = 0; soccer_vision_B.accumOne < 160;
         soccer_vision_B.accumOne++) {
      soccer_vision_B.accumTwo = (((soccer_vision_B.accumOne << 10) + 512) << 2)
        - 512;
      soccer_vision_B.q = soccer_vision_B.accumTwo >> 10;
      soccer_vision_B.imgIdx = soccer_vision_B.q * 480;
      soccer_vision_B.imgRow = soccer_vision_B.imgIdx + 480;
      soccer_vision_B.accumTwo -= soccer_vision_B.q << 10;

      // bilinear interpolation
      soccer_vision_B.imgIdx_d = 1024 - soccer_vision_B.accumTwo;

      // adjust offsets so that the input image index will not exceed the image bounds 
      for (soccer_vision_B.q = 0; soccer_vision_B.q < 480; soccer_vision_B.q++)
      {
        soccer_vision_B.i0 = (int64_T)
          soccer_vision_B.CoverAboveHorizon[soccer_vision_B.q +
          soccer_vision_B.imgIdx] * soccer_vision_B.imgIdx_d;
        if (soccer_vision_B.i0 > 2147483647L) {
          soccer_vision_B.i0 = 2147483647L;
        } else {
          if (soccer_vision_B.i0 < -2147483648L) {
            soccer_vision_B.i0 = -2147483648L;
          }
        }

        soccer_vision_B.q0 = (int32_T)soccer_vision_B.i0;
        soccer_vision_B.i0 = (int64_T)
          soccer_vision_B.CoverAboveHorizon[soccer_vision_B.imgRow +
          soccer_vision_B.q] * soccer_vision_B.accumTwo;
        if (soccer_vision_B.i0 > 2147483647L) {
          soccer_vision_B.i0 = 2147483647L;
        } else {
          if (soccer_vision_B.i0 < -2147483648L) {
            soccer_vision_B.i0 = -2147483648L;
          }
        }

        soccer_vision_B.imgCol = (int32_T)soccer_vision_B.i0;
        if ((soccer_vision_B.q0 < 0) && (soccer_vision_B.imgCol < MIN_int32_T
             - soccer_vision_B.q0)) {
          soccer_vision_B.imgCol = MIN_int32_T;
        } else if ((soccer_vision_B.q0 > 0) && (soccer_vision_B.imgCol >
                    MAX_int32_T - soccer_vision_B.q0)) {
          soccer_vision_B.imgCol = MAX_int32_T;
        } else {
          soccer_vision_B.imgCol += soccer_vision_B.q0;
        }

        soccer_vision_B.imgCol = ((soccer_vision_B.imgCol & 512U) != 0U) +
          (soccer_vision_B.imgCol >> 10);
        if (soccer_vision_B.imgCol < 0) {
          soccer_vision_B.imgCol = 0;
        } else {
          if (soccer_vision_B.imgCol > 255) {
            soccer_vision_B.imgCol = 255;
          }
        }

        soccer_vision_DW.Resize_IntBuffer[soccer_vision_B.accumFour +
          soccer_vision_B.q] = (uint8_T)soccer_vision_B.imgCol;
      }

      soccer_vision_B.accumFour += 480;
    }

    // resize along Y-axis direction
    soccer_vision_B.accumFour = 0;
    for (soccer_vision_B.accumOne = 0; soccer_vision_B.accumOne < 160;
         soccer_vision_B.accumOne++) {
      soccer_vision_B.q = soccer_vision_B.accumOne * 480;
      for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 480;
           soccer_vision_B.imgCol++) {
        soccer_vision_DW.Resize_LineBuffer[soccer_vision_B.imgCol] =
          soccer_vision_DW.Resize_IntBuffer[soccer_vision_B.q];
        soccer_vision_B.q++;
      }

      soccer_vision_B.imgRow = soccer_vision_B.accumFour;
      for (soccer_vision_B.q = 0; soccer_vision_B.q < 120; soccer_vision_B.q++)
      {
        soccer_vision_B.accumTwo = (((soccer_vision_B.q << 10) + 512) << 2) -
          512;
        soccer_vision_B.imgIdx = soccer_vision_B.accumTwo >> 10;
        soccer_vision_B.accumTwo -= soccer_vision_B.imgIdx << 10;
        soccer_vision_B.i0 = (int64_T)(1024 - soccer_vision_B.accumTwo) *
          soccer_vision_DW.Resize_LineBuffer[soccer_vision_B.imgIdx];
        if (soccer_vision_B.i0 > 2147483647L) {
          soccer_vision_B.i0 = 2147483647L;
        } else {
          if (soccer_vision_B.i0 < -2147483648L) {
            soccer_vision_B.i0 = -2147483648L;
          }
        }

        soccer_vision_B.q0 = (int32_T)soccer_vision_B.i0;
        soccer_vision_B.i0 = (int64_T)
          soccer_vision_DW.Resize_LineBuffer[soccer_vision_B.imgIdx + 1] *
          soccer_vision_B.accumTwo;
        if (soccer_vision_B.i0 > 2147483647L) {
          soccer_vision_B.i0 = 2147483647L;
        } else {
          if (soccer_vision_B.i0 < -2147483648L) {
            soccer_vision_B.i0 = -2147483648L;
          }
        }

        soccer_vision_B.imgCol = (int32_T)soccer_vision_B.i0;
        if ((soccer_vision_B.q0 < 0) && (soccer_vision_B.imgCol < MIN_int32_T
             - soccer_vision_B.q0)) {
          soccer_vision_B.accumTwo = MIN_int32_T;
        } else if ((soccer_vision_B.q0 > 0) && (soccer_vision_B.imgCol >
                    MAX_int32_T - soccer_vision_B.q0)) {
          soccer_vision_B.accumTwo = MAX_int32_T;
        } else {
          soccer_vision_B.accumTwo = soccer_vision_B.q0 + soccer_vision_B.imgCol;
        }

        soccer_vision_B.imgCol = ((soccer_vision_B.accumTwo & 512U) != 0U) +
          (soccer_vision_B.accumTwo >> 10);
        if (soccer_vision_B.imgCol < 0) {
          soccer_vision_B.imgCol = 0;
        } else {
          if (soccer_vision_B.imgCol > 255) {
            soccer_vision_B.imgCol = 255;
          }
        }

        soccer_vision_B.Resize[soccer_vision_B.imgRow] = (uint8_T)
          soccer_vision_B.imgCol;
        soccer_vision_B.imgRow++;
      }

      soccer_vision_B.accumFour += 120;
    }

    // End of S-Function (svipresize): '<S10>/Resize'

    // S-Function (svipedge): '<S10>/Edge Detection'
    for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 158;
         soccer_vision_B.imgCol++) {
      for (soccer_vision_B.imgRow = 0; soccer_vision_B.imgRow < 118;
           soccer_vision_B.imgRow++) {
        soccer_vision_B.accumOne = 0;
        soccer_vision_B.accumTwo = 0;
        soccer_vision_B.imgIdx = ((soccer_vision_B.imgCol + 1) * 120 +
          soccer_vision_B.imgRow) + 1;
        for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++)
        {
          soccer_vision_B.accumOne += (int32_T)(((int64_T)
            soccer_vision_B.Resize[soccer_vision_B.imgIdx +
            soccer_vision_DW.EdgeDetection_VO_DW[soccer_vision_B.q]] *
            soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
          soccer_vision_B.accumTwo += (int32_T)(((int64_T)
            soccer_vision_B.Resize[soccer_vision_B.imgIdx +
            soccer_vision_DW.EdgeDetection_HO_DW[soccer_vision_B.q]] *
            soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
        }

        soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.imgIdx] =
          (int32_T)(((int64_T)soccer_vision_B.accumOne *
                     soccer_vision_B.accumOne) >> 8);
        soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.imgIdx] =
          (int32_T)(((int64_T)soccer_vision_B.accumTwo *
                     soccer_vision_B.accumTwo) >> 8);
      }
    }

    for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 158;
         soccer_vision_B.imgCol++) {
      soccer_vision_B.accumOne = 0;
      soccer_vision_B.accumTwo = 0;
      soccer_vision_B.imgIdx = 0;
      soccer_vision_B.accumFour = 0;
      soccer_vision_B.imgRow = (soccer_vision_B.imgCol + 1) * 120;
      soccer_vision_B.imgIdx_d = (soccer_vision_B.imgCol + 1) * 120 + 119;
      for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++) {
        soccer_vision_B.accumOne += (int32_T)(((int64_T)
          soccer_vision_B.Resize[soccer_vision_B.imgRow +
          soccer_vision_DW.EdgeDetection_HOU_DW[soccer_vision_B.q]] *
          soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
        soccer_vision_B.accumTwo += (int32_T)(((int64_T)
          soccer_vision_B.Resize[soccer_vision_B.imgIdx_d +
          soccer_vision_DW.EdgeDetection_HOD_DW[soccer_vision_B.q]] *
          soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
        soccer_vision_B.imgIdx += (int32_T)(((int64_T)
          soccer_vision_B.Resize[soccer_vision_B.imgRow +
          soccer_vision_DW.EdgeDetection_VOU_DW[soccer_vision_B.q]] *
          soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
        soccer_vision_B.accumFour += (int32_T)(((int64_T)
          soccer_vision_B.Resize[soccer_vision_B.imgIdx_d +
          soccer_vision_DW.EdgeDetection_VOD_DW[soccer_vision_B.q]] *
          soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
      }

      soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.imgRow] =
        (int32_T)(((int64_T)soccer_vision_B.imgIdx * soccer_vision_B.imgIdx) >>
                  8);
      soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.imgRow] =
        (int32_T)(((int64_T)soccer_vision_B.accumOne * soccer_vision_B.accumOne)
                  >> 8);
      soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.imgIdx_d] =
        (int32_T)(((int64_T)soccer_vision_B.accumFour *
                   soccer_vision_B.accumFour) >> 8);
      soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.imgIdx_d] =
        (int32_T)(((int64_T)soccer_vision_B.accumTwo * soccer_vision_B.accumTwo)
                  >> 8);
    }

    for (soccer_vision_B.imgRow = 0; soccer_vision_B.imgRow < 118;
         soccer_vision_B.imgRow++) {
      soccer_vision_B.accumOne = 0;
      soccer_vision_B.accumTwo = 0;
      soccer_vision_B.imgIdx = 0;
      soccer_vision_B.accumFour = 0;
      for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++) {
        soccer_vision_B.accumOne += (int32_T)(((int64_T)soccer_vision_B.Resize
          [(soccer_vision_B.imgRow +
            soccer_vision_DW.EdgeDetection_VOL_DW[soccer_vision_B.q]) + 1] *
          soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
        soccer_vision_B.accumTwo += (int32_T)(((int64_T)soccer_vision_B.Resize
          [(soccer_vision_B.imgRow +
            soccer_vision_DW.EdgeDetection_VOR_DW[soccer_vision_B.q]) + 19081] *
          soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
        soccer_vision_B.imgIdx += (int32_T)(((int64_T)soccer_vision_B.Resize
          [(soccer_vision_B.imgRow +
            soccer_vision_DW.EdgeDetection_HOL_DW[soccer_vision_B.q]) + 1] *
          soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
        soccer_vision_B.accumFour += (int32_T)(((int64_T)soccer_vision_B.Resize
          [(soccer_vision_B.imgRow +
            soccer_vision_DW.EdgeDetection_HOR_DW[soccer_vision_B.q]) + 19081] *
          soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
      }

      soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.imgRow + 1] =
        (int32_T)(((int64_T)soccer_vision_B.accumOne * soccer_vision_B.accumOne)
                  >> 8);
      soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.imgRow + 1] =
        (int32_T)(((int64_T)soccer_vision_B.imgIdx * soccer_vision_B.imgIdx) >>
                  8);
      soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[19081 +
        soccer_vision_B.imgRow] = (int32_T)(((int64_T)soccer_vision_B.accumTwo *
        soccer_vision_B.accumTwo) >> 8);
      soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[19081 +
        soccer_vision_B.imgRow] = (int32_T)(((int64_T)soccer_vision_B.accumFour *
        soccer_vision_B.accumFour) >> 8);
    }

    soccer_vision_B.accumOne = 0;
    soccer_vision_B.accumTwo = 0;
    soccer_vision_B.imgIdx = 0;
    soccer_vision_B.accumFour = 0;
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++) {
      soccer_vision_B.accumOne += (int32_T)(((int64_T)
        soccer_vision_B.Resize[soccer_vision_DW.EdgeDetection_VOUL_DW[soccer_vision_B.q]]
        * soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
      soccer_vision_B.accumTwo += (int32_T)(((int64_T)
        soccer_vision_B.Resize[soccer_vision_DW.EdgeDetection_HOUL_DW[soccer_vision_B.q]]
        * soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
      soccer_vision_B.imgIdx += (int32_T)(((int64_T)soccer_vision_B.Resize[119 +
        soccer_vision_DW.EdgeDetection_VOLL_DW[soccer_vision_B.q]] *
        soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
      soccer_vision_B.accumFour += (int32_T)(((int64_T)soccer_vision_B.Resize
        [119 + soccer_vision_DW.EdgeDetection_HOLL_DW[soccer_vision_B.q]] *
        soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
    }

    soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[0] = (int32_T)(((int64_T)
      soccer_vision_B.accumOne * soccer_vision_B.accumOne) >> 8);
    soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[0] = (int32_T)(((int64_T)
      soccer_vision_B.accumTwo * soccer_vision_B.accumTwo) >> 8);
    soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[119] = (int32_T)(((int64_T)
      soccer_vision_B.imgIdx * soccer_vision_B.imgIdx) >> 8);
    soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[119] = (int32_T)(((int64_T)
      soccer_vision_B.accumFour * soccer_vision_B.accumFour) >> 8);
    soccer_vision_B.accumOne = 0;
    soccer_vision_B.accumTwo = 0;
    soccer_vision_B.imgIdx = 0;
    soccer_vision_B.accumFour = 0;
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++) {
      soccer_vision_B.accumOne += (int32_T)(((int64_T)soccer_vision_B.Resize
        [19080 + soccer_vision_DW.EdgeDetection_VOUR_DW[soccer_vision_B.q]] *
        soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
      soccer_vision_B.accumTwo += (int32_T)(((int64_T)soccer_vision_B.Resize
        [19080 + soccer_vision_DW.EdgeDetection_HOUR_DW[soccer_vision_B.q]] *
        soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
      soccer_vision_B.imgIdx += (int32_T)(((int64_T)soccer_vision_B.Resize[19199
        + soccer_vision_DW.EdgeDetection_VOLR_DW[soccer_vision_B.q]] *
        soccer_vision_ConstP.EdgeDetection_VC_RTP[soccer_vision_B.q]) >> 23);
      soccer_vision_B.accumFour += (int32_T)(((int64_T)soccer_vision_B.Resize
        [19199 + soccer_vision_DW.EdgeDetection_HOLR_DW[soccer_vision_B.q]] *
        soccer_vision_ConstP.EdgeDetection_HC_RTP[soccer_vision_B.q]) >> 23);
    }

    soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[19080] = (int32_T)(((int64_T)
      soccer_vision_B.accumOne * soccer_vision_B.accumOne) >> 8);
    soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[19080] = (int32_T)(((int64_T)
      soccer_vision_B.accumTwo * soccer_vision_B.accumTwo) >> 8);
    soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[19199] = (int32_T)(((int64_T)
      soccer_vision_B.imgIdx * soccer_vision_B.imgIdx) >> 8);
    soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[19199] = (int32_T)(((int64_T)
      soccer_vision_B.accumFour * soccer_vision_B.accumFour) >> 8);
    soccer_vision_B.accumTwo = 0;
    soccer_vision_B.q = 0;
    while (soccer_vision_B.q < 19200) {
      soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q] =
        soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.q];
      soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q] +=
        soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.q];
      soccer_vision_B.accumTwo += (int32_T)(((int64_T)
        soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q] *
        soccer_vision_DW.EdgeDetection_MEAN_FACTOR_DW) >> 31);
      soccer_vision_B.q++;
    }

    soccer_vision_B.accumOne = (int32_T)(((int64_T)
      soccer_vision_P.EdgeDetection_thresholdTuning * soccer_vision_B.accumTwo) >>
      8);
    for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 160;
         soccer_vision_B.imgCol++) {
      for (soccer_vision_B.imgRow = 0; soccer_vision_B.imgRow < 120;
           soccer_vision_B.imgRow++) {
        soccer_vision_B.q = soccer_vision_B.imgCol * 120 +
          soccer_vision_B.imgRow;
        b1 = true;
        b2 = true;
        b3 = true;
        b4 = true;
        if (soccer_vision_B.imgCol != 0) {
          b1 = (soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q -
                120] <=
                soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q]);
        }

        if (soccer_vision_B.imgCol != 159) {
          b2 = (soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q] >
                soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q +
                120]);
        }

        if (soccer_vision_B.imgRow != 0) {
          b3 = (soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q - 1]
                <= soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q]);
        }

        if (soccer_vision_B.imgRow != 119) {
          b4 = (soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q] >
                soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q + 1]);
        }

        soccer_vision_B.EdgeDetection[soccer_vision_B.q] =
          ((soccer_vision_DW.EdgeDetection_GRAD_SUM_DW[soccer_vision_B.q] >
            soccer_vision_B.accumOne) &&
           (((soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.q] >=
              soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.q]) &&
             b1 && b2) ||
            ((soccer_vision_DW.EdgeDetection_GH_SQUARED_DW[soccer_vision_B.q] >=
              soccer_vision_DW.EdgeDetection_GV_SQUARED_DW[soccer_vision_B.q]) &&
             b3 && b4)));
      }
    }

    // End of S-Function (svipedge): '<S10>/Edge Detection'

    // S-Function (sviphough): '<S10>/Hough Transform'
    if (soccer_vision_M->Timing.TaskCounters.TID[1] == 0) {
      MWVIP_Hough_D(&soccer_vision_B.EdgeDetection[0],
                    &soccer_vision_B.HoughTransform_o1[0],
                    &soccer_vision_ConstP.HoughTransform_SINE_TABLE_RTP[0],
                    &soccer_vision_ConstP.HoughTransform_FIRSTRHO_RTP, 120, 160,
                    399, 91);
    }

    // End of S-Function (sviphough): '<S10>/Hough Transform'

    // Gain: '<S10>/Gain'
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 399; soccer_vision_B.q++) {
      soccer_vision_B.Gain[soccer_vision_B.q] = soccer_vision_P.Gain_Gain *
        soccer_vision_B.HoughTransform_o3[soccer_vision_B.q];
    }

    // End of Gain: '<S10>/Gain'

    // S-Function (svipfindlocalmax): '<S10>/Find Local Maxima'
    soccer_vision_B.accumTwo = 0;
    b1 = false;
    soccer_vision_B.q = 0;
    while (soccer_vision_B.q < 71820) {
      soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.q] =
        soccer_vision_B.HoughTransform_o1[soccer_vision_B.q];
      soccer_vision_B.q++;
    }

    memset(&soccer_vision_B.FindLocalMaxima_o1[0], 0, 40U * sizeof(uint32_T));
    while (!b1) {
      soccer_vision_B.imgCol = 0;
      soccer_vision_B.sumdbest =
        soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[0];
      soccer_vision_B.q = 0;
      while (soccer_vision_B.q < 71820) {
        if (soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.q] >
            soccer_vision_B.sumdbest) {
          soccer_vision_B.imgCol = soccer_vision_B.q;
          soccer_vision_B.sumdbest =
            soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.q];
        }

        soccer_vision_B.q++;
      }

      soccer_vision_B.accumOne = soccer_vision_B.imgCol % 399;
      soccer_vision_B.q = soccer_vision_B.imgCol / 399;
      if (soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.imgCol]
          >= soccer_vision_P.FindLocalMaxima_threshold) {
        soccer_vision_B.FindLocalMaxima_o1[soccer_vision_B.accumTwo] = (uint32_T)
          (1 + soccer_vision_B.q);
        soccer_vision_B.FindLocalMaxima_o1[20U + soccer_vision_B.accumTwo] =
          (uint32_T)(1 + soccer_vision_B.accumOne);
        soccer_vision_B.accumTwo++;
        if (soccer_vision_B.accumTwo == 20) {
          b1 = true;
        }

        soccer_vision_B.imgIdx = soccer_vision_B.accumOne - 100;
        if (!(soccer_vision_B.imgIdx > 0)) {
          soccer_vision_B.imgIdx = 0;
        }

        soccer_vision_B.accumFour = soccer_vision_B.accumOne + 100;
        if (!(soccer_vision_B.accumFour < 398)) {
          soccer_vision_B.accumFour = 398;
        }

        soccer_vision_B.imgCol = soccer_vision_B.q - 25;
        soccer_vision_B.imgRow = soccer_vision_B.q + 25;
        if (!((soccer_vision_B.imgCol < 0) || (soccer_vision_B.imgRow > 179))) {
          while (soccer_vision_B.imgCol <= soccer_vision_B.imgRow) {
            soccer_vision_B.imgIdx_d = soccer_vision_B.imgCol * 399;
            soccer_vision_B.q = soccer_vision_B.imgIdx;
            while (soccer_vision_B.q <= soccer_vision_B.accumFour) {
              soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.q
                + soccer_vision_B.imgIdx_d] = 0.0;
              soccer_vision_B.q++;
            }

            soccer_vision_B.imgCol++;
          }
        } else {
          if (soccer_vision_B.imgCol < 0) {
            soccer_vision_B.accumOne = soccer_vision_B.imgCol;
            while (soccer_vision_B.accumOne <= soccer_vision_B.imgRow) {
              if (soccer_vision_B.accumOne < 0) {
                soccer_vision_B.imgIdx_d = (soccer_vision_B.accumOne + 180) *
                  399 + 398;
                soccer_vision_B.q = soccer_vision_B.imgIdx;
                while (soccer_vision_B.q <= soccer_vision_B.accumFour) {
                  soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.imgIdx_d
                    - soccer_vision_B.q] = 0.0;
                  soccer_vision_B.q++;
                }
              } else {
                soccer_vision_B.imgIdx_d = soccer_vision_B.accumOne * 399;
                soccer_vision_B.q = soccer_vision_B.imgIdx;
                while (soccer_vision_B.q <= soccer_vision_B.accumFour) {
                  soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.q
                    + soccer_vision_B.imgIdx_d] = 0.0;
                  soccer_vision_B.q++;
                }
              }

              soccer_vision_B.accumOne++;
            }
          }

          if (soccer_vision_B.imgRow > 179) {
            soccer_vision_B.accumOne = soccer_vision_B.imgCol;
            while (soccer_vision_B.accumOne <= soccer_vision_B.imgRow) {
              if (soccer_vision_B.accumOne > 179) {
                soccer_vision_B.imgIdx_d = (soccer_vision_B.accumOne - 180) *
                  399 + 398;
                soccer_vision_B.q = soccer_vision_B.imgIdx;
                while (soccer_vision_B.q <= soccer_vision_B.accumFour) {
                  soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.imgIdx_d
                    - soccer_vision_B.q] = 0.0;
                  soccer_vision_B.q++;
                }
              } else {
                soccer_vision_B.imgIdx_d = soccer_vision_B.accumOne * 399;
                soccer_vision_B.q = soccer_vision_B.imgIdx;
                while (soccer_vision_B.q <= soccer_vision_B.accumFour) {
                  soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[soccer_vision_B.q
                    + soccer_vision_B.imgIdx_d] = 0.0;
                  soccer_vision_B.q++;
                }
              }

              soccer_vision_B.accumOne++;
            }
          }
        }
      } else {
        b1 = true;
      }
    }

    // End of S-Function (svipfindlocalmax): '<S10>/Find Local Maxima'
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 20; soccer_vision_B.q++) {
      // S-Function (sdspperm2): '<S10>/Variable Selector' incorporates:
      //   Selector: '<S10>/Selector1'

      soccer_vision_B.imgCol = (int32_T)
        soccer_vision_B.FindLocalMaxima_o1[soccer_vision_B.q] - 1;
      if (soccer_vision_B.imgCol < 0) {
        soccer_vision_B.imgCol = 0;
      } else {
        if (soccer_vision_B.imgCol >= 180) {
          soccer_vision_B.imgCol = 179;
        }
      }

      soccer_vision_B.rho[soccer_vision_B.q] =
        soccer_vision_B.HoughTransform_o2[soccer_vision_B.imgCol];

      // End of S-Function (sdspperm2): '<S10>/Variable Selector'

      // S-Function (sdspperm2): '<S10>/Variable Selector1' incorporates:
      //   Selector: '<S10>/Selector'

      soccer_vision_B.imgCol = (int32_T)soccer_vision_B.FindLocalMaxima_o1[20 +
        soccer_vision_B.q] - 1;
      if (soccer_vision_B.imgCol < 0) {
        soccer_vision_B.imgCol = 0;
      } else {
        if (soccer_vision_B.imgCol >= 399) {
          soccer_vision_B.imgCol = 398;
        }
      }

      soccer_vision_B.VariableSelector1[soccer_vision_B.q] =
        soccer_vision_B.Gain[soccer_vision_B.imgCol];

      // End of S-Function (sdspperm2): '<S10>/Variable Selector1'

      // MATLAB Function: '<S2>/Filter'
      soccer_vision_B.lines[soccer_vision_B.q] =
        soccer_vision_B.rho[soccer_vision_B.q];
      soccer_vision_B.lines[soccer_vision_B.q + 20] =
        soccer_vision_B.VariableSelector1[soccer_vision_B.q] *
        soccer_vision_P.scalerho;
      if (soccer_vision_B.lines[20 + soccer_vision_B.q] < 0.0) {
        soccer_vision_B.lines[soccer_vision_B.q] += 3.1415926535897931;
        soccer_vision_B.lines[20 + soccer_vision_B.q] = -soccer_vision_B.lines
          [20 + soccer_vision_B.q];
      }

      if (soccer_vision_B.lines[soccer_vision_B.q] > 3.1415926535897931) {
        soccer_vision_B.lines[soccer_vision_B.q] -= 6.2831853071795862;
      }
    }

    // MATLAB Function: '<S2>/Filter' incorporates:
    //   Constant: '<S2>/Constant1'
    //   MATLAB Function: '<S2>/Cover Edge'

    soccer_vision_B.sumdbest = 1.0E+6;
    soccer_vision_B.imgCol = -1;
    soccer_vision_B.accumTwo = 1;
    soccer_vision_B.linesFilteredBest_data[0] = 0.0;
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 4; soccer_vision_B.q++) {
      soccer_vision_kmeans(soccer_vision_B.lines, 3.0 + (real_T)
                           soccer_vision_B.q, soccer_vision_B.unusedU0,
                           soccer_vision_B.C_data, soccer_vision_B.C_size,
                           soccer_vision_B.sumd_data, &soccer_vision_B.sumd_size);
      soccer_vision_B.imgIdx = 0;
      soccer_vision_B.accumFour = 0;
      while (soccer_vision_B.accumFour <= soccer_vision_B.q + 2) {
        soccer_vision_B.accumOne = 0;
        while (soccer_vision_B.accumOne <= soccer_vision_B.q + 2) {
          if (!(1 + soccer_vision_B.accumFour <= 1 + soccer_vision_B.accumOne))
          {
            soccer_vision_B.scale = 3.3121686421112381E-170;
            soccer_vision_B.absxk = fabs
              (soccer_vision_B.C_data[soccer_vision_B.accumFour] -
               soccer_vision_B.C_data[soccer_vision_B.accumOne]);
            if (soccer_vision_B.absxk > 3.3121686421112381E-170) {
              soccer_vision_B.c_y = 1.0;
              soccer_vision_B.scale = soccer_vision_B.absxk;
            } else {
              soccer_vision_B.t = soccer_vision_B.absxk /
                3.3121686421112381E-170;
              soccer_vision_B.c_y = soccer_vision_B.t * soccer_vision_B.t;
            }

            soccer_vision_B.absxk = fabs
              (soccer_vision_B.C_data[soccer_vision_B.accumFour +
               soccer_vision_B.C_size[0]] -
               soccer_vision_B.C_data[soccer_vision_B.accumOne +
               soccer_vision_B.C_size[0]]);
            if (soccer_vision_B.absxk > soccer_vision_B.scale) {
              soccer_vision_B.t = soccer_vision_B.scale / soccer_vision_B.absxk;
              soccer_vision_B.c_y = soccer_vision_B.c_y * soccer_vision_B.t *
                soccer_vision_B.t + 1.0;
              soccer_vision_B.scale = soccer_vision_B.absxk;
            } else {
              soccer_vision_B.t = soccer_vision_B.absxk / soccer_vision_B.scale;
              soccer_vision_B.c_y += soccer_vision_B.t * soccer_vision_B.t;
            }

            soccer_vision_B.c_y = soccer_vision_B.scale * sqrt
              (soccer_vision_B.c_y);
            if (soccer_vision_B.c_y < soccer_vision_P.clusterProximityThreshold)
            {
              soccer_vision_B.imgIdx = 1;
            }
          }

          soccer_vision_B.accumOne++;
        }

        soccer_vision_B.accumFour++;
      }

      if (soccer_vision_B.imgIdx != 1) {
        soccer_vision_B.scale = soccer_vision_B.sumd_data[0];
        soccer_vision_B.accumOne = 2;
        while (soccer_vision_B.accumOne <= soccer_vision_B.sumd_size) {
          soccer_vision_B.scale +=
            soccer_vision_B.sumd_data[soccer_vision_B.accumOne - 1];
          soccer_vision_B.accumOne++;
        }

        if (soccer_vision_B.scale / (real_T)soccer_vision_B.sumd_size <
            soccer_vision_B.sumdbest) {
          soccer_vision_B.sumdbest = soccer_vision_B.sumd_data[0];
          soccer_vision_B.imgCol = 2;
          while (soccer_vision_B.imgCol <= soccer_vision_B.sumd_size) {
            soccer_vision_B.sumdbest +=
              soccer_vision_B.sumd_data[soccer_vision_B.imgCol - 1];
            soccer_vision_B.imgCol++;
          }

          soccer_vision_B.sumdbest /= (real_T)soccer_vision_B.sumd_size;
          soccer_vision_B.imgCol = soccer_vision_B.q + 2;
          soccer_vision_B.accumTwo = soccer_vision_B.C_size[0];
          soccer_vision_B.accumOne = soccer_vision_B.C_size[0] *
            soccer_vision_B.C_size[1] - 1;
          if (0 <= soccer_vision_B.accumOne) {
            memcpy(&soccer_vision_B.linesFilteredBest_data[0],
                   &soccer_vision_B.C_data[0], (soccer_vision_B.accumOne + 1) *
                   sizeof(real_T));
          }
        }
      }
    }

    for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++) {
      soccer_vision_B.thetaField[soccer_vision_B.q] = 0.0;
      soccer_vision_B.rhoField[soccer_vision_B.q] = 0.0;
      soccer_vision_B.thetaNet[soccer_vision_B.q] = 0.0;
      soccer_vision_B.rhoNet[soccer_vision_B.q] = 0.0;
    }

    soccer_vision_B.sumdbest = 0.0;
    soccer_vision_B.scale = 0.0;
    soccer_vision_B.q = 0;
    while (soccer_vision_B.q <= soccer_vision_B.imgCol) {
      if ((soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q] <
           soccer_vision_P.netAngleThreshold + soccer_vision_P.Constant1_Value) &&
          (soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q] >
           -soccer_vision_P.netAngleThreshold + soccer_vision_P.Constant1_Value))
      {
        soccer_vision_B.scale++;
        soccer_vision_B.accumOne = (int32_T)soccer_vision_B.scale - 1;
        soccer_vision_B.thetaNet[soccer_vision_B.accumOne] =
          soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q];
        soccer_vision_B.rhoNet[soccer_vision_B.accumOne] =
          soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q +
          soccer_vision_B.accumTwo] / soccer_vision_P.scalerho;
      } else {
        if ((soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q +
             soccer_vision_B.accumTwo] > soccer_vision_B.totalLineCount *
             soccer_vision_P.scalerho) ||
            ((soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q] < 0.0) &&
             (soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q] >
              -1.0471975511965976))) {
          soccer_vision_B.sumdbest++;
          soccer_vision_B.accumOne = (int32_T)soccer_vision_B.sumdbest - 1;
          soccer_vision_B.thetaField[soccer_vision_B.accumOne] =
            soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q];
          soccer_vision_B.rhoField[soccer_vision_B.accumOne] =
            soccer_vision_B.linesFilteredBest_data[soccer_vision_B.q +
            soccer_vision_B.accumTwo] / soccer_vision_P.scalerho;
        }
      }

      soccer_vision_B.q++;
    }
  }

  // MATLAB Function: '<S12>/Label Net Lines' incorporates:
  //   Memory: '<S12>/Memory'

  memset(&soccer_vision_B.netLinesNew[0], 0, sizeof(real_T) << 3U);
  y[0] = false;
  y[1] = false;
  soccer_vision_B.q = 0;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.q + 1 <= 4)) {
    if (!((soccer_vision_DW.Memory_PreviousInput[soccer_vision_B.q] == 0.0) ||
          rtIsNaN(soccer_vision_DW.Memory_PreviousInput[soccer_vision_B.q]))) {
      y[0] = true;
      exitg1 = true;
    } else {
      soccer_vision_B.q++;
    }
  }

  soccer_vision_B.q = 4;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.q + 1 <= 8)) {
    if (!((soccer_vision_DW.Memory_PreviousInput[soccer_vision_B.q] == 0.0) ||
          rtIsNaN(soccer_vision_DW.Memory_PreviousInput[soccer_vision_B.q]))) {
      y[1] = true;
      exitg1 = true;
    } else {
      soccer_vision_B.q++;
    }
  }

  b1 = true;
  soccer_vision_B.q = 1;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.q < 3)) {
    if (!y[soccer_vision_B.q - 1]) {
      b1 = false;
      exitg1 = true;
    } else {
      soccer_vision_B.q++;
    }
  }

  if (b1) {
    memcpy(&soccer_vision_B.netLinesNew[0],
           &soccer_vision_DW.Memory_PreviousInput[0], sizeof(real_T) << 3U);
  }

  soccer_vision_B.totalLineCount = 0.0;
  for (soccer_vision_B.q = 0; soccer_vision_B.q < 6; soccer_vision_B.q++) {
    if (soccer_vision_B.thetaNet[soccer_vision_B.q] == 0.0) {
      if (!(soccer_vision_B.rhoNet[soccer_vision_B.q] == 0.0)) {
        soccer_vision_B.totalLineCount++;
      }
    } else {
      soccer_vision_B.totalLineCount++;
    }
  }

  if (soccer_vision_B.totalLineCount == 2.0) {
    if (soccer_vision_B.rhoNet[1] < soccer_vision_B.rhoNet[0]) {
      soccer_vision_B.netLinesNew[0] = soccer_vision_B.thetaNet[0];
      soccer_vision_B.netLinesNew[4] = soccer_vision_B.rhoNet[0];
      soccer_vision_B.netLinesNew[1] = soccer_vision_B.thetaNet[1];
      soccer_vision_B.netLinesNew[5] = soccer_vision_B.rhoNet[1];
    } else {
      soccer_vision_B.netLinesNew[0] = soccer_vision_B.thetaNet[1];
      soccer_vision_B.netLinesNew[4] = soccer_vision_B.rhoNet[1];
      soccer_vision_B.netLinesNew[1] = soccer_vision_B.thetaNet[0];
      soccer_vision_B.netLinesNew[5] = soccer_vision_B.rhoNet[0];
    }
  }

  if (soccer_vision_B.totalLineCount == 1.0) {
    if ((soccer_vision_B.netLinesNew[0] != 0.0) && (soccer_vision_B.netLinesNew
         [1] != 0.0) && (soccer_vision_B.netLinesNew[4] != 0.0) &&
        (soccer_vision_B.netLinesNew[5] != 0.0)) {
      soccer_vision_B.totalLineCount = soccer_vision_B.rhoNet[0] *
        soccer_vision_scalerho;
      soccer_vision_B.dv0[0] = soccer_vision_B.thetaNet[0] -
        soccer_vision_B.netLinesNew[0];
      soccer_vision_B.dv0[1] = soccer_vision_B.totalLineCount -
        soccer_vision_B.netLinesNew[4] * soccer_vision_scalerho;
      soccer_vision_B.dv1[0] = soccer_vision_B.thetaNet[0] -
        soccer_vision_B.netLinesNew[1];
      soccer_vision_B.dv1[1] = soccer_vision_B.totalLineCount -
        soccer_vision_B.netLinesNew[5] * soccer_vision_scalerho;
      if (soccer_vision_norm(soccer_vision_B.dv0) < soccer_vision_norm
          (soccer_vision_B.dv1)) {
        soccer_vision_B.netLinesNew[0] = soccer_vision_B.thetaNet[0];
        soccer_vision_B.netLinesNew[4] = soccer_vision_B.rhoNet[0];
      } else {
        soccer_vision_B.netLinesNew[1] = soccer_vision_B.thetaNet[0];
        soccer_vision_B.netLinesNew[5] = soccer_vision_B.rhoNet[0];
      }
    } else {
      soccer_vision_B.netLinesNew[0] = soccer_vision_B.thetaNet[0];
      soccer_vision_B.netLinesNew[4] = soccer_vision_B.rhoNet[0];
    }
  }

  // End of MATLAB Function: '<S12>/Label Net Lines'

  // MATLAB Function: '<S12>/Label Field Lines' incorporates:
  //   Memory: '<S12>/Memory1'

  memset(&soccer_vision_B.fieldLinesNew[0], 0, 10U * sizeof(real_T));
  y[0] = false;
  y[1] = false;
  soccer_vision_B.q = 0;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.q + 1 <= 5)) {
    if (!((soccer_vision_DW.Memory1_PreviousInput[soccer_vision_B.q] == 0.0) ||
          rtIsNaN(soccer_vision_DW.Memory1_PreviousInput[soccer_vision_B.q]))) {
      y[0] = true;
      exitg1 = true;
    } else {
      soccer_vision_B.q++;
    }
  }

  soccer_vision_B.q = 5;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.q + 1 <= 10)) {
    if (!((soccer_vision_DW.Memory1_PreviousInput[soccer_vision_B.q] == 0.0) ||
          rtIsNaN(soccer_vision_DW.Memory1_PreviousInput[soccer_vision_B.q]))) {
      y[1] = true;
      exitg1 = true;
    } else {
      soccer_vision_B.q++;
    }
  }

  b1 = true;
  soccer_vision_B.q = 1;
  exitg1 = false;
  while ((!exitg1) && (soccer_vision_B.q < 3)) {
    if (!y[soccer_vision_B.q - 1]) {
      b1 = false;
      exitg1 = true;
    } else {
      soccer_vision_B.q++;
    }
  }

  if (b1) {
    memcpy(&soccer_vision_B.fieldLinesNew[0],
           &soccer_vision_DW.Memory1_PreviousInput[0], 10U * sizeof(real_T));
  }

  for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 5;
       soccer_vision_B.imgCol++) {
    soccer_vision_B.x[soccer_vision_B.imgCol] =
      soccer_vision_B.thetaField[soccer_vision_B.imgCol];
  }

  soccer_vision_sort(soccer_vision_B.x, soccer_vision_B.iidx);
  for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 5;
       soccer_vision_B.imgCol++) {
    soccer_vision_B.fieldLinesNew[soccer_vision_B.imgCol] =
      soccer_vision_B.x[soccer_vision_B.imgCol];
    soccer_vision_B.fieldLinesNew[5 + soccer_vision_B.imgCol] =
      soccer_vision_B.rhoField[soccer_vision_B.iidx[soccer_vision_B.imgCol] - 1];
  }

  // End of MATLAB Function: '<S12>/Label Field Lines'

  // Concatenate: '<S12>/Matrix Concatenate'
  for (soccer_vision_B.imgCol = 0; soccer_vision_B.imgCol < 2;
       soccer_vision_B.imgCol++) {
    soccer_vision_B.MatrixConcatenate[9 * soccer_vision_B.imgCol] =
      soccer_vision_B.netLinesNew[soccer_vision_B.imgCol << 2];
    soccer_vision_B.MatrixConcatenate[1 + 9 * soccer_vision_B.imgCol] =
      soccer_vision_B.netLinesNew[(soccer_vision_B.imgCol << 2) + 1];
    soccer_vision_B.MatrixConcatenate[2 + 9 * soccer_vision_B.imgCol] =
      soccer_vision_B.netLinesNew[(soccer_vision_B.imgCol << 2) + 2];
    soccer_vision_B.MatrixConcatenate[3 + 9 * soccer_vision_B.imgCol] =
      soccer_vision_B.netLinesNew[(soccer_vision_B.imgCol << 2) + 3];
    for (soccer_vision_B.q = 0; soccer_vision_B.q < 5; soccer_vision_B.q++) {
      soccer_vision_B.MatrixConcatenate[(soccer_vision_B.q + 9 *
        soccer_vision_B.imgCol) + 4] = soccer_vision_B.fieldLinesNew[5 *
        soccer_vision_B.imgCol + soccer_vision_B.q];
    }
  }

  // End of Concatenate: '<S12>/Matrix Concatenate'

  // S-Function (sviphoughlines): '<S2>/Hough Lines' incorporates:
  //   Selector: '<S2>/Selector'
  //   Selector: '<S2>/Selector1'

  for (soccer_vision_B.q = 0; soccer_vision_B.q < 9; soccer_vision_B.q++) {
    soccer_vision_B.imgCol = 0;
    soccer_vision_B.totalLineCount = (soccer_vision_B.MatrixConcatenate[9 +
      soccer_vision_B.q] + 2.2204460492503131E-16) / (cos
      (soccer_vision_B.MatrixConcatenate[soccer_vision_B.q]) +
      2.2204460492503131E-16);

    // part-1: top horizontal axis
    soccer_vision_B.sumdbest = floor(soccer_vision_B.totalLineCount + 0.5);
    if ((soccer_vision_B.sumdbest >= 0.0) && (soccer_vision_B.sumdbest <= 639.0))
    {
      soccer_vision_B.tmpOutRC[0U] = 0;
      if (soccer_vision_B.sumdbest >= 0.5) {
        soccer_vision_B.tmpOutRC[1U] = (int32_T)floor(soccer_vision_B.sumdbest +
          0.5);
      } else {
        soccer_vision_B.tmpOutRC[1U] = (int32_T)(soccer_vision_B.sumdbest * 0.0);
      }

      soccer_vision_B.imgCol = 1;
    }

    soccer_vision_B.scale = (soccer_vision_B.MatrixConcatenate[9 +
      soccer_vision_B.q] + 2.2204460492503131E-16) / (sin
      (soccer_vision_B.MatrixConcatenate[soccer_vision_B.q]) +
      2.2204460492503131E-16);

    // part-2: left vertical axis
    soccer_vision_B.sumdbest = floor(soccer_vision_B.scale + 0.5);
    if ((soccer_vision_B.sumdbest >= 0.0) && (soccer_vision_B.sumdbest <= 479.0))
    {
      if (soccer_vision_B.sumdbest >= 0.5) {
        soccer_vision_B.tmpOutRC[soccer_vision_B.imgCol << 1] = (int32_T)floor
          (soccer_vision_B.sumdbest + 0.5);
      } else {
        soccer_vision_B.tmpOutRC[soccer_vision_B.imgCol << 1] = (int32_T)
          (soccer_vision_B.sumdbest * 0.0);
      }

      soccer_vision_B.tmpOutRC[1 + (soccer_vision_B.imgCol << 1)] = 0;
      soccer_vision_B.imgCol++;
    }

    // part-3: Right vertical axis
    if (soccer_vision_B.imgCol < 2) {
      soccer_vision_B.sumdbest = floor((soccer_vision_B.totalLineCount - 639.0) *
        (soccer_vision_B.scale / soccer_vision_B.totalLineCount) + 0.5);
      if ((soccer_vision_B.sumdbest >= 0.0) && (soccer_vision_B.sumdbest <=
           479.0)) {
        if (soccer_vision_B.sumdbest >= 0.5) {
          soccer_vision_B.tmpOutRC[soccer_vision_B.imgCol << 1] = (int32_T)floor
            (soccer_vision_B.sumdbest + 0.5);
        } else {
          soccer_vision_B.tmpOutRC[soccer_vision_B.imgCol << 1] = (int32_T)
            (soccer_vision_B.sumdbest * 0.0);
        }

        soccer_vision_B.tmpOutRC[1 + (soccer_vision_B.imgCol << 1)] = 639;
        soccer_vision_B.imgCol++;
      }
    }

    // part-4: bottom horizontal axis
    if (soccer_vision_B.imgCol < 2) {
      soccer_vision_B.sumdbest = floor((soccer_vision_B.totalLineCount -
        soccer_vision_B.totalLineCount / soccer_vision_B.scale * 479.0) + 0.5);
      if ((soccer_vision_B.sumdbest >= 0.0) && (soccer_vision_B.sumdbest <=
           639.0)) {
        soccer_vision_B.tmpOutRC[soccer_vision_B.imgCol << 1] = 479;
        if (soccer_vision_B.sumdbest >= 0.5) {
          soccer_vision_B.tmpOutRC[1 + (soccer_vision_B.imgCol << 1)] = (int32_T)
            floor(soccer_vision_B.sumdbest + 0.5);
        } else {
          soccer_vision_B.tmpOutRC[1 + (soccer_vision_B.imgCol << 1)] = (int32_T)
            (soccer_vision_B.sumdbest * 0.0);
        }

        soccer_vision_B.imgCol++;
      }
    }

    if (soccer_vision_B.imgCol < 2) {
      soccer_vision_B.tmpOutRC[0U] = -1;
      soccer_vision_B.tmpOutRC[1U] = -1;
      soccer_vision_B.tmpOutRC[2U] = -1;
      soccer_vision_B.tmpOutRC[3U] = -1;
    }

    soccer_vision_B.HoughLines[soccer_vision_B.q] = soccer_vision_B.tmpOutRC[1]
      + 1;
    soccer_vision_B.HoughLines[soccer_vision_B.q + 9] =
      soccer_vision_B.tmpOutRC[0] + 1;
    if (soccer_vision_B.tmpOutRC[3] > 2147483646) {
      soccer_vision_B.HoughLines[soccer_vision_B.q + 18] = MAX_int32_T;
    } else {
      soccer_vision_B.HoughLines[soccer_vision_B.q + 18] =
        soccer_vision_B.tmpOutRC[3] + 1;
    }

    if (soccer_vision_B.tmpOutRC[2] > 2147483646) {
      soccer_vision_B.HoughLines[soccer_vision_B.q + 27] = MAX_int32_T;
    } else {
      soccer_vision_B.HoughLines[soccer_vision_B.q + 27] =
        soccer_vision_B.tmpOutRC[2] + 1;
    }
  }

  // End of S-Function (sviphoughlines): '<S2>/Hough Lines'

  // MATLABSystem: '<S2>/MATLAB System' incorporates:
  //   Constant: '<S2>/Constant6'

  //  Implement algorithm. Calculate y as a function of input u and
  //  discrete states.
  lines = &soccer_vision_B.HoughLines[0];
  for (soccer_vision_B.accumFour = 0; soccer_vision_B.accumFour < 9;
       soccer_vision_B.accumFour++) {
    if ((lines[soccer_vision_B.accumFour] == 1) && (lines[9 +
         soccer_vision_B.accumFour] == 1) && (lines[18 +
         soccer_vision_B.accumFour] == 1) && (lines[27 +
         soccer_vision_B.accumFour] == 2)) {
    } else {
      // POINT2F Summary of this class goes here
      //    Detailed explanation goes here
      // LINE Construct an instance of this class
      //    Detailed explanation goes here
      soccer_vision_B.totalLineCount = ((real_T)lines[27 +
        soccer_vision_B.accumFour] - (real_T)lines[9 + soccer_vision_B.accumFour])
        / ((real_T)lines[18 + soccer_vision_B.accumFour] - (real_T)
           lines[soccer_vision_B.accumFour]);
      if (soccer_vision_B.totalLineCount == (rtMinusInf)) {
        //  Kinda sketchy
        soccer_vision_B.totalLineCount = -1.0E+9;
      }

      if (soccer_vision_B.totalLineCount == (rtInf)) {
        soccer_vision_B.totalLineCount = 1.0E+9;
      }

      soccer_vision_B.lobj_7.rho = fabs((real_T)lines[9 +
        soccer_vision_B.accumFour] + soccer_vision_B.totalLineCount * (real_T)
        lines[soccer_vision_B.accumFour]) / sqrt(soccer_vision_B.totalLineCount *
        soccer_vision_B.totalLineCount + 1.0);
      soccer_vision_B.lobj_7.theta = rt_atan2d_snf((real_T)lines[27 +
        soccer_vision_B.accumFour] - (real_T)lines[9 + soccer_vision_B.accumFour],
        (real_T)lines[18 + soccer_vision_B.accumFour] - (real_T)
        lines[soccer_vision_B.accumFour]) - 1.5707963267948966;
      if (soccer_vision_B.lobj_7.rho < 0.0) {
        soccer_vision_B.lobj_7.rho = -soccer_vision_B.lobj_7.rho;
        soccer_vision_B.lobj_7.theta += 3.1415926535897931;
      }

      //  Check if the line has crossed the horizon, clip it at the horizon
      soccer_vision_B.totalLineCount = 0.0 * soccer_vision_P.Constant6_Value;
      soccer_vision_B.horizonLine.rho = 240.0 - soccer_vision_B.totalLineCount;
      soccer_vision_B.horizonLine.theta = 1.5707963267948966;
      if (soccer_vision_B.horizonLine.rho < 0.0) {
        soccer_vision_B.horizonLine.rho = -soccer_vision_B.horizonLine.rho;
        soccer_vision_B.horizonLine.theta += 3.1415926535897931;
      }

      if ((lines[9 + soccer_vision_B.accumFour] < 240.0 -
           soccer_vision_B.totalLineCount) && (lines[27 +
           soccer_vision_B.accumFour] < 240.0 - soccer_vision_B.totalLineCount))
      {
      } else {
        if (lines[9 + soccer_vision_B.accumFour] < 240.0 -
            soccer_vision_B.totalLineCount) {
          soccer_vision_Line2f_screenIntersection(&soccer_vision_B.horizonLine,
            &soccer_vision_B.lobj_7, &soccer_vision_B.b_inter);
        }

        if (lines[27 + soccer_vision_B.accumFour] < 240.0 -
            soccer_vision_B.totalLineCount) {
          soccer_vision_Line2f_screenIntersection(&soccer_vision_B.horizonLine,
            &soccer_vision_B.lobj_7, &soccer_vision_B.inter);
        }

        //  Wrap into a big ass matrix
        // LINE Summary of this class goes here
        //    Detailed explanation goes here
        // LINE Construct an instance of this class
      }
    }
  }

  // End of MATLABSystem: '<S2>/MATLAB System'

  // Update for Memory: '<S12>/Memory'
  memcpy(&soccer_vision_DW.Memory_PreviousInput[0],
         &soccer_vision_B.netLinesNew[0], sizeof(real_T) << 3U);

  // Update for Memory: '<S12>/Memory1'
  memcpy(&soccer_vision_DW.Memory1_PreviousInput[0],
         &soccer_vision_B.fieldLinesNew[0], 10U * sizeof(real_T));

  // External mode
  rtExtModeUploadCheckTrigger(2);

  {                                    // Sample time: [0.01s, 0.0s]
    rtExtModeUpload(0, soccer_vision_M->Timing.taskTime0);
  }

  if (soccer_vision_M->Timing.TaskCounters.TID[1] == 0) {// Sample time: [0.1s, 0.0s] 
    rtExtModeUpload(1, ((soccer_vision_M->Timing.clockTick1) * 0.1));
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.01s, 0.0s]
    if ((rtmGetTFinal(soccer_vision_M)!=-1) &&
        !((rtmGetTFinal(soccer_vision_M)-soccer_vision_M->Timing.taskTime0) >
          soccer_vision_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(soccer_vision_M, "Simulation finished");
    }

    if (rtmGetStopRequested(soccer_vision_M)) {
      rtmSetErrorStatus(soccer_vision_M, "Simulation finished");
    }
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  soccer_vision_M->Timing.taskTime0 =
    (++soccer_vision_M->Timing.clockTick0) * soccer_vision_M->Timing.stepSize0;
  if (soccer_vision_M->Timing.TaskCounters.TID[1] == 0) {
    // Update absolute timer for sample time: [0.1s, 0.0s]
    // The "clockTick1" counts the number of times the code of this task has
    //  been executed. The resolution of this integer timer is 0.1, which is the step size
    //  of the task. Size of "clockTick1" ensures timer will not overflow during the
    //  application lifespan selected.

    soccer_vision_M->Timing.clockTick1++;
  }

  rate_scheduler();
}

// Model initialize function
void soccer_vision_initialize(void)
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // initialize real-time model
  (void) memset((void *)soccer_vision_M, 0,
                sizeof(RT_MODEL_soccer_vision_T));
  rtmSetTFinal(soccer_vision_M, -1);
  soccer_vision_M->Timing.stepSize0 = 0.01;

  // External mode info
  soccer_vision_M->Sizes.checksums[0] = (3812702237U);
  soccer_vision_M->Sizes.checksums[1] = (2286541348U);
  soccer_vision_M->Sizes.checksums[2] = (1009730477U);
  soccer_vision_M->Sizes.checksums[3] = (3966864686U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[10];
    soccer_vision_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = (sysRanDType *)&soccer_vision_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(soccer_vision_M->extModeInfo,
      &soccer_vision_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(soccer_vision_M->extModeInfo,
                        soccer_vision_M->Sizes.checksums);
    rteiSetTPtr(soccer_vision_M->extModeInfo, rtmGetTPtr(soccer_vision_M));
  }

  // block I/O
  (void) memset(((void *) &soccer_vision_B), 0,
                sizeof(B_soccer_vision_T));

  {
    soccer_vision_B.HoughTransform_o2[0] = -1.5707963267948966;
    soccer_vision_B.HoughTransform_o2[1] = -1.5533430342749532;
    soccer_vision_B.HoughTransform_o2[2] = -1.53588974175501;
    soccer_vision_B.HoughTransform_o2[3] = -1.5184364492350666;
    soccer_vision_B.HoughTransform_o2[4] = -1.5009831567151235;
    soccer_vision_B.HoughTransform_o2[5] = -1.4835298641951802;
    soccer_vision_B.HoughTransform_o2[6] = -1.4660765716752369;
    soccer_vision_B.HoughTransform_o2[7] = -1.4486232791552935;
    soccer_vision_B.HoughTransform_o2[8] = -1.4311699866353502;
    soccer_vision_B.HoughTransform_o2[9] = -1.4137166941154069;
    soccer_vision_B.HoughTransform_o2[10] = -1.3962634015954636;
    soccer_vision_B.HoughTransform_o2[11] = -1.3788101090755203;
    soccer_vision_B.HoughTransform_o2[12] = -1.3613568165555769;
    soccer_vision_B.HoughTransform_o2[13] = -1.3439035240356338;
    soccer_vision_B.HoughTransform_o2[14] = -1.3264502315156905;
    soccer_vision_B.HoughTransform_o2[15] = -1.3089969389957472;
    soccer_vision_B.HoughTransform_o2[16] = -1.2915436464758039;
    soccer_vision_B.HoughTransform_o2[17] = -1.2740903539558606;
    soccer_vision_B.HoughTransform_o2[18] = -1.2566370614359172;
    soccer_vision_B.HoughTransform_o2[19] = -1.2391837689159739;
    soccer_vision_B.HoughTransform_o2[20] = -1.2217304763960306;
    soccer_vision_B.HoughTransform_o2[21] = -1.2042771838760873;
    soccer_vision_B.HoughTransform_o2[22] = -1.1868238913561442;
    soccer_vision_B.HoughTransform_o2[23] = -1.1693705988362009;
    soccer_vision_B.HoughTransform_o2[24] = -1.1519173063162575;
    soccer_vision_B.HoughTransform_o2[25] = -1.1344640137963142;
    soccer_vision_B.HoughTransform_o2[26] = -1.1170107212763709;
    soccer_vision_B.HoughTransform_o2[27] = -1.0995574287564276;
    soccer_vision_B.HoughTransform_o2[28] = -1.0821041362364843;
    soccer_vision_B.HoughTransform_o2[29] = -1.064650843716541;
    soccer_vision_B.HoughTransform_o2[30] = -1.0471975511965976;
    soccer_vision_B.HoughTransform_o2[31] = -1.0297442586766545;
    soccer_vision_B.HoughTransform_o2[32] = -1.0122909661567112;
    soccer_vision_B.HoughTransform_o2[33] = -0.99483767363676789;
    soccer_vision_B.HoughTransform_o2[34] = -0.97738438111682457;
    soccer_vision_B.HoughTransform_o2[35] = -0.95993108859688125;
    soccer_vision_B.HoughTransform_o2[36] = -0.94247779607693793;
    soccer_vision_B.HoughTransform_o2[37] = -0.92502450355699462;
    soccer_vision_B.HoughTransform_o2[38] = -0.90757121103705141;
    soccer_vision_B.HoughTransform_o2[39] = -0.89011791851710809;
    soccer_vision_B.HoughTransform_o2[40] = -0.87266462599716477;
    soccer_vision_B.HoughTransform_o2[41] = -0.85521133347722145;
    soccer_vision_B.HoughTransform_o2[42] = -0.83775804095727824;
    soccer_vision_B.HoughTransform_o2[43] = -0.82030474843733492;
    soccer_vision_B.HoughTransform_o2[44] = -0.8028514559173916;
    soccer_vision_B.HoughTransform_o2[45] = -0.78539816339744828;
    soccer_vision_B.HoughTransform_o2[46] = -0.767944870877505;
    soccer_vision_B.HoughTransform_o2[47] = -0.75049157835756175;
    soccer_vision_B.HoughTransform_o2[48] = -0.73303828583761843;
    soccer_vision_B.HoughTransform_o2[49] = -0.71558499331767511;
    soccer_vision_B.HoughTransform_o2[50] = -0.69813170079773179;
    soccer_vision_B.HoughTransform_o2[51] = -0.68067840827778847;
    soccer_vision_B.HoughTransform_o2[52] = -0.66322511575784526;
    soccer_vision_B.HoughTransform_o2[53] = -0.64577182323790194;
    soccer_vision_B.HoughTransform_o2[54] = -0.62831853071795862;
    soccer_vision_B.HoughTransform_o2[55] = -0.6108652381980153;
    soccer_vision_B.HoughTransform_o2[56] = -0.59341194567807209;
    soccer_vision_B.HoughTransform_o2[57] = -0.57595865315812877;
    soccer_vision_B.HoughTransform_o2[58] = -0.55850536063818546;
    soccer_vision_B.HoughTransform_o2[59] = -0.54105206811824214;
    soccer_vision_B.HoughTransform_o2[60] = -0.52359877559829882;
    soccer_vision_B.HoughTransform_o2[61] = -0.50614548307835561;
    soccer_vision_B.HoughTransform_o2[62] = -0.48869219055841229;
    soccer_vision_B.HoughTransform_o2[63] = -0.47123889803846897;
    soccer_vision_B.HoughTransform_o2[64] = -0.4537856055185257;
    soccer_vision_B.HoughTransform_o2[65] = -0.43633231299858238;
    soccer_vision_B.HoughTransform_o2[66] = -0.41887902047863912;
    soccer_vision_B.HoughTransform_o2[67] = -0.4014257279586958;
    soccer_vision_B.HoughTransform_o2[68] = -0.38397243543875248;
    soccer_vision_B.HoughTransform_o2[69] = -0.36651914291880922;
    soccer_vision_B.HoughTransform_o2[70] = -0.3490658503988659;
    soccer_vision_B.HoughTransform_o2[71] = -0.33161255787892263;
    soccer_vision_B.HoughTransform_o2[72] = -0.31415926535897931;
    soccer_vision_B.HoughTransform_o2[73] = -0.29670597283903605;
    soccer_vision_B.HoughTransform_o2[74] = -0.27925268031909273;
    soccer_vision_B.HoughTransform_o2[75] = -0.26179938779914941;
    soccer_vision_B.HoughTransform_o2[76] = -0.24434609527920614;
    soccer_vision_B.HoughTransform_o2[77] = -0.22689280275926285;
    soccer_vision_B.HoughTransform_o2[78] = -0.20943951023931956;
    soccer_vision_B.HoughTransform_o2[79] = -0.19198621771937624;
    soccer_vision_B.HoughTransform_o2[80] = -0.17453292519943295;
    soccer_vision_B.HoughTransform_o2[81] = -0.15707963267948966;
    soccer_vision_B.HoughTransform_o2[82] = -0.13962634015954636;
    soccer_vision_B.HoughTransform_o2[83] = -0.12217304763960307;
    soccer_vision_B.HoughTransform_o2[84] = -0.10471975511965978;
    soccer_vision_B.HoughTransform_o2[85] = -0.087266462599716474;
    soccer_vision_B.HoughTransform_o2[86] = -0.069813170079773182;
    soccer_vision_B.HoughTransform_o2[87] = -0.05235987755982989;
    soccer_vision_B.HoughTransform_o2[88] = -0.034906585039886591;
    soccer_vision_B.HoughTransform_o2[89] = -0.017453292519943295;
    soccer_vision_B.HoughTransform_o2[90] = 0.0;
    soccer_vision_B.HoughTransform_o2[91] = 0.017453292519943295;
    soccer_vision_B.HoughTransform_o2[92] = 0.034906585039886591;
    soccer_vision_B.HoughTransform_o2[93] = 0.05235987755982989;
    soccer_vision_B.HoughTransform_o2[94] = 0.069813170079773182;
    soccer_vision_B.HoughTransform_o2[95] = 0.087266462599716474;
    soccer_vision_B.HoughTransform_o2[96] = 0.10471975511965978;
    soccer_vision_B.HoughTransform_o2[97] = 0.12217304763960307;
    soccer_vision_B.HoughTransform_o2[98] = 0.13962634015954636;
    soccer_vision_B.HoughTransform_o2[99] = 0.15707963267948966;
    soccer_vision_B.HoughTransform_o2[100] = 0.17453292519943295;
    soccer_vision_B.HoughTransform_o2[101] = 0.19198621771937624;
    soccer_vision_B.HoughTransform_o2[102] = 0.20943951023931956;
    soccer_vision_B.HoughTransform_o2[103] = 0.22689280275926285;
    soccer_vision_B.HoughTransform_o2[104] = 0.24434609527920614;
    soccer_vision_B.HoughTransform_o2[105] = 0.26179938779914941;
    soccer_vision_B.HoughTransform_o2[106] = 0.27925268031909273;
    soccer_vision_B.HoughTransform_o2[107] = 0.29670597283903605;
    soccer_vision_B.HoughTransform_o2[108] = 0.31415926535897931;
    soccer_vision_B.HoughTransform_o2[109] = 0.33161255787892263;
    soccer_vision_B.HoughTransform_o2[110] = 0.3490658503988659;
    soccer_vision_B.HoughTransform_o2[111] = 0.36651914291880922;
    soccer_vision_B.HoughTransform_o2[112] = 0.38397243543875248;
    soccer_vision_B.HoughTransform_o2[113] = 0.4014257279586958;
    soccer_vision_B.HoughTransform_o2[114] = 0.41887902047863912;
    soccer_vision_B.HoughTransform_o2[115] = 0.43633231299858238;
    soccer_vision_B.HoughTransform_o2[116] = 0.4537856055185257;
    soccer_vision_B.HoughTransform_o2[117] = 0.47123889803846897;
    soccer_vision_B.HoughTransform_o2[118] = 0.48869219055841229;
    soccer_vision_B.HoughTransform_o2[119] = 0.50614548307835561;
    soccer_vision_B.HoughTransform_o2[120] = 0.52359877559829882;
    soccer_vision_B.HoughTransform_o2[121] = 0.54105206811824214;
    soccer_vision_B.HoughTransform_o2[122] = 0.55850536063818546;
    soccer_vision_B.HoughTransform_o2[123] = 0.57595865315812877;
    soccer_vision_B.HoughTransform_o2[124] = 0.59341194567807209;
    soccer_vision_B.HoughTransform_o2[125] = 0.6108652381980153;
    soccer_vision_B.HoughTransform_o2[126] = 0.62831853071795862;
    soccer_vision_B.HoughTransform_o2[127] = 0.64577182323790194;
    soccer_vision_B.HoughTransform_o2[128] = 0.66322511575784526;
    soccer_vision_B.HoughTransform_o2[129] = 0.68067840827778847;
    soccer_vision_B.HoughTransform_o2[130] = 0.69813170079773179;
    soccer_vision_B.HoughTransform_o2[131] = 0.71558499331767511;
    soccer_vision_B.HoughTransform_o2[132] = 0.73303828583761843;
    soccer_vision_B.HoughTransform_o2[133] = 0.75049157835756175;
    soccer_vision_B.HoughTransform_o2[134] = 0.767944870877505;
    soccer_vision_B.HoughTransform_o2[135] = 0.78539816339744828;
    soccer_vision_B.HoughTransform_o2[136] = 0.8028514559173916;
    soccer_vision_B.HoughTransform_o2[137] = 0.82030474843733492;
    soccer_vision_B.HoughTransform_o2[138] = 0.83775804095727824;
    soccer_vision_B.HoughTransform_o2[139] = 0.85521133347722145;
    soccer_vision_B.HoughTransform_o2[140] = 0.87266462599716477;
    soccer_vision_B.HoughTransform_o2[141] = 0.89011791851710809;
    soccer_vision_B.HoughTransform_o2[142] = 0.90757121103705141;
    soccer_vision_B.HoughTransform_o2[143] = 0.92502450355699462;
    soccer_vision_B.HoughTransform_o2[144] = 0.94247779607693793;
    soccer_vision_B.HoughTransform_o2[145] = 0.95993108859688125;
    soccer_vision_B.HoughTransform_o2[146] = 0.97738438111682457;
    soccer_vision_B.HoughTransform_o2[147] = 0.99483767363676789;
    soccer_vision_B.HoughTransform_o2[148] = 1.0122909661567112;
    soccer_vision_B.HoughTransform_o2[149] = 1.0297442586766545;
    soccer_vision_B.HoughTransform_o2[150] = 1.0471975511965976;
    soccer_vision_B.HoughTransform_o2[151] = 1.064650843716541;
    soccer_vision_B.HoughTransform_o2[152] = 1.0821041362364843;
    soccer_vision_B.HoughTransform_o2[153] = 1.0995574287564276;
    soccer_vision_B.HoughTransform_o2[154] = 1.1170107212763709;
    soccer_vision_B.HoughTransform_o2[155] = 1.1344640137963142;
    soccer_vision_B.HoughTransform_o2[156] = 1.1519173063162575;
    soccer_vision_B.HoughTransform_o2[157] = 1.1693705988362009;
    soccer_vision_B.HoughTransform_o2[158] = 1.1868238913561442;
    soccer_vision_B.HoughTransform_o2[159] = 1.2042771838760873;
    soccer_vision_B.HoughTransform_o2[160] = 1.2217304763960306;
    soccer_vision_B.HoughTransform_o2[161] = 1.2391837689159739;
    soccer_vision_B.HoughTransform_o2[162] = 1.2566370614359172;
    soccer_vision_B.HoughTransform_o2[163] = 1.2740903539558606;
    soccer_vision_B.HoughTransform_o2[164] = 1.2915436464758039;
    soccer_vision_B.HoughTransform_o2[165] = 1.3089969389957472;
    soccer_vision_B.HoughTransform_o2[166] = 1.3264502315156905;
    soccer_vision_B.HoughTransform_o2[167] = 1.3439035240356338;
    soccer_vision_B.HoughTransform_o2[168] = 1.3613568165555769;
    soccer_vision_B.HoughTransform_o2[169] = 1.3788101090755203;
    soccer_vision_B.HoughTransform_o2[170] = 1.3962634015954636;
    soccer_vision_B.HoughTransform_o2[171] = 1.4137166941154069;
    soccer_vision_B.HoughTransform_o2[172] = 1.4311699866353502;
    soccer_vision_B.HoughTransform_o2[173] = 1.4486232791552935;
    soccer_vision_B.HoughTransform_o2[174] = 1.4660765716752369;
    soccer_vision_B.HoughTransform_o2[175] = 1.4835298641951802;
    soccer_vision_B.HoughTransform_o2[176] = 1.5009831567151235;
    soccer_vision_B.HoughTransform_o2[177] = 1.5184364492350666;
    soccer_vision_B.HoughTransform_o2[178] = 1.53588974175501;
    soccer_vision_B.HoughTransform_o2[179] = 1.5533430342749532;
    soccer_vision_B.HoughTransform_o3[0] = -199.0;
    soccer_vision_B.HoughTransform_o3[1] = -198.0;
    soccer_vision_B.HoughTransform_o3[2] = -197.0;
    soccer_vision_B.HoughTransform_o3[3] = -196.0;
    soccer_vision_B.HoughTransform_o3[4] = -195.0;
    soccer_vision_B.HoughTransform_o3[5] = -194.0;
    soccer_vision_B.HoughTransform_o3[6] = -193.0;
    soccer_vision_B.HoughTransform_o3[7] = -192.0;
    soccer_vision_B.HoughTransform_o3[8] = -191.0;
    soccer_vision_B.HoughTransform_o3[9] = -190.0;
    soccer_vision_B.HoughTransform_o3[10] = -189.0;
    soccer_vision_B.HoughTransform_o3[11] = -188.0;
    soccer_vision_B.HoughTransform_o3[12] = -187.0;
    soccer_vision_B.HoughTransform_o3[13] = -186.0;
    soccer_vision_B.HoughTransform_o3[14] = -185.0;
    soccer_vision_B.HoughTransform_o3[15] = -184.0;
    soccer_vision_B.HoughTransform_o3[16] = -183.0;
    soccer_vision_B.HoughTransform_o3[17] = -182.0;
    soccer_vision_B.HoughTransform_o3[18] = -181.0;
    soccer_vision_B.HoughTransform_o3[19] = -180.0;
    soccer_vision_B.HoughTransform_o3[20] = -179.0;
    soccer_vision_B.HoughTransform_o3[21] = -178.0;
    soccer_vision_B.HoughTransform_o3[22] = -177.0;
    soccer_vision_B.HoughTransform_o3[23] = -176.0;
    soccer_vision_B.HoughTransform_o3[24] = -175.0;
    soccer_vision_B.HoughTransform_o3[25] = -174.0;
    soccer_vision_B.HoughTransform_o3[26] = -173.0;
    soccer_vision_B.HoughTransform_o3[27] = -172.0;
    soccer_vision_B.HoughTransform_o3[28] = -171.0;
    soccer_vision_B.HoughTransform_o3[29] = -170.0;
    soccer_vision_B.HoughTransform_o3[30] = -169.0;
    soccer_vision_B.HoughTransform_o3[31] = -168.0;
    soccer_vision_B.HoughTransform_o3[32] = -167.0;
    soccer_vision_B.HoughTransform_o3[33] = -166.0;
    soccer_vision_B.HoughTransform_o3[34] = -165.0;
    soccer_vision_B.HoughTransform_o3[35] = -164.0;
    soccer_vision_B.HoughTransform_o3[36] = -163.0;
    soccer_vision_B.HoughTransform_o3[37] = -162.0;
    soccer_vision_B.HoughTransform_o3[38] = -161.0;
    soccer_vision_B.HoughTransform_o3[39] = -160.0;
    soccer_vision_B.HoughTransform_o3[40] = -159.0;
    soccer_vision_B.HoughTransform_o3[41] = -158.0;
    soccer_vision_B.HoughTransform_o3[42] = -157.0;
    soccer_vision_B.HoughTransform_o3[43] = -156.0;
    soccer_vision_B.HoughTransform_o3[44] = -155.0;
    soccer_vision_B.HoughTransform_o3[45] = -154.0;
    soccer_vision_B.HoughTransform_o3[46] = -153.0;
    soccer_vision_B.HoughTransform_o3[47] = -152.0;
    soccer_vision_B.HoughTransform_o3[48] = -151.0;
    soccer_vision_B.HoughTransform_o3[49] = -150.0;
    soccer_vision_B.HoughTransform_o3[50] = -149.0;
    soccer_vision_B.HoughTransform_o3[51] = -148.0;
    soccer_vision_B.HoughTransform_o3[52] = -147.0;
    soccer_vision_B.HoughTransform_o3[53] = -146.0;
    soccer_vision_B.HoughTransform_o3[54] = -145.0;
    soccer_vision_B.HoughTransform_o3[55] = -144.0;
    soccer_vision_B.HoughTransform_o3[56] = -143.0;
    soccer_vision_B.HoughTransform_o3[57] = -142.0;
    soccer_vision_B.HoughTransform_o3[58] = -141.0;
    soccer_vision_B.HoughTransform_o3[59] = -140.0;
    soccer_vision_B.HoughTransform_o3[60] = -139.0;
    soccer_vision_B.HoughTransform_o3[61] = -138.0;
    soccer_vision_B.HoughTransform_o3[62] = -137.0;
    soccer_vision_B.HoughTransform_o3[63] = -136.0;
    soccer_vision_B.HoughTransform_o3[64] = -135.0;
    soccer_vision_B.HoughTransform_o3[65] = -134.0;
    soccer_vision_B.HoughTransform_o3[66] = -133.0;
    soccer_vision_B.HoughTransform_o3[67] = -132.0;
    soccer_vision_B.HoughTransform_o3[68] = -131.0;
    soccer_vision_B.HoughTransform_o3[69] = -130.0;
    soccer_vision_B.HoughTransform_o3[70] = -129.0;
    soccer_vision_B.HoughTransform_o3[71] = -128.0;
    soccer_vision_B.HoughTransform_o3[72] = -127.0;
    soccer_vision_B.HoughTransform_o3[73] = -126.0;
    soccer_vision_B.HoughTransform_o3[74] = -125.0;
    soccer_vision_B.HoughTransform_o3[75] = -124.0;
    soccer_vision_B.HoughTransform_o3[76] = -123.0;
    soccer_vision_B.HoughTransform_o3[77] = -122.0;
    soccer_vision_B.HoughTransform_o3[78] = -121.0;
    soccer_vision_B.HoughTransform_o3[79] = -120.0;
    soccer_vision_B.HoughTransform_o3[80] = -119.0;
    soccer_vision_B.HoughTransform_o3[81] = -118.0;
    soccer_vision_B.HoughTransform_o3[82] = -117.0;
    soccer_vision_B.HoughTransform_o3[83] = -116.0;
    soccer_vision_B.HoughTransform_o3[84] = -115.0;
    soccer_vision_B.HoughTransform_o3[85] = -114.0;
    soccer_vision_B.HoughTransform_o3[86] = -113.0;
    soccer_vision_B.HoughTransform_o3[87] = -112.0;
    soccer_vision_B.HoughTransform_o3[88] = -111.0;
    soccer_vision_B.HoughTransform_o3[89] = -110.0;
    soccer_vision_B.HoughTransform_o3[90] = -109.0;
    soccer_vision_B.HoughTransform_o3[91] = -108.0;
    soccer_vision_B.HoughTransform_o3[92] = -107.0;
    soccer_vision_B.HoughTransform_o3[93] = -106.0;
    soccer_vision_B.HoughTransform_o3[94] = -105.0;
    soccer_vision_B.HoughTransform_o3[95] = -104.0;
    soccer_vision_B.HoughTransform_o3[96] = -103.0;
    soccer_vision_B.HoughTransform_o3[97] = -102.0;
    soccer_vision_B.HoughTransform_o3[98] = -101.0;
    soccer_vision_B.HoughTransform_o3[99] = -100.0;
    soccer_vision_B.HoughTransform_o3[100] = -99.0;
    soccer_vision_B.HoughTransform_o3[101] = -98.0;
    soccer_vision_B.HoughTransform_o3[102] = -97.0;
    soccer_vision_B.HoughTransform_o3[103] = -96.0;
    soccer_vision_B.HoughTransform_o3[104] = -95.0;
    soccer_vision_B.HoughTransform_o3[105] = -94.0;
    soccer_vision_B.HoughTransform_o3[106] = -93.0;
    soccer_vision_B.HoughTransform_o3[107] = -92.0;
    soccer_vision_B.HoughTransform_o3[108] = -91.0;
    soccer_vision_B.HoughTransform_o3[109] = -90.0;
    soccer_vision_B.HoughTransform_o3[110] = -89.0;
    soccer_vision_B.HoughTransform_o3[111] = -88.0;
    soccer_vision_B.HoughTransform_o3[112] = -87.0;
    soccer_vision_B.HoughTransform_o3[113] = -86.0;
    soccer_vision_B.HoughTransform_o3[114] = -85.0;
    soccer_vision_B.HoughTransform_o3[115] = -84.0;
    soccer_vision_B.HoughTransform_o3[116] = -83.0;
    soccer_vision_B.HoughTransform_o3[117] = -82.0;
    soccer_vision_B.HoughTransform_o3[118] = -81.0;
    soccer_vision_B.HoughTransform_o3[119] = -80.0;
    soccer_vision_B.HoughTransform_o3[120] = -79.0;
    soccer_vision_B.HoughTransform_o3[121] = -78.0;
    soccer_vision_B.HoughTransform_o3[122] = -77.0;
    soccer_vision_B.HoughTransform_o3[123] = -76.0;
    soccer_vision_B.HoughTransform_o3[124] = -75.0;
    soccer_vision_B.HoughTransform_o3[125] = -74.0;
    soccer_vision_B.HoughTransform_o3[126] = -73.0;
    soccer_vision_B.HoughTransform_o3[127] = -72.0;
    soccer_vision_B.HoughTransform_o3[128] = -71.0;
    soccer_vision_B.HoughTransform_o3[129] = -70.0;
    soccer_vision_B.HoughTransform_o3[130] = -69.0;
    soccer_vision_B.HoughTransform_o3[131] = -68.0;
    soccer_vision_B.HoughTransform_o3[132] = -67.0;
    soccer_vision_B.HoughTransform_o3[133] = -66.0;
    soccer_vision_B.HoughTransform_o3[134] = -65.0;
    soccer_vision_B.HoughTransform_o3[135] = -64.0;
    soccer_vision_B.HoughTransform_o3[136] = -63.0;
    soccer_vision_B.HoughTransform_o3[137] = -62.0;
    soccer_vision_B.HoughTransform_o3[138] = -61.0;
    soccer_vision_B.HoughTransform_o3[139] = -60.0;
    soccer_vision_B.HoughTransform_o3[140] = -59.0;
    soccer_vision_B.HoughTransform_o3[141] = -58.0;
    soccer_vision_B.HoughTransform_o3[142] = -57.0;
    soccer_vision_B.HoughTransform_o3[143] = -56.0;
    soccer_vision_B.HoughTransform_o3[144] = -55.0;
    soccer_vision_B.HoughTransform_o3[145] = -54.0;
    soccer_vision_B.HoughTransform_o3[146] = -53.0;
    soccer_vision_B.HoughTransform_o3[147] = -52.0;
    soccer_vision_B.HoughTransform_o3[148] = -51.0;
    soccer_vision_B.HoughTransform_o3[149] = -50.0;
    soccer_vision_B.HoughTransform_o3[150] = -49.0;
    soccer_vision_B.HoughTransform_o3[151] = -48.0;
    soccer_vision_B.HoughTransform_o3[152] = -47.0;
    soccer_vision_B.HoughTransform_o3[153] = -46.0;
    soccer_vision_B.HoughTransform_o3[154] = -45.0;
    soccer_vision_B.HoughTransform_o3[155] = -44.0;
    soccer_vision_B.HoughTransform_o3[156] = -43.0;
    soccer_vision_B.HoughTransform_o3[157] = -42.0;
    soccer_vision_B.HoughTransform_o3[158] = -41.0;
    soccer_vision_B.HoughTransform_o3[159] = -40.0;
    soccer_vision_B.HoughTransform_o3[160] = -39.0;
    soccer_vision_B.HoughTransform_o3[161] = -38.0;
    soccer_vision_B.HoughTransform_o3[162] = -37.0;
    soccer_vision_B.HoughTransform_o3[163] = -36.0;
    soccer_vision_B.HoughTransform_o3[164] = -35.0;
    soccer_vision_B.HoughTransform_o3[165] = -34.0;
    soccer_vision_B.HoughTransform_o3[166] = -33.0;
    soccer_vision_B.HoughTransform_o3[167] = -32.0;
    soccer_vision_B.HoughTransform_o3[168] = -31.0;
    soccer_vision_B.HoughTransform_o3[169] = -30.0;
    soccer_vision_B.HoughTransform_o3[170] = -29.0;
    soccer_vision_B.HoughTransform_o3[171] = -28.0;
    soccer_vision_B.HoughTransform_o3[172] = -27.0;
    soccer_vision_B.HoughTransform_o3[173] = -26.0;
    soccer_vision_B.HoughTransform_o3[174] = -25.0;
    soccer_vision_B.HoughTransform_o3[175] = -24.0;
    soccer_vision_B.HoughTransform_o3[176] = -23.0;
    soccer_vision_B.HoughTransform_o3[177] = -22.0;
    soccer_vision_B.HoughTransform_o3[178] = -21.0;
    soccer_vision_B.HoughTransform_o3[179] = -20.0;
    soccer_vision_B.HoughTransform_o3[180] = -19.0;
    soccer_vision_B.HoughTransform_o3[181] = -18.0;
    soccer_vision_B.HoughTransform_o3[182] = -17.0;
    soccer_vision_B.HoughTransform_o3[183] = -16.0;
    soccer_vision_B.HoughTransform_o3[184] = -15.0;
    soccer_vision_B.HoughTransform_o3[185] = -14.0;
    soccer_vision_B.HoughTransform_o3[186] = -13.0;
    soccer_vision_B.HoughTransform_o3[187] = -12.0;
    soccer_vision_B.HoughTransform_o3[188] = -11.0;
    soccer_vision_B.HoughTransform_o3[189] = -10.0;
    soccer_vision_B.HoughTransform_o3[190] = -9.0;
    soccer_vision_B.HoughTransform_o3[191] = -8.0;
    soccer_vision_B.HoughTransform_o3[192] = -7.0;
    soccer_vision_B.HoughTransform_o3[193] = -6.0;
    soccer_vision_B.HoughTransform_o3[194] = -5.0;
    soccer_vision_B.HoughTransform_o3[195] = -4.0;
    soccer_vision_B.HoughTransform_o3[196] = -3.0;
    soccer_vision_B.HoughTransform_o3[197] = -2.0;
    soccer_vision_B.HoughTransform_o3[198] = -1.0;
    soccer_vision_B.HoughTransform_o3[199] = 0.0;
    soccer_vision_B.HoughTransform_o3[200] = 1.0;
    soccer_vision_B.HoughTransform_o3[201] = 2.0;
    soccer_vision_B.HoughTransform_o3[202] = 3.0;
    soccer_vision_B.HoughTransform_o3[203] = 4.0;
    soccer_vision_B.HoughTransform_o3[204] = 5.0;
    soccer_vision_B.HoughTransform_o3[205] = 6.0;
    soccer_vision_B.HoughTransform_o3[206] = 7.0;
    soccer_vision_B.HoughTransform_o3[207] = 8.0;
    soccer_vision_B.HoughTransform_o3[208] = 9.0;
    soccer_vision_B.HoughTransform_o3[209] = 10.0;
    soccer_vision_B.HoughTransform_o3[210] = 11.0;
    soccer_vision_B.HoughTransform_o3[211] = 12.0;
    soccer_vision_B.HoughTransform_o3[212] = 13.0;
    soccer_vision_B.HoughTransform_o3[213] = 14.0;
    soccer_vision_B.HoughTransform_o3[214] = 15.0;
    soccer_vision_B.HoughTransform_o3[215] = 16.0;
    soccer_vision_B.HoughTransform_o3[216] = 17.0;
    soccer_vision_B.HoughTransform_o3[217] = 18.0;
    soccer_vision_B.HoughTransform_o3[218] = 19.0;
    soccer_vision_B.HoughTransform_o3[219] = 20.0;
    soccer_vision_B.HoughTransform_o3[220] = 21.0;
    soccer_vision_B.HoughTransform_o3[221] = 22.0;
    soccer_vision_B.HoughTransform_o3[222] = 23.0;
    soccer_vision_B.HoughTransform_o3[223] = 24.0;
    soccer_vision_B.HoughTransform_o3[224] = 25.0;
    soccer_vision_B.HoughTransform_o3[225] = 26.0;
    soccer_vision_B.HoughTransform_o3[226] = 27.0;
    soccer_vision_B.HoughTransform_o3[227] = 28.0;
    soccer_vision_B.HoughTransform_o3[228] = 29.0;
    soccer_vision_B.HoughTransform_o3[229] = 30.0;
    soccer_vision_B.HoughTransform_o3[230] = 31.0;
    soccer_vision_B.HoughTransform_o3[231] = 32.0;
    soccer_vision_B.HoughTransform_o3[232] = 33.0;
    soccer_vision_B.HoughTransform_o3[233] = 34.0;
    soccer_vision_B.HoughTransform_o3[234] = 35.0;
    soccer_vision_B.HoughTransform_o3[235] = 36.0;
    soccer_vision_B.HoughTransform_o3[236] = 37.0;
    soccer_vision_B.HoughTransform_o3[237] = 38.0;
    soccer_vision_B.HoughTransform_o3[238] = 39.0;
    soccer_vision_B.HoughTransform_o3[239] = 40.0;
    soccer_vision_B.HoughTransform_o3[240] = 41.0;
    soccer_vision_B.HoughTransform_o3[241] = 42.0;
    soccer_vision_B.HoughTransform_o3[242] = 43.0;
    soccer_vision_B.HoughTransform_o3[243] = 44.0;
    soccer_vision_B.HoughTransform_o3[244] = 45.0;
    soccer_vision_B.HoughTransform_o3[245] = 46.0;
    soccer_vision_B.HoughTransform_o3[246] = 47.0;
    soccer_vision_B.HoughTransform_o3[247] = 48.0;
    soccer_vision_B.HoughTransform_o3[248] = 49.0;
    soccer_vision_B.HoughTransform_o3[249] = 50.0;
    soccer_vision_B.HoughTransform_o3[250] = 51.0;
    soccer_vision_B.HoughTransform_o3[251] = 52.0;
    soccer_vision_B.HoughTransform_o3[252] = 53.0;
    soccer_vision_B.HoughTransform_o3[253] = 54.0;
    soccer_vision_B.HoughTransform_o3[254] = 55.0;
    soccer_vision_B.HoughTransform_o3[255] = 56.0;
    soccer_vision_B.HoughTransform_o3[256] = 57.0;
    soccer_vision_B.HoughTransform_o3[257] = 58.0;
    soccer_vision_B.HoughTransform_o3[258] = 59.0;
    soccer_vision_B.HoughTransform_o3[259] = 60.0;
    soccer_vision_B.HoughTransform_o3[260] = 61.0;
    soccer_vision_B.HoughTransform_o3[261] = 62.0;
    soccer_vision_B.HoughTransform_o3[262] = 63.0;
    soccer_vision_B.HoughTransform_o3[263] = 64.0;
    soccer_vision_B.HoughTransform_o3[264] = 65.0;
    soccer_vision_B.HoughTransform_o3[265] = 66.0;
    soccer_vision_B.HoughTransform_o3[266] = 67.0;
    soccer_vision_B.HoughTransform_o3[267] = 68.0;
    soccer_vision_B.HoughTransform_o3[268] = 69.0;
    soccer_vision_B.HoughTransform_o3[269] = 70.0;
    soccer_vision_B.HoughTransform_o3[270] = 71.0;
    soccer_vision_B.HoughTransform_o3[271] = 72.0;
    soccer_vision_B.HoughTransform_o3[272] = 73.0;
    soccer_vision_B.HoughTransform_o3[273] = 74.0;
    soccer_vision_B.HoughTransform_o3[274] = 75.0;
    soccer_vision_B.HoughTransform_o3[275] = 76.0;
    soccer_vision_B.HoughTransform_o3[276] = 77.0;
    soccer_vision_B.HoughTransform_o3[277] = 78.0;
    soccer_vision_B.HoughTransform_o3[278] = 79.0;
    soccer_vision_B.HoughTransform_o3[279] = 80.0;
    soccer_vision_B.HoughTransform_o3[280] = 81.0;
    soccer_vision_B.HoughTransform_o3[281] = 82.0;
    soccer_vision_B.HoughTransform_o3[282] = 83.0;
    soccer_vision_B.HoughTransform_o3[283] = 84.0;
    soccer_vision_B.HoughTransform_o3[284] = 85.0;
    soccer_vision_B.HoughTransform_o3[285] = 86.0;
    soccer_vision_B.HoughTransform_o3[286] = 87.0;
    soccer_vision_B.HoughTransform_o3[287] = 88.0;
    soccer_vision_B.HoughTransform_o3[288] = 89.0;
    soccer_vision_B.HoughTransform_o3[289] = 90.0;
    soccer_vision_B.HoughTransform_o3[290] = 91.0;
    soccer_vision_B.HoughTransform_o3[291] = 92.0;
    soccer_vision_B.HoughTransform_o3[292] = 93.0;
    soccer_vision_B.HoughTransform_o3[293] = 94.0;
    soccer_vision_B.HoughTransform_o3[294] = 95.0;
    soccer_vision_B.HoughTransform_o3[295] = 96.0;
    soccer_vision_B.HoughTransform_o3[296] = 97.0;
    soccer_vision_B.HoughTransform_o3[297] = 98.0;
    soccer_vision_B.HoughTransform_o3[298] = 99.0;
    soccer_vision_B.HoughTransform_o3[299] = 100.0;
    soccer_vision_B.HoughTransform_o3[300] = 101.0;
    soccer_vision_B.HoughTransform_o3[301] = 102.0;
    soccer_vision_B.HoughTransform_o3[302] = 103.0;
    soccer_vision_B.HoughTransform_o3[303] = 104.0;
    soccer_vision_B.HoughTransform_o3[304] = 105.0;
    soccer_vision_B.HoughTransform_o3[305] = 106.0;
    soccer_vision_B.HoughTransform_o3[306] = 107.0;
    soccer_vision_B.HoughTransform_o3[307] = 108.0;
    soccer_vision_B.HoughTransform_o3[308] = 109.0;
    soccer_vision_B.HoughTransform_o3[309] = 110.0;
    soccer_vision_B.HoughTransform_o3[310] = 111.0;
    soccer_vision_B.HoughTransform_o3[311] = 112.0;
    soccer_vision_B.HoughTransform_o3[312] = 113.0;
    soccer_vision_B.HoughTransform_o3[313] = 114.0;
    soccer_vision_B.HoughTransform_o3[314] = 115.0;
    soccer_vision_B.HoughTransform_o3[315] = 116.0;
    soccer_vision_B.HoughTransform_o3[316] = 117.0;
    soccer_vision_B.HoughTransform_o3[317] = 118.0;
    soccer_vision_B.HoughTransform_o3[318] = 119.0;
    soccer_vision_B.HoughTransform_o3[319] = 120.0;
    soccer_vision_B.HoughTransform_o3[320] = 121.0;
    soccer_vision_B.HoughTransform_o3[321] = 122.0;
    soccer_vision_B.HoughTransform_o3[322] = 123.0;
    soccer_vision_B.HoughTransform_o3[323] = 124.0;
    soccer_vision_B.HoughTransform_o3[324] = 125.0;
    soccer_vision_B.HoughTransform_o3[325] = 126.0;
    soccer_vision_B.HoughTransform_o3[326] = 127.0;
    soccer_vision_B.HoughTransform_o3[327] = 128.0;
    soccer_vision_B.HoughTransform_o3[328] = 129.0;
    soccer_vision_B.HoughTransform_o3[329] = 130.0;
    soccer_vision_B.HoughTransform_o3[330] = 131.0;
    soccer_vision_B.HoughTransform_o3[331] = 132.0;
    soccer_vision_B.HoughTransform_o3[332] = 133.0;
    soccer_vision_B.HoughTransform_o3[333] = 134.0;
    soccer_vision_B.HoughTransform_o3[334] = 135.0;
    soccer_vision_B.HoughTransform_o3[335] = 136.0;
    soccer_vision_B.HoughTransform_o3[336] = 137.0;
    soccer_vision_B.HoughTransform_o3[337] = 138.0;
    soccer_vision_B.HoughTransform_o3[338] = 139.0;
    soccer_vision_B.HoughTransform_o3[339] = 140.0;
    soccer_vision_B.HoughTransform_o3[340] = 141.0;
    soccer_vision_B.HoughTransform_o3[341] = 142.0;
    soccer_vision_B.HoughTransform_o3[342] = 143.0;
    soccer_vision_B.HoughTransform_o3[343] = 144.0;
    soccer_vision_B.HoughTransform_o3[344] = 145.0;
    soccer_vision_B.HoughTransform_o3[345] = 146.0;
    soccer_vision_B.HoughTransform_o3[346] = 147.0;
    soccer_vision_B.HoughTransform_o3[347] = 148.0;
    soccer_vision_B.HoughTransform_o3[348] = 149.0;
    soccer_vision_B.HoughTransform_o3[349] = 150.0;
    soccer_vision_B.HoughTransform_o3[350] = 151.0;
    soccer_vision_B.HoughTransform_o3[351] = 152.0;
    soccer_vision_B.HoughTransform_o3[352] = 153.0;
    soccer_vision_B.HoughTransform_o3[353] = 154.0;
    soccer_vision_B.HoughTransform_o3[354] = 155.0;
    soccer_vision_B.HoughTransform_o3[355] = 156.0;
    soccer_vision_B.HoughTransform_o3[356] = 157.0;
    soccer_vision_B.HoughTransform_o3[357] = 158.0;
    soccer_vision_B.HoughTransform_o3[358] = 159.0;
    soccer_vision_B.HoughTransform_o3[359] = 160.0;
    soccer_vision_B.HoughTransform_o3[360] = 161.0;
    soccer_vision_B.HoughTransform_o3[361] = 162.0;
    soccer_vision_B.HoughTransform_o3[362] = 163.0;
    soccer_vision_B.HoughTransform_o3[363] = 164.0;
    soccer_vision_B.HoughTransform_o3[364] = 165.0;
    soccer_vision_B.HoughTransform_o3[365] = 166.0;
    soccer_vision_B.HoughTransform_o3[366] = 167.0;
    soccer_vision_B.HoughTransform_o3[367] = 168.0;
    soccer_vision_B.HoughTransform_o3[368] = 169.0;
    soccer_vision_B.HoughTransform_o3[369] = 170.0;
    soccer_vision_B.HoughTransform_o3[370] = 171.0;
    soccer_vision_B.HoughTransform_o3[371] = 172.0;
    soccer_vision_B.HoughTransform_o3[372] = 173.0;
    soccer_vision_B.HoughTransform_o3[373] = 174.0;
    soccer_vision_B.HoughTransform_o3[374] = 175.0;
    soccer_vision_B.HoughTransform_o3[375] = 176.0;
    soccer_vision_B.HoughTransform_o3[376] = 177.0;
    soccer_vision_B.HoughTransform_o3[377] = 178.0;
    soccer_vision_B.HoughTransform_o3[378] = 179.0;
    soccer_vision_B.HoughTransform_o3[379] = 180.0;
    soccer_vision_B.HoughTransform_o3[380] = 181.0;
    soccer_vision_B.HoughTransform_o3[381] = 182.0;
    soccer_vision_B.HoughTransform_o3[382] = 183.0;
    soccer_vision_B.HoughTransform_o3[383] = 184.0;
    soccer_vision_B.HoughTransform_o3[384] = 185.0;
    soccer_vision_B.HoughTransform_o3[385] = 186.0;
    soccer_vision_B.HoughTransform_o3[386] = 187.0;
    soccer_vision_B.HoughTransform_o3[387] = 188.0;
    soccer_vision_B.HoughTransform_o3[388] = 189.0;
    soccer_vision_B.HoughTransform_o3[389] = 190.0;
    soccer_vision_B.HoughTransform_o3[390] = 191.0;
    soccer_vision_B.HoughTransform_o3[391] = 192.0;
    soccer_vision_B.HoughTransform_o3[392] = 193.0;
    soccer_vision_B.HoughTransform_o3[393] = 194.0;
    soccer_vision_B.HoughTransform_o3[394] = 195.0;
    soccer_vision_B.HoughTransform_o3[395] = 196.0;
    soccer_vision_B.HoughTransform_o3[396] = 197.0;
    soccer_vision_B.HoughTransform_o3[397] = 198.0;
    soccer_vision_B.HoughTransform_o3[398] = 199.0;
  }

  // states (dwork)
  (void) memset((void *)&soccer_vision_DW, 0,
                sizeof(DW_soccer_vision_T));

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    soccer_vision_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 28;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  {
    uint32_T r;
    int32_T nonZeroIdx;
    static const char_T tmp[28] = { '/', 's', 'o', 'c', 'c', 'e', 'r', 'b', 'o',
      't', '/', 'c', 'a', 'm', 'e', 'r', 'a', '1', '/', 'i', 'm', 'a', 'g', 'e',
      '_', 'r', 'a', 'w' };

    char_T tmp_0[29];
    int32_T EdgeDetection_VO_DW_tmp;

    // Start for Atomic SubSystem: '<S4>/Subscribe'
    // Start for MATLABSystem: '<S5>/SourceBlock'
    soccer_vision_DW.obj_h.matlabCodegenIsDeleted = true;
    soccer_vision_DW.obj_h.isInitialized = 0;
    soccer_vision_DW.obj_h.matlabCodegenIsDeleted = false;
    soccer_vision_DW.obj_h.isSetupComplete = false;
    soccer_vision_DW.obj_h.isInitialized = 1;
    for (nonZeroIdx = 0; nonZeroIdx < 28; nonZeroIdx++) {
      tmp_0[nonZeroIdx] = tmp[nonZeroIdx];
    }

    tmp_0[28] = '\x00';
    Sub_soccer_vision_197.createSubscriber(tmp_0, soccer_vision_MessageQueueLen);
    soccer_vision_DW.obj_h.isSetupComplete = true;

    // End of Start for MATLABSystem: '<S5>/SourceBlock'
    // End of Start for SubSystem: '<S4>/Subscribe'

    // Start for MATLABSystem: '<S4>/Read Image'
    soccer_vision_DW.obj.isInitialized = 0;
    soccer_vision_DW.obj.isInitialized = 1;

    // Start for S-Function (svipedge): '<S10>/Edge Detection'
    // Calculate FillColor times Opacity.
    // Calculate One minus Opacity.
    soccer_vision_DW.EdgeDetection_MEAN_FACTOR_DW = 111848;
    for (nonZeroIdx = 0; nonZeroIdx < 6; nonZeroIdx++) {
      EdgeDetection_VO_DW_tmp =
        soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120 +
        soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
      soccer_vision_DW.EdgeDetection_VO_DW[nonZeroIdx] = EdgeDetection_VO_DW_tmp;
      if (soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx] > 0) {
        soccer_vision_DW.EdgeDetection_VOU_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_VOD_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120;
      } else {
        soccer_vision_DW.EdgeDetection_VOU_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120;
        soccer_vision_DW.EdgeDetection_VOD_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
      }

      if (soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] > 0) {
        soccer_vision_DW.EdgeDetection_VOL_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_VOR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
      } else {
        soccer_vision_DW.EdgeDetection_VOL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
        soccer_vision_DW.EdgeDetection_VOR_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
      }

      if ((soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx] < 0) &&
          (soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] < 0)) {
        soccer_vision_DW.EdgeDetection_VOUL_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_VOLR_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_VOLL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
        soccer_vision_DW.EdgeDetection_VOUR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120;
      }

      if ((soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx] >= 0) &&
          (soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] < 0)) {
        soccer_vision_DW.EdgeDetection_VOLL_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_VOUR_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_VOUL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
        soccer_vision_DW.EdgeDetection_VOLR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120;
      }

      if ((soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx] < 0) &&
          (soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] >= 0)) {
        soccer_vision_DW.EdgeDetection_VOUR_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_VOLL_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_VOUL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120;
        soccer_vision_DW.EdgeDetection_VOLR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
      }

      if ((soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx] >= 0) &&
          (soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] >= 0)) {
        soccer_vision_DW.EdgeDetection_VOLR_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_VOUL_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_VOLL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VRO_RTP[nonZeroIdx] * 120;
        soccer_vision_DW.EdgeDetection_VOUR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_VCO_RTP[nonZeroIdx];
      }

      EdgeDetection_VO_DW_tmp =
        soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120 +
        soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
      soccer_vision_DW.EdgeDetection_HO_DW[nonZeroIdx] = EdgeDetection_VO_DW_tmp;
      if (soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx] > 0) {
        soccer_vision_DW.EdgeDetection_HOU_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_HOD_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120;
      } else {
        soccer_vision_DW.EdgeDetection_HOU_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120;
        soccer_vision_DW.EdgeDetection_HOD_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
      }

      if (soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] > 0) {
        soccer_vision_DW.EdgeDetection_HOL_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_HOR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
      } else {
        soccer_vision_DW.EdgeDetection_HOL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
        soccer_vision_DW.EdgeDetection_HOR_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
      }

      if ((soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx] < 0) &&
          (soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] < 0)) {
        soccer_vision_DW.EdgeDetection_HOUL_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_HOLR_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_HOLL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
        soccer_vision_DW.EdgeDetection_HOUR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120;
      }

      if ((soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx] >= 0) &&
          (soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] < 0)) {
        soccer_vision_DW.EdgeDetection_HOLL_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_HOUR_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_HOUL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
        soccer_vision_DW.EdgeDetection_HOLR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120;
      }

      if ((soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx] < 0) &&
          (soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] >= 0)) {
        soccer_vision_DW.EdgeDetection_HOUR_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_HOLL_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_HOUL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120;
        soccer_vision_DW.EdgeDetection_HOLR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
      }

      if ((soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx] >= 0) &&
          (soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] >= 0)) {
        soccer_vision_DW.EdgeDetection_HOLR_DW[nonZeroIdx] = 0;
        soccer_vision_DW.EdgeDetection_HOUL_DW[nonZeroIdx] =
          EdgeDetection_VO_DW_tmp;
        soccer_vision_DW.EdgeDetection_HOLL_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HRO_RTP[nonZeroIdx] * 120;
        soccer_vision_DW.EdgeDetection_HOUR_DW[nonZeroIdx] =
          soccer_vision_ConstP.EdgeDetection_HCO_RTP[nonZeroIdx];
      }
    }

    // End of Start for S-Function (svipedge): '<S10>/Edge Detection'
    //  Converts a 2d to 3d point on the field
    //
    //  Pre-computed constants
    //  Perform one-time calculations, such as computing constants

    // InitializeConditions for Memory: '<S12>/Memory'
    memcpy(&soccer_vision_DW.Memory_PreviousInput[0],
           &soccer_vision_P.Memory_InitialCondition[0], sizeof(real_T) << 3U);

    // InitializeConditions for Memory: '<S12>/Memory1'
    memcpy(&soccer_vision_DW.Memory1_PreviousInput[0],
           &soccer_vision_P.Memory1_InitialCondition[0], 10U * sizeof(real_T));

    // SystemInitialize for Atomic SubSystem: '<S4>/Subscribe'
    // SystemInitialize for Enabled SubSystem: '<S5>/Enabled Subsystem'
    // SystemInitialize for Outport: '<S6>/Out1'
    soccer_vision_B.In1 = soccer_vision_P.Out1_Y0;

    // End of SystemInitialize for SubSystem: '<S5>/Enabled Subsystem'
    // End of SystemInitialize for SubSystem: '<S4>/Subscribe'

    // InitializeConditions for MATLABSystem: '<S4>/Read Image'
    memset(&soccer_vision_DW.obj.Image[0], 0, 921600U * sizeof(uint8_T));
    soccer_vision_DW.obj.ImageSize[0] = 480U;
    soccer_vision_DW.obj.ImageSize[1] = 640U;

    // SystemInitialize for MATLAB Function: '<S2>/Filter'
    memset(&soccer_vision_DW.state[0], 0, 625U * sizeof(uint32_T));
    r = 5489U;
    soccer_vision_DW.state[0] = 5489U;
    for (nonZeroIdx = 0; nonZeroIdx < 623; nonZeroIdx++) {
      r = ((r >> 30U ^ r) * 1812433253U + nonZeroIdx) + 1U;
      soccer_vision_DW.state[nonZeroIdx + 1] = r;
    }

    soccer_vision_DW.state[624] = 624U;

    // End of SystemInitialize for MATLAB Function: '<S2>/Filter'
  }
}

// Model terminate function
void soccer_vision_terminate(void)
{
  // Terminate for Atomic SubSystem: '<S4>/Subscribe'
  // Terminate for MATLABSystem: '<S5>/SourceBlock'
  soccer_vision_matlabCodegenHandle_matlabCodegenDestructor
    (&soccer_vision_DW.obj_h);

  // End of Terminate for SubSystem: '<S4>/Subscribe'
}

//
// File trailer for generated code.
//
// [EOF]
//
