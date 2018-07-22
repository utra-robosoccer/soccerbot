//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_strategy_data.cpp
//
// Code generated for Simulink model 'soccer_strategy'.
//
// Model version                  : 1.641
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sat Jul 14 19:27:01 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Emulation hardware selection:
//    Differs from embedded hardware (MATLAB Host)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "soccer_strategy.h"
#include "soccer_strategy_private.h"

// Block parameters (default storage)
P_soccer_strategy_T soccer_strategy_P = {
  // Variable: ready
  //  Referenced by: '<S27>/Constant1'

  { 0.0, -0.05, 0.6, -1.4, 0.8, 0.05, 0.0, 0.05, 0.6, -1.4, 0.8, -0.05, 0.75,
    0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Variable: standing
  //  Referenced by: '<S27>/Constant'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },

  // Variable: motorCalibration
  //  Referenced by: '<S2>/Calibration'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Mask Parameter: PIDController1_D
  //  Referenced by: '<S18>/Derivative Gain'

  -3.0F,

  // Mask Parameter: PIDController_D
  //  Referenced by: '<S21>/Derivative Gain'

  0.0F,

  // Mask Parameter: PIDController1_D_b
  //  Referenced by: '<S22>/Derivative Gain'

  0.0F,

  // Mask Parameter: PIDController_D_e
  //  Referenced by: '<S17>/Derivative Gain'

  0.0F,

  // Mask Parameter: PIDController_I
  //  Referenced by: '<S17>/Integral Gain'

  0.2F,

  // Mask Parameter: PIDController1_I
  //  Referenced by: '<S18>/Integral Gain'

  1.0F,

  // Mask Parameter: PIDController_I_k
  //  Referenced by: '<S21>/Integral Gain'

  0.0F,

  // Mask Parameter: PIDController1_I_c
  //  Referenced by: '<S22>/Integral Gain'

  0.0F,

  // Mask Parameter: PIDController_N
  //  Referenced by: '<S21>/Filter Coefficient'

  100.0F,

  // Mask Parameter: PIDController1_N
  //  Referenced by: '<S22>/Filter Coefficient'

  100.0F,

  // Mask Parameter: PIDController1_P
  //  Referenced by: '<S18>/Proportional Gain'

  100.0F,

  // Mask Parameter: PIDController_P
  //  Referenced by: '<S21>/Proportional Gain'

  1.0F,

  // Mask Parameter: PIDController1_P_h
  //  Referenced by: '<S22>/Proportional Gain'

  1.0F,

  // Mask Parameter: PIDController_P_n
  //  Referenced by: '<S17>/Proportional Gain'

  1.3F,

  // Mask Parameter: WrapToZero1_Threshold
  //  Referenced by: '<S24>/FixPt Switch'

  0.0F,

  // Mask Parameter: WrapToZero_Threshold
  //  Referenced by: '<S23>/FixPt Switch'

  0.0F,

  // Computed Parameter: Out1_Y0
  //  Referenced by: '<S41>/Out1'

  {
    0U,                                // ProtocolVersion
    0U,                                // PacketNum
    0U,                                // PlayersPerTeam
    0U,                                // GameType
    0U,                                // State
    0U,                                // Firsthalf
    0U,                                // KickoffTeam
    0U,                                // SecondaryState
    0U,                                // DropInTeam
    0U,                                // DropInTime
    0U,                                // SecsRemaining
    0U,                                // SecondaryTime

    {
      0U, 0U, 0U, 0U }
    ,                                  // Header

    {
      0U, 0U, 0U, 0U }
    ,                                  // SecondaryStateInfo

    {
      {
        0U,                            // TeamNum
        0U,                            // TeamColour
        0U,                            // Score
        0U,                            // PenaltyShot
        0U,                            // SingleShots
        0U,                            // CoachSequence

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U }
        ,                              // CoachMessage

        {
          0U,                          // Penalty
          0U,                          // SecsTillUnpenalised
          0U,                          // YellowCardCount
          0U                           // RedCardCount
        },                             // Coach

        {
          {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          } }
        // Players
      }, {
        0U,                            // TeamNum
        0U,                            // TeamColour
        0U,                            // Score
        0U,                            // PenaltyShot
        0U,                            // SingleShots
        0U,                            // CoachSequence

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U }
        ,                              // CoachMessage

        {
          0U,                          // Penalty
          0U,                          // SecsTillUnpenalised
          0U,                          // YellowCardCount
          0U                           // RedCardCount
        },                             // Coach

        {
          {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          } }
        // Players
      } }
    // Teams
  },

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S8>/Constant'

  {
    0U,                                // ProtocolVersion
    0U,                                // PacketNum
    0U,                                // PlayersPerTeam
    0U,                                // GameType
    0U,                                // State
    0U,                                // Firsthalf
    0U,                                // KickoffTeam
    0U,                                // SecondaryState
    0U,                                // DropInTeam
    0U,                                // DropInTime
    0U,                                // SecsRemaining
    0U,                                // SecondaryTime

    {
      0U, 0U, 0U, 0U }
    ,                                  // Header

    {
      0U, 0U, 0U, 0U }
    ,                                  // SecondaryStateInfo

    {
      {
        0U,                            // TeamNum
        0U,                            // TeamColour
        0U,                            // Score
        0U,                            // PenaltyShot
        0U,                            // SingleShots
        0U,                            // CoachSequence

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U }
        ,                              // CoachMessage

        {
          0U,                          // Penalty
          0U,                          // SecsTillUnpenalised
          0U,                          // YellowCardCount
          0U                           // RedCardCount
        },                             // Coach

        {
          {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          } }
        // Players
      }, {
        0U,                            // TeamNum
        0U,                            // TeamColour
        0U,                            // Score
        0U,                            // PenaltyShot
        0U,                            // SingleShots
        0U,                            // CoachSequence

        {
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
          0U }
        ,                              // CoachMessage

        {
          0U,                          // Penalty
          0U,                          // SecsTillUnpenalised
          0U,                          // YellowCardCount
          0U                           // RedCardCount
        },                             // Coach

        {
          {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          }, {
            0U,                        // Penalty
            0U,                        // SecsTillUnpenalised
            0U,                        // YellowCardCount
            0U                         // RedCardCount
          } }
        // Players
      } }
    // Teams
  },

  // Computed Parameter: Out1_Y0_l
  //  Referenced by: '<S40>/Out1'

  {
    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // OrientationCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // AngularVelocityCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // LinearAccelerationCovariance

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
    },                                 // Header

    {
      0.0,                             // X
      0.0,                             // Y
      0.0,                             // Z
      0.0                              // W
    },                                 // Orientation

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // AngularVelocity

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // LinearAcceleration
  },

  // Computed Parameter: Constant_Value_l
  //  Referenced by: '<S7>/Constant'

  {
    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // OrientationCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // AngularVelocityCovariance

    {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
    ,                                  // LinearAccelerationCovariance

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
    },                                 // Header

    {
      0.0,                             // X
      0.0,                             // Y
      0.0,                             // Z
      0.0                              // W
    },                                 // Orientation

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    },                                 // AngularVelocity

    {
      0.0,                             // X
      0.0,                             // Y
      0.0                              // Z
    }                                  // LinearAcceleration
  },

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S1>/Constant'

  {
    {
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // Trajectories

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

  // Expression: 1/180
  //  Referenced by: '<S3>/Gain'

  0.0055555555555555558,

  // Expression: 0
  //  Referenced by: '<S3>/Constant6'

  0.0,

  // Expression: 1/180
  //  Referenced by: '<S3>/Gain1'

  0.0055555555555555558,

  // Expression: 0.05
  //  Referenced by: '<S3>/Constant7'

  0.05,

  // Expression: [0,0,0,0,0,0,0,0]
  //  Referenced by: '<S25>/Constant'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: getupfrontsmooth.time(end)
  //  Referenced by: '<S30>/Step'

  20.0,

  // Expression: 0
  //  Referenced by: '<S30>/Step'

  0.0,

  // Expression: 1
  //  Referenced by: '<S30>/Step'

  1.0,

  // Expression: getupbacksmooth.time(end)
  //  Referenced by: '<S29>/Step1'

  5.0,

  // Expression: 0
  //  Referenced by: '<S29>/Step1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S29>/Step1'

  1.0,

  // Expression: standingtoready.time(end)
  //  Referenced by: '<S33>/Step'

  3.0,

  // Expression: 0
  //  Referenced by: '<S33>/Step'

  0.0,

  // Expression: 1
  //  Referenced by: '<S33>/Step'

  1.0,

  // Expression: readytostanding.time(end)
  //  Referenced by: '<S32>/Step'

  3.0,

  // Expression: 0
  //  Referenced by: '<S32>/Step'

  0.0,

  // Expression: 1
  //  Referenced by: '<S32>/Step'

  1.0,

  // Expression: headTrajectory.time(end)
  //  Referenced by: '<S31>/Step'

  9.99,

  // Expression: 0
  //  Referenced by: '<S31>/Step'

  0.0,

  // Expression: 1
  //  Referenced by: '<S31>/Step'

  1.0,

  // Expression: customTrajectory.time(end)
  //  Referenced by: '<S28>/Step'

  0.0,

  // Expression: 0
  //  Referenced by: '<S28>/Step'

  0.0,

  // Expression: 1
  //  Referenced by: '<S28>/Step'

  1.0,

  // Expression: 10
  //  Referenced by: '<S27>/Constant2'

  10.0,

  // Expression: 12
  //  Referenced by: '<Root>/Override Command'

  12.0,

  // Expression: 0.0
  //  Referenced by: '<Root>/Delay'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Switch'

  0.0,

  // Expression: 0
  //  Referenced by: '<S3>/Constant3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S3>/Switch2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S3>/Constant2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S3>/Switch3'

  0.0,

  // Expression: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
  //  Referenced by: '<S2>/Constant'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0,
    15.0, 16.0, 17.0, 18.0, 19.0, 20.0 },

  // Expression: 0.0
  //  Referenced by: '<S4>/Delay'

  0.0,

  // Expression: 1
  //  Referenced by: '<Root>/Constant2'

  1.0,

  // Expression: 0
  //  Referenced by: '<Root>/Constant3'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Constant4'

  0.0,

  // Expression: 0
  //  Referenced by: '<Root>/Constant5'

  0.0,

  // Expression: [0 0 0]
  //  Referenced by: '<S14>/Delay'

  { 0.0, 0.0, 0.0 },

  // Computed Parameter: Merge1_InitialOutput
  //  Referenced by: '<S4>/Merge1'

  0.0,

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S23>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_d
  //  Referenced by: '<S24>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S3>/Constant'

  0.05F,

  // Computed Parameter: DeadZone_Start
  //  Referenced by: '<S10>/Dead Zone'

  -0.03F,

  // Computed Parameter: DeadZone_End
  //  Referenced by: '<S10>/Dead Zone'

  0.01F,

  // Computed Parameter: Integrator_gainval
  //  Referenced by: '<S18>/Integrator'

  0.01F,

  // Computed Parameter: Integrator_IC
  //  Referenced by: '<S18>/Integrator'

  0.0F,

  // Computed Parameter: TSamp_WtEt
  //  Referenced by: '<S20>/TSamp'

  100.0F,

  // Computed Parameter: UD_InitialCondition
  //  Referenced by: '<S20>/UD'

  0.0F,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S10>/Saturation1'

  0.5F,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S10>/Saturation1'

  -0.5F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S10>/Gain2'

  0.0F,

  // Computed Parameter: DiscreteTimeIntegrator_gainval
  //  Referenced by: '<S10>/Discrete-Time Integrator'

  0.05F,

  // Computed Parameter: DiscreteTimeIntegrator_IC
  //  Referenced by: '<S10>/Discrete-Time Integrator'

  0.0F,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S10>/Saturation'

  0.5F,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S10>/Saturation'

  -0.5F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S3>/Constant1'

  0.0F,

  // Computed Parameter: Gain2_Gain_n
  //  Referenced by: '<S11>/Gain2'

  -1.0F,

  // Computed Parameter: Integrator_gainval_e
  //  Referenced by: '<S21>/Integrator'

  0.01F,

  // Computed Parameter: Integrator_IC_j
  //  Referenced by: '<S21>/Integrator'

  0.0F,

  // Computed Parameter: Filter_gainval
  //  Referenced by: '<S21>/Filter'

  0.01F,

  // Computed Parameter: Filter_IC
  //  Referenced by: '<S21>/Filter'

  0.0F,

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S11>/Gain4'

  -0.0F,

  // Computed Parameter: Saturation_UpperSat_h
  //  Referenced by: '<S11>/Saturation'

  1.0F,

  // Computed Parameter: Saturation_LowerSat_e
  //  Referenced by: '<S11>/Saturation'

  -1.0F,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S11>/Gain3'

  1.0F,

  // Computed Parameter: Integrator_gainval_d
  //  Referenced by: '<S22>/Integrator'

  0.01F,

  // Computed Parameter: Integrator_IC_o
  //  Referenced by: '<S22>/Integrator'

  0.0F,

  // Computed Parameter: Filter_gainval_d
  //  Referenced by: '<S22>/Filter'

  0.01F,

  // Computed Parameter: Filter_IC_j
  //  Referenced by: '<S22>/Filter'

  0.0F,

  // Computed Parameter: Gain5_Gain
  //  Referenced by: '<S11>/Gain5'

  0.0F,

  // Computed Parameter: Saturation1_UpperSat_m
  //  Referenced by: '<S11>/Saturation1'

  1.0F,

  // Computed Parameter: Saturation1_LowerSat_d
  //  Referenced by: '<S11>/Saturation1'

  -1.0F,

  // Computed Parameter: Gain_Gain_o
  //  Referenced by: '<S11>/Gain'

  2.0F,

  // Computed Parameter: Gain6_Gain
  //  Referenced by: '<S11>/Gain6'

  -2.0F,

  // Computed Parameter: Saturation2_UpperSat
  //  Referenced by: '<S11>/Saturation2'

  1.0F,

  // Computed Parameter: Saturation2_LowerSat
  //  Referenced by: '<S11>/Saturation2'

  -1.0F,

  // Computed Parameter: Gain1_Gain_n
  //  Referenced by: '<S11>/Gain1'

  2.0F,

  // Computed Parameter: Gain7_Gain
  //  Referenced by: '<S11>/Gain7'

  -2.0F,

  // Computed Parameter: Saturation3_UpperSat
  //  Referenced by: '<S11>/Saturation3'

  1.0F,

  // Computed Parameter: Saturation3_LowerSat
  //  Referenced by: '<S11>/Saturation3'

  -1.0F,

  // Computed Parameter: TSamp_WtEt_c
  //  Referenced by: '<S19>/TSamp'

  100.0F,

  // Computed Parameter: UD_InitialCondition_n
  //  Referenced by: '<S19>/UD'

  0.0F,

  // Computed Parameter: Integrator_gainval_i
  //  Referenced by: '<S17>/Integrator'

  0.01F,

  // Computed Parameter: Integrator_IC_b
  //  Referenced by: '<S17>/Integrator'

  0.0F,

  // Computed Parameter: Delay_DelayLength
  //  Referenced by: '<Root>/Delay'

  1U,

  // Computed Parameter: UD_DelayLength
  //  Referenced by: '<S20>/UD'

  1U,

  // Computed Parameter: Delay_DelayLength_n
  //  Referenced by: '<S4>/Delay'

  1U,

  // Computed Parameter: UD_DelayLength_d
  //  Referenced by: '<S19>/UD'

  1U,

  // Computed Parameter: Delay_DelayLength_g
  //  Referenced by: '<S14>/Delay'

  1U
};

//
// File trailer for generated code.
//
// [EOF]
//
