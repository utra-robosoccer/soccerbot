/**
  ******************************************************************************
  * @file    MX28.h
  * @author  Tyler
  * @brief   This file provides interfaces for MX28-specific functions
  *
  * @defgroup MX28Header Header
  * @ingroup  MX28
  * @{
  ******************************************************************************
  */




#ifndef __MX28_H__
#define __MX28_H__

#ifdef __cplusplus
extern "C" {
#endif




/********************************* Includes **********************************/
#include "Dynamixel_Types.h"
#include "DynamixelProtocolV1_IO.h"




/********************************* Constants *********************************/
// Value limit definitions
extern const uint8_t MX28_MAX_VELOCITY;

// Default register value definitions
extern const uint8_t MX28_DEFAULT_BAUD_RATE;
extern const uint16_t MX28_DEFAULT_CCW_ANGLE_LIMIT;
extern const uint8_t MX28_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
extern const uint8_t MX28_DEFAULT_D_GAIN;
extern const uint8_t MX28_DEFAULT_I_GAIN;
extern const uint8_t MX28_DEFAULT_P_GAIN;
extern const uint16_t MX28_DEFAULT_PUNCH;




/***************************** Function prototypes ***************************/
// Setters (use the WRITE DATA instruction)
void MX28_SetMultiTurnOffset(Dynamixel_HandleTypeDef* hdynamixel, int16_t offset);
void MX28_SetResolutionDivider(Dynamixel_HandleTypeDef* hdynamixel, uint8_t divider);
void MX28_SetDGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t DGain);
void MX28_SetIGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t IGain);
void MX28_SetPGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t PGain);
void MX28_SetGoalAcceleration(Dynamixel_HandleTypeDef* hdynamixel, float goalAcceleration);

// Interfaces for previously-defined functions
void MX28_EnterMultiTurnMode(Dynamixel_HandleTypeDef* hdynamixel);

/**
 * @}
 */
/* end MX28Header */

#ifdef __cplusplus
}
#endif

#endif /* __MX28_H__ */
