/**
  ******************************************************************************
  * @file    MX28.h
  * @author  Tyler
  * @brief   This file provides interfaces for MX28-specific functions.
  ******************************************************************************
  */

/******************** Define to prevent recursive inclusion *******************/
#ifndef __MX28_H__
#define __MX28_H__

/********************************** Includes **********************************/
#include "../Dynamixel_HandleTypeDef.h"

/*********************************** Macros ***********************************/
/* Value limit definitions */
#define MX28_MAX_VELOCITY          117	/**< Maximum angular velocity (RPM) */

/* Register addresses */
#define MX28_REG_MULTI_TURN_OFFSET				0x14	/**< Register to fine-tune "0" position */
#define MX28_REG_RESOLUTION_DIVIDER				0x16	/**< Register to change how many bits of resolution are used */
#define MX28_REG_D_GAIN							0x1A	/**< Derivative gain register */
#define MX28_REG_I_GAIN							0x1B	/**< Integral gain register */
#define MX28_REG_P_GAIN							0x1C	/**< Proportional gain register */
#define MX28_REG_GOAL_ACCELERATION				0x49	/**< Goal acceleration register */

/* Default register value definitions */
#define MX28_DEFAULT_BAUD_RATE		 			0x22	/**< Default baud rate register setting */
#define MX28_DEFAULT_CCW_ANGLE_LIMIT			0x0FFF	/**< Default counter-clockwise angle limit */
#define MX28_DEFAULT_HIGHEST_VOLTAGE_LIMIT		0xA0	/**< Default permitted maximum voltage (0xA0 = 160 -> 16.0 V) */
#define MX28_DEFAULT_D_GAIN						0x08	/**< Default derivative gain parameter value */
#define MX28_DEFAULT_I_GAIN						0x00	/**< Default integral gain parameter value */
#define MX28_DEFAULT_P_GAIN						0x08 	/**< Default proportional gain parameter value */
#define MX28_DEFAULT_PUNCH						0x0000	/**< Default punch */

/***************************** Function prototypes ****************************/
// Setters (use the WRITE DATA instruction)
void MX28_SetMultiTurnOffset(Dynamixel_HandleTypeDef* hdynamixel, int16_t offset);
void MX28_SetResolutionDivider(Dynamixel_HandleTypeDef* hdynamixel, uint8_t divider);
void MX28_SetDGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t DGain);
void MX28_SetIGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t IGain);
void MX28_SetPGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t PGain);
void MX28_SetGoalAcceleration(Dynamixel_HandleTypeDef* hdynamixel, float goalAcceleration);

// Interfaces for previously-defined functions
void MX28_EnterMultiTurnMode(Dynamixel_HandleTypeDef* hdynamixel);

#endif /* __MX28_H__ */
