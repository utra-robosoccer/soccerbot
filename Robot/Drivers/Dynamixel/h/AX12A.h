/**
  ******************************************************************************
  * @file    AX12A.h
  * @author  Tyler
  * @brief   This file provides interfaces for AX12A-specific functions
  *
  * @defgroup AX12AHeader Header
  * @ingroup  AX12A
  * @{
  ******************************************************************************
  */




#ifndef __AX12A__
#define __AX12A__

#ifdef __cplusplus
extern "C" {
#endif




/********************************* Includes **********************************/
#include "Dynamixel_Types.h"
#include "DynamixelProtocolV1_IO.h"




/********************************* Constants *********************************/
// Value limit definitions
extern const uint8_t AX12A_MAX_VELOCITY;

// Default register values
extern const uint8_t AX12A_DEFAULT_BAUD_RATE;
extern const uint16_t AX12A_DEFAULT_CCW_ANGLE_LIMIT;
extern const uint8_t AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT;
extern const uint8_t AX12A_DEFAULT_CW_COMPLIANCE_MARGIN;
extern const uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN;
extern const uint8_t AX12A_DEFAULT_CW_COMPLIANCE_SLOPE;
extern const uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE;
extern const uint16_t AX12A_DEFAULT_PUNCH;




/***************************** Function prototypes ***************************/
void AX12A_SetCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceMargin); // (RAM)
void AX12A_SetCCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceMargin); // (RAM)
void AX12A_SetCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceSlope); // (RAM)
void AX12A_SetCCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceSlope); // (RAM)

// Interfaces for previously-defined functions
void AX12A_SetComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceSlope);
void AX12A_SetComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceSlope);

/**
 * @}
 */
/* end AX12AHeader */

#ifdef __cplusplus
}
#endif

#endif /* __DYNAMIXEL_AX-12A_H__ */
