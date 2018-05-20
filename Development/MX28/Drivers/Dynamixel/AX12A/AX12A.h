/****************** Drivers for Dynamixel AX-12A Servo Motor ******************/

/******************** Define to prevent recursive inclusion *******************/
#ifndef __AX12A__
#define __AX12A__

/********************************** Includes **********************************/
#include "../DynamixelV1IO.h"

/*********************************** Macros ***********************************/
/* Value limit definitions */
#define AX12A_MAX_VELOCITY          114	// Maximum angular velocity (RPM)

/* Register addresses */
#define AX12A_REG_CW_COMPLIANCE_MARGIN	0x1A		// Clockwise compliance margin register
#define AX12A_REG_CCW_COMPLIANCE_MARGIN	0x1B		// Counter-clockwise compliance margin register
#define AX12A_REG_CW_COMPLIANCE_SLOPE	0x1C		// Clockwise compliance slope register
#define AX12A_REG_CCW_COMPLIANCE_SLOPE	0x1D		// Counter-clockwise compliance slope register

/* Default register values */
#define AX12A_DEFAULT_BAUD_RATE					0x01	// Default baud rate register setting
#define AX12A_DEFAULT_CCW_ANGLE_LIMIT			0x03FF	// Default counter-clockwise angle limit
#define AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT		0xBE	// Default permitted maximum voltage (0xBE = 140 -> 14.0 V)
#define AX12A_DEFAULT_CW_COMPLIANCE_MARGIN		0x01	// Default clockwise compliance margin (position error)
#define AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN		0x01	// Default counter-clockwise compliance margin (position error)
#define AX12A_DEFAULT_CW_COMPLIANCE_SLOPE		0x20	// Default clockwise compliance slope (torque near goal position)
#define AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE		0x20	// Default counter-clockwise compliance slope (torque near goal position)
#define AX12A_DEFAULT_PUNCH						0x0020	// Default punch


/***************************** Function prototypes ****************************/
void AX12A_SetCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceMargin); // (RAM)
void AX12A_SetCCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceMargin); // (RAM)
void AX12A_SetCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceSlope); // (RAM)
void AX12A_SetCCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceSlope); // (RAM)

// Interfaces for previously-defined functions
void AX12A_SetComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceSlope);
void AX12A_SetComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceSlope);

#endif /* __DYNAMIXEL_AX-12A_H__ */
