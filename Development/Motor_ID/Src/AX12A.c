/* This file implements MX28-specific functions.
 *
 * Author: Tyler
 */

/********************************* Includes ************************************/
#include "AX12A.h"

/*********************************** Externs **********************************/
extern void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs);

/******************************** Functions ************************************/
/*******************************************************************************/
/*	Setter helper functions													   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void AX12A_SetCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceMargin){
	/* Sets the clockwise compliance margin for the current motor.
	 * The compliance margin is the acceptable error between the current position and goal position.
	 * The greater the value, the more error is acceptable.
	 *
	 * Instruction register address: 0x1A (RAM)
	 * Default value: 0x01
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  CWcomplianceMargin, the acceptable error between the current and goal position. Arguments in range [0, 255]
	 *
	 * Returns: none
	 */

	/* Write data to motor. */
	if(hdynamixel -> _motorType == AX12ATYPE){
		uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_MARGIN, CWcomplianceMargin};
		Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
	}
}

void AX12A_SetCCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceMargin){
	/* Sets the counter-clockwise compliance margin for the current motor.
	 * The compliance margin is the acceptable error between the current position and goal position.
	 * The greater the value, the more error is acceptable.
	 *
	 * Instruction register address: 0x1B (RAM)
	 * Default value: 0x01
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  CCWcomplianceMargin, the acceptable error between the current and goal position. Arguments in range [0, 255]
	 *
	 * Returns: none
	 */

	/* Write data to motor. */
	if(hdynamixel -> _motorType == AX12ATYPE){
		uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_MARGIN, CCWcomplianceMargin};
		Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
	}
}

void AX12A_SetCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceSlope){
	/* Sets the clockwise compliance slope for the current motor, which sets the level of torque near the goal position.
	 * The higher the value, the more flexibility that is obtained. That is, a high value
	 * means that as the goal position is approached, the torque will be significantly reduced. If a low
	 * value is used, then as the goal position is approached, the torque will not be reduced all that much.
	 *
	 * Instruction register address: 0x1C (RAM)
	 * Default value: 0x20
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  CWcomplianceSlope, arguments in range [1, 7], with 1 being the least flexible
	 *
	 * Returns: none
	 */

	/* Translate the step into motor data. */
	uint8_t step;

	if(CWcomplianceSlope == 1){
		step = 2;
	}
	else if(CWcomplianceSlope == 2){
		step = 4;
	}
	else if(CWcomplianceSlope == 3){
		step = 8;
	}
	else if(CWcomplianceSlope == 4){
		step = 16;
	}
	else if(CWcomplianceSlope == 5){
		step = 32;
	}
	else if(CWcomplianceSlope == 6){
		step = 64;
	}
	else if(CWcomplianceSlope == 7){
		step = 128;
	}
	else{
		step = AX12A_DEFAULT_CW_COMPLIANCE_SLOPE;
	}

	/* Write data to motor. */
	if(hdynamixel -> _motorType == AX12ATYPE){
		uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_SLOPE, step};
		Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
	}
}

// TODO: Test
void AX12A_SetCCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceSlope){
	/* Sets the counter-clockwise compliance slope for the current motor, which sets the level of torque near the goal position.
	 * The higher the value, the more flexibility that is obtained. That is, a high value
	 * means that as the goal position is approached, the torque will be significantly reduced. If a low
	 * value is used, then as the goal position is approached, the torque will not be reduced all that much.
	 *
	 * Instruction register address: 0x1D (RAM)
	 * Default value: 0x20
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  CWcomplianceSlope, arguments in range [1, 7], with 1 being the least flexible
	 *
	 * Returns: none
	 */

	/* Translate the step into motor data. */
	uint8_t step;

	if(CCWcomplianceSlope == 1){
		step = 2;
	}
	else if(CCWcomplianceSlope == 2){
		step = 4;
	}
	else if(CCWcomplianceSlope == 3){
		step = 8;
	}
	else if(CCWcomplianceSlope == 4){
		step = 16;
	}
	else if(CCWcomplianceSlope == 5){
		step = 32;
	}
	else if(CCWcomplianceSlope == 6){
		step = 64;
	}
	else if(CCWcomplianceSlope == 7){
		step = 128;
	}
	else{
		step = AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE;
	}

	/* Write data to motor. */
	if(hdynamixel -> _motorType == AX12ATYPE){
		uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_SLOPE, step};
		Dynamixel_DataWriter(hdynamixel, arr, sizeof(arr));
	}
}




/*******************************************************************************/
/*	Interfaces for previously-defined functions								   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
void AX12A_SetComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceSlope){
	/* Sets CW and CCW compliance slope.
	 *
	 * "Sets the compliance slope for the current motor, which sets the level of torque near the goal position.
	 * The higher the value, the more flexibility that is obtained. That is, a high value
	 * means that as the goal position is approached, the torque will be significantly reduced. If a low
	 * value is used, then as the goal position is approached, the torque will not be reduced all that much."
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  complianceSlope, arguments in range [1, 7], with 1 being the least flexible
	 *
	 * Returns: none
	 */

	AX12A_SetCWComplianceSlope(hdynamixel, complianceSlope);
	AX12A_SetCCWComplianceSlope(hdynamixel, complianceSlope);
}

void AX12A_SetComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t complianceMargin){
	/* Sets CW and CCW compliance margin.
	 *
	 * "Sets the compliance margin for the current motor.
	 * The compliance margin is the acceptable error between the current position and goal position.
	 * The greater the value, the more error is acceptable."
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  complianceMargin, the acceptable error between the current and goal position. Arguments in range [0, 255]
	 *
	 * Returns: none
	 */

	AX12A_SetCWComplianceMargin(hdynamixel, complianceMargin);
	AX12A_SetCCWComplianceMargin(hdynamixel, complianceMargin);
}
