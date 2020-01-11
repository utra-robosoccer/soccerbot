/* This file implements MX28-specific functions.
 *
 * Author: Tyler
 */

/********************************* Includes ************************************/
#include "MX28.h"

/********************************** Externs ************************************/
extern void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs);
extern void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, float minAngle); // (EEPROM)
extern void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, float maxAngle); // (EEPROM)

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
void MX28_SetMultiTurnOffset(Dynamixel_HandleTypeDef* hdynamixel, int16_t offset){
	/* For an actuator not in multi-turn mode, this has no effect.
	 *
	 * For an actuator in multi-turn mode, this applies a tunable offset to all positions.
	 * That is, it allows you to change where the actuator considers position 0 to be.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  offset, the signed offset argument indicating the offset to program
	 *
	 * Returns: none
	 */

	if(offset > 28672){
		offset = 28672;
	}
	else if(offset < -28672){
		offset = -28672;
	}

	uint8_t args[3] = {MX28_REG_MULTI_TURN_OFFSET, offset & 0xFF, (offset >> 16) & 0xFF};
	Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
}

// TODO. Challenge with this is it means we need to track resolution divider in the
// handle so that we can compute position properly in the position functions
void MX28_SetResolutionDivider(Dynamixel_HandleTypeDef* hdynamixel, uint8_t divider){

}

void MX28_SetDGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t DGain){
	/* Sets the value of the derivative gain used in the motor's PID controller.
	 * kD = DGain / 250
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  DGain, the derivative gain parameter
	 *
	 * Returns: none
	 */

	if(hdynamixel -> _motorType == MX28TYPE){
		uint8_t args[2] = {MX28_REG_D_GAIN, DGain};
		Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
	}
}

void MX28_SetIGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t IGain){
	/* Sets the value of the integral gain used in the motor's PID controller.
	 * kI = IGain * 125/256
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  IGain, the integral gain parameter
	 *
	 * Returns: none
	 */

	if(hdynamixel -> _motorType == MX28TYPE){
		uint8_t args[2] = {MX28_REG_I_GAIN, IGain};
		Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
	}
}

void MX28_SetPGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t PGain){
	/* Sets the value of the proportional gain used in the motor's PID controller.
	 * kP = PGain / 8
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  PGain, the proportional gain parameter
	 *
	 * Returns: none
	 */

	if(hdynamixel -> _motorType == MX28TYPE){
		uint8_t args[2] = {MX28_REG_P_GAIN, PGain};
		Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
	}
}

void MX28_SetGoalAcceleration(Dynamixel_HandleTypeDef* hdynamixel, float goalAcceleration){
	/* Sets the goal acceleration. The argument should be in units of degree/s^2
	 *
	 * Special: goalAcceleration of 0 means no control over accel (uses max accel of motor)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 		      goalAcceleration, the target acceleration in degree/s^2
	 */

	if(hdynamixel -> _motorType == MX28TYPE){
		if(goalAcceleration > 2180){
			goalAcceleration = 2180;
		}
		if(goalAcceleration < 0){
			goalAcceleration = 0;
		}

		uint8_t accelArg = (uint8_t)(goalAcceleration / 8.583);

		uint8_t args[2] = {MX28_REG_GOAL_ACCELERATION, accelArg};
		Dynamixel_DataWriter(hdynamixel, args, sizeof(args));
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
void MX28_EnterMultiTurnMode(Dynamixel_HandleTypeDef* hdynamixel){
	/* Activates multi-turn mode, which allows the actuator to have a range of
	 * controllable position values from -28672 to 28672.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	if(hdynamixel -> _motorType == MX28TYPE){
		Dynamixel_SetCWAngleLimit(hdynamixel, 4095);
		Dynamixel_SetCCWAngleLimit(hdynamixel, 4095);
	}
}
