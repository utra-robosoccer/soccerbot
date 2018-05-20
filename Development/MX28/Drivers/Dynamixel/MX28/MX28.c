/* This file implements MX28-specific functions.
 *
 * Author: Tyler
 */

/********************************* Includes ************************************/
#include "MX28.h"

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
// TODO
void MX28_SetMultiTurnOffset(Dynamixel_HandleTypeDef* hdynamixel, int16_t offset){

}

// TODO
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

void MX28_SetGoalAcceleration(Dynamixel_HandleTypeDef* hdynamixel, double goalAcceleration){
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
// TODO
void MX28_EnterMultiTurnMode(Dynamixel_HandleTypeDef* hdynamixel){

}
