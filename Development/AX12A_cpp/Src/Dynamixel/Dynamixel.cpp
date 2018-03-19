/*
 * Dynamixel.cpp+
 *
 *  Created on: Mar 16, 2018
 *      Author: Tyler Gamvrelis
 */

#include "Dynamixel.h"

Dynamixel::Dynamixel(MotorInitData* motorInitData) {
		/* Initializes the motor object.
		 *
		 * Arguments: motorInitData, a struct with the following data fields:
		 *
		 * 			  	    id, the ID the motor has. Note that this function will not set
		 * 			  	      	the ID in case there are multiple actuators on the same bus
		 * 			  	    uart, the handle to the UART that will be used to
		 * 			          	communicate with this motor
		 * 			  	    DataDirPort, the pointer to the port that the data direction pin
		 * 			  	      	for the motor is on
		 * 			  	    DataDirPinNum, the number corresponding to the pin that controls
		 * 			          	data direction (a power of two, e.g. 2^0 for pin 0, 2^15 for pin 15)
		 *
		 * Returns: none
		 */

		/* Set fields in motor handle. */
		this -> id = motorInitData.id;
		this -> baudRate = motorInitData.uartHandle->Init.BaudRate;
		this -> lastPosition = -1;
		this -> lastVelocity = -1;
		this -> lastLoad = -1;
		this -> lastLoadDirection = -1;
		this -> lastVoltage = -1;
		this -> lastTemperature = -1;
		this -> uartHandle = motorInitData.uartHandle;
		this -> dataDirPort = motorInitData.dataDirPort;
		this -> dataDirPinNum = motorInitData.dataDirPinNum;
}

Dynamixel::~Dynamixel() {
	// TODO Auto-generated destructor stub
}

int Dynamixel::Init(){
	return -1;
}

void Dynamixel::setGoalPosition(double goalAngle){
	/* Takes a double between 0 and 300, encodes this position in an
	 * upper and low hex byte pair (with a maximum of 1023 as defined in the AX-12
	 * user manual), and sends this information (along with requisites) over UART.
	 * Low byte is 0x1E in motor RAM, high byte is 0x1F in motor RAM.
	 *
	 * Instruction register address: 0x1E (RAM)
	 * Default value: none
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  angle, the desired angular position. Arguemnts between 0 and 300 are valid
	 *
	 * Returns: none
	 */

	/* Check for input validity. If input not valid, replace goalAngle with closest
	 * valid value to ensure code won't halt. */
	if((goalAngle < MIN_ANGLE) || (goalAngle > MAX_ANGLE)){
		if(goalAngle > MIN_ANGLE){
			goalAngle = MAX_ANGLE;
		}
		else{
			goalAngle = MIN_ANGLE;
		}
	}

	/* Translate the angle from degrees into a 10-bit number. */
	uint16_t normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 1023); // TODO: remove the dependence on 1023 here,
																		  // resolution varies between motors

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal position
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal position

	/* Write data to motor. */
	this -> dataWriter(9, this -> REG_GOAL_POSITION, lowByte, highByte);
}
