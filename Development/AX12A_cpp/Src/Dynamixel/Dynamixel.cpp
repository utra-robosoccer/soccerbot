/*
 * Dynamixel.cpp+
 *
 *  Created on: Mar 16, 2018
 *      Author: Tyler Gamvrelis
 */




/********************************** Includes **********************************/
#include "Dynamixel.h"




/********************************* Constants **********************************/
/* Communications */
const uint TRANSMIT_TIMEOUT = 10;
const uint RECEIVE_TIMEOUT = 10;

/* Value limit definitions. */
const double MAX_VELOCITY            = 114;		// Maximum angular velocity (RPM)
const double MIN_VELOCITY            = 1;		// Minimum angular velocity (RPM)
const uint16_t MAX_ANGLE             = 300;		// Maximum angular position (joint mode)
const uint8_t MIN_ANGLE              = 0;		// Minimum angular position (joint mode)
const uint8_t MAX_TORQUE             = 100;		// Maximum torque (percent of maximum)
const uint8_t MIN_TORQUE             = 0;		// Minimum torque (percent of maximum)
const uint8_t MAX_VOLTAGE            = 14;		// Maximum operating voltage
const uint8_t MIN_VOLTAGE            = 6;		// Minimum operating voltage
const uint16_t MAX_PUNCH             = 1023;		// Maximum punch (proportional to minimum current)
const uint8_t MIN_PUNCH              = 0;		// Minimum punch (proportional to minimum current)

/* Default register value definitions. */
const uint8_t BROADCAST_ID					= 0xFE;	    // Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus)
const uint8_t DEFAULT_ID					= 0x01;	    // Default motor ID
const uint8_t DEFAULT_RETURN_DELAY			= 0xFA;	    // Default time motor waits before returning status packet (microseconds)
const uint8_t DEFAULT_TORQUE_ENABLE			= 0x00;	    // Default motor power state
const uint8_t DEFAULT_LED_ENABLE			= 0x00;	    // Default LED state
const uint8_t DEFAULT_STATUS_RETURN_LEVEL	= 0x02;	    // Default condition(s) under which a status packet will be returned (all)

/********************************* Functions *********************************/
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
		this -> id = motorInitData -> id;
		this -> baudRate = motorInitData -> uartHandle -> Init.BaudRate;
		this -> lastPosition = -1;
		this -> lastVelocity = -1;
		this -> lastLoad = -1;
		this -> lastLoadDirection = -1;
		this -> lastVoltage = -1;
		this -> lastTemperature = -1;
		this -> uartHandle = motorInitData -> uartHandle;
		this -> dataDirPort = motorInitData -> dataDirPort;
		this -> dataDirPinNum = motorInitData -> dataDirPinNum;
		this -> isJointMode = 1;

		/* The following must be set properly in the constructor
		 * of the derived class.
		 */
		this -> REG_GOAL_POSITION = 0;
		this -> REG_GOAL_VELOCITY = 0;
		this -> REG_TORQUE_ENABLE = 0;
		this -> REG_RETURN_DELAY_TIME = 0;
		this -> REG_LED_ENABLE = 0;
}

Dynamixel::~Dynamixel() {
	// TODO Auto-generated destructor stub
}

void Dynamixel::setReturnDelayTime(double microSec){
	/* Sets the time, in microseconds, that the motor should wait before returning a status packet.
	 *
	 * Instruction register address: 0x05(EEPROM)
	 * Default value: 250 (0xFA)
	 *
	 * Arguments: microSec, the time in microseconds to delay. Arguments in range [2, 508] are valid. Default: 500
	 *
	 * Returns: none
	 */

	/* Compute validity. */
	uint8_t motor_data;
	if((microSec < 2) || (microSec > 508)){
		motor_data = DEFAULT_RETURN_DELAY;
	}
	else{
		motor_data = (uint8_t)(microSec / 2);
	}

	/* Write data to motor. */
	dataWriter(8, this -> REG_RETURN_DELAY_TIME, motor_data, 0);
}

void Dynamixel::setTorqueEnable(uint8_t isEnabled){
	/* Enables or disables torque for current motor.
	 *
	 * Instruction register address: 0x18 (RAM)
	 * Default value: 0x00
	 *
	 * Arguments: isEnabled, if 1, then generates torque by impressing power to the motor
	 * 			  			 if 0, then interrupts power to the motor to prevent it from generating torque
	 *
	 * Returns: none
	 */

	/* Evaluate argument validity. */
	if((isEnabled != 1) && (isEnabled != 0)){
		isEnabled = DEFAULT_TORQUE_ENABLE;
	}

	/* Write data to motor. */
	dataWriter(8, this -> REG_TORQUE_ENABLE, isEnabled, 0);
}

void Dynamixel::setLEDEnable(uint8_t isEnabled){
	/* Toggles the motor LED.
	 *
	 * Register address: 0x19
	 * Default value: 0x00
	 *
	 * Arguments: isEnabled, 0 if LED off
	 * 			  			 1 if LED on
	 *
	 * Returns: none
	 */

	/* Evaluate argument validity. */
	if((isEnabled != 1) && (isEnabled != 0)){
		isEnabled = DEFAULT_LED_ENABLE;
	}

	/* Write data to motor. */
	dataWriter(8, this -> REG_LED_ENABLE, isEnabled, -1);
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
	 * Arguments: angle, the desired angular position. Arguments between 0 and 300 are valid
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
	dataWriter(9, this -> REG_GOAL_POSITION, lowByte, highByte);
}

void Dynamixel::setGoalVelocity(double goalVelocity){
	/* Sets the goal velocity of the motor in RAM.
	 * Low byte is 0x20 in motor RAM, high byte is 0x21 in motor RAM.
	 *
	 * Instruction register address: 0x20 (RAM)
	 * Default value: none
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  velocity, the goal velocity in RPM. Arguments of 0-114 are valid when in joint mode.
	 * 			  0 corresponds to MAX motion in joint mode, and minimum motion in wheel mode
	 * 			  In wheel mode, negative arguments correspond to CW rotation
	 *
	 * Returns: none
	 */

	/* Translate the position from RPM into a 10-bit number. */
	uint16_t normalized_value = 0;
	if(this -> isJointMode){

		/* Check for input validity. If input not valid, replace goalAngle with closest
		 * valid value to ensure code won't halt. */
		if((goalVelocity < MIN_VELOCITY) || (goalVelocity > MAX_VELOCITY)){
			if(goalVelocity > MIN_VELOCITY){
				goalVelocity = MAX_VELOCITY;
			}
			else{
				goalVelocity = MIN_VELOCITY;
			}
		}
	}

	normalized_value = (uint16_t)(goalVelocity / MAX_VELOCITY * 1023);
	if(goalVelocity < 0){
		normalized_value = ((uint16_t)((goalVelocity * -1) / MAX_VELOCITY * 1023)) | 0b000010000000000;
	}

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal velocity
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal velocity

	/* Write data to motor. */
	dataWriter(9, this -> REG_GOAL_VELOCITY, lowByte, highByte); // Implemented in derived classes
}
