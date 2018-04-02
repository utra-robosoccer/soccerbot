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

/* Instruction set definitions. */
// The intersection of protocol version 1.0 and version 2.0:
// (Note that the AX12A is only compatible with version 1.0, while
// the MX-28 is compatible with both protocols. Also, note that
// protocol version 1.0 is a proper subset of protocol version 2.0.)
const uint8_t INST_PING					= 0x01;	    // Gets a status packet
const uint8_t INST_READ_DATA			= 0x02;	    // Reads data from a motor register
const uint8_t INST_WRITE_DATA			= 0x03;	    // Writes data for immediate execution
const uint8_t INST_REG_WRITE			= 0x04;	    // Registers an instruction to be executed at a later time
const uint8_t INST_ACTION				= 0x05;	    // Triggers instructions registered by INST_REG_WRITE
const uint8_t INST_RESET				= 0x06;	    // Resets the control tables of the Dynamixel actuator(s) specified
const uint8_t INST_SYNC_WRITE			= 0x83;	    // Writes on a specified address with a specified data length on multiple devices

// Instructions that are only in protocol version 2.0
const uint8_t V2_INST_REBOOT 			= 0x07;		// Reboots the actuators
const uint8_t V2_INST_RETURN 			= 0x55;		// Return instruction for the instruction packet
const uint8_t V2_INST_SYNC_READ 		= 0x82;		// Reads from a specified address with a specified data length on multiple devices
const uint8_t V2_INST_BULK_READ 		= 0x92;		// Reads from various addresses with various data lengths on multiple devices
const uint8_t V2_INST_BULK_WRITE 		= 0x93;		// Writes on various addresses with various data lengths on multiple devices

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
const uint8_t DEFAULT_BAUD_RATE				= 0x01;	    // Default baud rate
const uint8_t DEFAULT_RETURN_DELAY			= 0xFA;	    // Default time motor waits before returning status packet (microseconds)
const uint8_t DEFAULT_TORQUE_ENABLE			= 0x00;	    // Default motor power state
const uint8_t DEFAULT_LED_ENABLE			= 0x00;	    // Default LED state
const uint8_t DEFAULT_STATUS_RETURN_LEVEL	= 0x02;	    // Default condition(s) under which a status packet will be returned (all)




/********************************* Functions *********************************/
// Constructor and destructor
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
		this -> lastPosition = 0;
		this -> lastVelocity = -1;
		this -> lastLoad = 0;
		this -> lastVoltage = -1;
		this -> lastTemperature = 0;
		this -> uartHandle = motorInitData -> uartHandle;
		this -> dataDirPort = motorInitData -> dataDirPort;
		this -> dataDirPinNum = motorInitData -> dataDirPinNum;
		this -> isJointMode = 1;

		/* The following must be set properly in the constructor
		 * of the derived class. This is because different motors
		 * have different addresses for equivalent registers (in general).
		 */
		this -> angleResolution = 0;

		this -> REG_ID = 0;
		this -> REG_BAUD_RATE = 0;
		this -> REG_GOAL_POSITION = 0;
		this -> REG_GOAL_VELOCITY = 0;
		this -> REG_TORQUE_ENABLE = 0;
		this -> REG_RETURN_DELAY_TIME = 0;
		this -> REG_LED_ENABLE = 0;
		this -> REG_CURRENT_POSITION = 0;
		this -> REG_CURRENT_VELOCITY = 0;
		this -> REG_CURRENT_LOAD = 0;
}

Dynamixel::~Dynamixel() {
	// TODO Auto-generated destructor stub
}




// Other low-level motor commands (seldom used)
// TODO: TEST ping
uint8_t Dynamixel::ping(){
	/* Used only for returning a status packet or checking the existence of a motor
	 * with a specified ID. Does not command any operations.
	 *
	 * Arguments: none
	 *
	 * Returns: motor ID seen in status packet
	 */

	/* Define array for transmission and reception. */
	uint8_t arrTransmit[6];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[2] = this -> id; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_PING; // PING instruction
	arrTransmit[5] = computeChecksum(arrTransmit, 6);

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(this -> uartHandle, arrTransmit, 6, TRANSMIT_TIMEOUT);

	/* Set data direction for receive. */
	__DYNAMIXEL_RECEIVE(this -> dataDirPort, this -> dataDirPinNum);

	/* Receive. */
	HAL_UART_Receive(this -> uartHandle, arrTransmit, 6, RECEIVE_TIMEOUT);
	return(arrTransmit[2]);
}

// TODO: Test regWrite
void Dynamixel::regWrite(uint8_t arrSize, uint8_t writeAddr, uint8_t param1, uint8_t param2){
	/* Implementation of REG WRITE instruction with 1 or 2 parameters.
	 *
	 * Arguments: writeAddr, the starting address for where the data is to be written
	 * 			  param1, the first parameter
	 * 			  param2, the second parameter
	 *
	 * Returns: none
	 */

	/* Define arrays for transmission. */
	uint8_t arrTransmit[arrSize];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[2] = this -> id; // Motor ID
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_REG_WRITE; // REG WRITE instruction
	arrTransmit[5] = writeAddr;
	arrTransmit[7] = (arrSize == 8) ? computeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = computeChecksum(arrTransmit, arrSize);
	}

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(this -> uartHandle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);
}

// TODO: Test action
void Dynamixel::action(){
	/* Implements the ACTION instruction. This triggers the instruction registered by the REG_WRITE
	 * instruction. This way, time delays can be reduced for the concurrent motion of several
	 * motors.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	/* Define arrays for transmission and reception. */
	uint8_t arrTransmit[6];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[2] = this -> id; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_ACTION; // ACTION instruction
	arrTransmit[5] = computeChecksum(arrTransmit, 6);

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(this -> uartHandle, arrTransmit, 6, TRANSMIT_TIMEOUT);
}

// TODO: test reset
void Dynamixel::reset(){
	/* Resets the control table values of the motor to the Factory Default Value settings.
	 * Note that post-reset, motor ID will be 1. Thus, if several motors with ID 1 are
	 * connected on the same bus, there will not be a way to assign them unique IDs without
	 * first disconnecting them.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	/* Define array for transmission. */
	uint8_t arrTransmit[6];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = this -> id; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_RESET; // Reset instruction
	arrTransmit[5] = computeChecksum(arrTransmit, 6);

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(this -> uartHandle, arrTransmit, 6, TRANSMIT_TIMEOUT);

	/* Set ID property in the object to the default ID that will be applied to the
	 * motor by its internal controller so that the two match. */
	this -> id = DEFAULT_ID;

	/* Wait for motor to finish resetting. */
	HAL_Delay(500); // Should find a way of polling the motor here
}




// Setters (use the WRITE DATA instruction)
// TODO: test setID
void Dynamixel::setID(uint8_t ID){
	/* Sets the ID (identification number) for the current motor.
	 * Note that the instruction will be broadcasted using the current ID.
	 * As such, if the ID is not known, the motor ID should be initialized
	 * to the broadcast ID (0xFE) in the Dynamixel_Init function.
	 *
	 * Default value: 1
	 *
	 * Arguments: ID, the number between 0 and 252 or equal to 254 to identify the motor.
	 * 			  	If 0xFE (254), any messages broadcasted to that ID will be broadcasted
	 * 			  	to all motors.
	 *
	 * Returns: none
	 */


	/* Compute validity. */
	if((ID == 253) || (ID > 254)){
		ID = DEFAULT_ID;
	}

	/* Write data to motor. */
	dataWriter(8, REG_ID, ID, 0);
	this -> id = ID;
}

void Dynamixel::setReturnDelayTime(double microSec){
	/* Sets the time, in microseconds, that the motor should wait before returning a status packet.
	 *
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

//TODO: min/max angle might need to be implemented differently for the AX12
// than the MX28 since the MX28 has an "operating mode" register to determine
// whether it operates in position control mode, velocity control mode, and others
//void Dynamixel::setMinAngle(double minAngle); // (EEPROM) -- CW angle limit
//void Dynamixel::setMaxAngle(double maxAngle); // (EEPROM) -- CCW angle limit

// TODO: implement setTemperatureLimit
// void Dynamixel::setTemperatureLimit();

// TODO: implement voltage limit setters
//void Dynamixel::setHighestVoltageLimit(double highestVoltage); // (EEPROM)
//void Dynamixel::setLowestVoltageLimit(double lowestVoltage); // (EEPROM)

void Dynamixel::setTorqueEnable(uint8_t isEnabled){
	/* Enables or disables torque for current motor.
	 *
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




// Getters (use READ DATA instruction)
// TODO: test getPosition
void Dynamixel::getPosition(){
	/* Reads what the current position of the motor is.
	 *
	 * Writes the results to this -> lastPosition
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint16_t retVal = dataReader(REG_CURRENT_POSITION, 2);

	/* Parse data and write it into motor handle. */
	this -> lastPosition = (float)(retVal * MAX_ANGLE / this -> angleResolution);
}

void Dynamixel::getVelocity(){
	/* Reads what the current velocity of the motor is.
	 *
	 * Writes the results to this -> lastVelocity
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint16_t retVal = dataReader(REG_CURRENT_VELOCITY, 2);

	/* Parse data and write it into motor handle. */
	uint16_t modifier;
	if(this -> isJointMode){
		modifier = this -> angleResolution;
	}
	else{
		modifier = 2 * (this -> angleResolution) + 1;
	}
	this -> lastVelocity = (float)(retVal / modifier * 114);
}

void Dynamixel::getLoad(){
	/* Reads present load from the motor.
	 *
	 * Load is a percentage of the maximum torque. A value in 0-1023
	 * gets translated into a counterclockwise load, and value of 1024-2047
	 * gets translated into a clockwise load.
	 *
	 * Writes the torque percentage to this -> lastLoad
	 * A positive value indicates a CCW load, and a negative value indicates a CW
	 * load.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint16_t retVal = dataReader(REG_CURRENT_LOAD, 2);

	/* Parse data and write it into motor handle. */
	if(retVal > 1023){
		retVal = -1.0 * (retVal - 1023);
	}
	this -> lastLoad = (uint8_t)(retVal / 1023 * 100);
}

// TODO: implement these
//void Dynamixel::getVoltage(){
//
//}
//
//void Dynamixel::getTemperature(){
//
//}
//
//uint8_t Dynamixel::isRegistered(){
//
//}
//
//bool Dynamixel::isMoving(){
//
//}

