/*
 * AX12A.cpp
 *
 *  Created on: Mar 16, 2018
 *      Author: Admin
 */




/********************************** Includes **********************************/
#include "AX12A.h"




/********************************* Constants *********************************/
/* Register definitions. */
const uint8_t AX12A_REG_ID 					    = 0x03;		// Motor ID register
const uint8_t AX12A_REG_BAUD_RATE				= 0x04;		// Baud rate register
const uint8_t AX12A_REG_RETURN_DELAY_TIME		= 0x05;		// Status packet return delay time register
const uint8_t AX12A_REG_CW_ANGLE_LIMIT		    = 0x06;		// Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
const uint8_t AX12A_REG_CCW_ANGLE_LIMIT		    = 0x08;		// Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte)
const uint8_t AX12A_REG_HIGH_VOLTAGE_LIMIT	    = 0x0C;		// Maximum voltage limit register
const uint8_t AX12A_REG_LOW_VOLTAGE_LIMIT		= 0x0D;		// Minimum voltage limit register
const uint8_t AX12A_REG_MAX_TORQUE			    = 0x0E;		// Maximum torque limit register (0x0E = low byte, 0x0F = high byte)
const uint8_t AX12A_REG_STATUS_RETURN_LEVEL	    = 0x10;		// Status packet return condition(s) register
const uint8_t AX12A_REG_ALARM_LED				= 0x11;		// Alarm LED condition(s) register
const uint8_t AX12A_REG_ALARM_SHUTDOWN		    = 0x12;		// Alarm shutdown condition(s) register
const uint8_t AX12A_REG_TORQUE_ENABLE 		    = 0x18;		// Motor power control register
const uint8_t AX12A_REG_LED_ENABLE			    = 0x19;		// LED control register
const uint8_t AX12A_REG_CW_COMPLIANCE_MARGIN	= 0x1A;		// Clockwise compliance margin register
const uint8_t AX12A_REG_CCW_COMPLIANCE_MARGIN	= 0x1B;		// Counter-clockwise compliance margin register
const uint8_t AX12A_REG_CW_COMPLIANCE_SLOPE	    = 0x1C;		// Clockwise compliance slope register
const uint8_t AX12A_REG_CCW_COMPLIANCE_SLOPE    = 0x1D;		// Counter-clockwise compliance slope register
const uint8_t AX12A_REG_GOAL_POSITION		    = 0x1E;		// Goal position register (0x1E = low byte, 0x1F = high byte)
const uint8_t AX12A_REG_GOAL_VELOCITY		    = 0x20;		// Goal velocity register (0x20 = low byte, 0x21 = high byte)
const uint8_t AX12A_REG_GOAL_TORQUE			    = 0x22;		// Goal torque register (0x22 = low byte, 0x23 = high byte)
const uint8_t AX12A_REG_LOCK_EEPROM 	 	    = 0x2F;		// EEPROM lock register
const uint8_t AX12A_REG_PUNCH 	 			    = 0x30;		// Punch (0x30 = low register, 0x31 = high register)
const uint8_t AX12A_REG_CURRENT_POSITION 	    = 0x24;		// Current position register (0x24 = low byte, 0x25 = high byte)
const uint8_t AX12A_REG_CURRENT_VELOCITY 	    = 0x26;		// Current velocity register (0x26 = low byte, 0x27 = high byte)
const uint8_t AX12A_REG_CURRENT_LOAD 		    = 0x28;		// Current load register (0x28 = low byte, 0x29 = high byte)
const uint8_t AX12A_REG_CURRENT_VOLTAGE 	    = 0x2A;		// Current voltage register
const uint8_t AX12A_REG_CURRENT_TEMPERATURE     = 0x2B;		// Current temperature register
const uint8_t AX12A_REG_REGISTERED 			    = 0x2C;		// Command execution status register
const uint8_t AX12A_REG_MOVING 				    = 0x2E;		// Motor motion register

/* Default register value definitions. */
const uint16_t DEFAULT_CW_ANGLE_LIMIT		= 0x0000;	// Default clockwise angle limit
const uint16_t DEFAULT_CCW_ANGLE_LIMIT		= 0x03FF;	// Default counter-clockwise angle limit
const uint8_t DEFAULT_HIGH_VOLTAGE_LIMIT	= 0xBE;	    // Default permitted maximum voltage (0xBE = 140 -> 14.0 V)
const uint8_t DEFAULT_LOW_VOLTAGE_LIMIT		= 0x3C;	    // Default permitted minimum voltage (0x3C = 60 -> 6.0 V)
const uint16_t DEFAULT_MAXIMUM_TORQUE		= 0x03FF;	// Default maximum torque limit (10-bit resolution percentage)
const uint8_t DEFAULT_ALARM_LED				= 0x24;	    // Default condition(s) under which the alarm LED will be set
const uint8_t DEFAULT_ALARM_SHUTDOWN		= 0x24;	    // Default condition(s) under which the motor will shut down due to an alarm
const uint8_t DEFAULT_CW_COMPLIANCE_MARGIN	= 0x01;	    // Default clockwise compliance margin (position error)
const uint8_t DEFAULT_CCW_COMPLIANCE_MARGIN	= 0x01;	    // Default counter-clockwise compliance margin (position error)
const uint8_t DEFAULT_CW_COMPLIANCE_SLOPE	= 0x20;	    // Default clockwise compliance slope (torque near goal position)
const uint8_t DEFAULT_CCW_COMPLIANCE_SLOPE	= 0x20;	    // Default counter-clockwise compliance slope (torque near goal position)
const uint8_t DEFAULT_EEPROM_LOCK			= 0x00;	    // Default value for the EEPROM lock
const uint16_t DEFAULT_PUNCH				= 0x0020;	// Default punch




/********************************* Functions *********************************/
// Constructor and destructor
AX12A::AX12A(MotorInitData* motorInitData) :
	Dynamixel(motorInitData)
{
	// TODO Auto-generated constructor stub
	this -> angleResolution = 1023; // 2 ^ 10

	this -> REG_ID = AX12A_REG_ID;
	this -> REG_BAUD_RATE = AX12A_REG_BAUD_RATE;
	this -> REG_GOAL_POSITION = AX12A_REG_GOAL_POSITION;
	this -> REG_GOAL_VELOCITY = AX12A_REG_GOAL_VELOCITY;
	this -> REG_TORQUE_ENABLE = AX12A_REG_TORQUE_ENABLE;
	this -> REG_RETURN_DELAY_TIME = AX12A_REG_RETURN_DELAY_TIME;
	this -> REG_LED_ENABLE = AX12A_REG_LED_ENABLE;
	this -> REG_CURRENT_POSITION = 	AX12A_REG_CURRENT_POSITION;
	this -> REG_CURRENT_VELOCITY = AX12A_REG_CURRENT_VELOCITY;
	this -> REG_CURRENT_LOAD = AX12A_REG_CURRENT_LOAD;
}

AX12A::~AX12A() {
	// TODO Auto-generated destructor stub
}

int AX12A::Init(){
	/* Configure motor to return status packets only for read commands. */
	AX12A::setStatusReturnLevel(1);
	HAL_Delay(10);

	/* Set minimum delay return time (2 microseconds). */
	AX12A::setReturnDelayTime(2);
	HAL_Delay(10);

	/* Impress current to motor to enable torque. */
	AX12A::setTorqueEnable(1);
	HAL_Delay(10);

	return 1;
}




// Low-level transmission and reception
void AX12A::dataWriter(uint8_t arrSize, \
						   uint8_t writeAddr, uint8_t param1, uint8_t param2){
	/* Handles sending of data since this nearly identical for all setters.
	 * Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
	 *
	 * Arguments: arrSize, the size of the array to be transmitted. Must be 8 or 9
	 * 			  writeAddr, the address for the parameters to be written to (may be motor EEPROM or RAM)
	 * 			  param1, the first parameter
	 * 			  param2, the second parameter. If arrSize == 8, then param2 is ignored and should be passed as -1
	 *
	 * Returns: none
	 */

	/* Do assignments and computations. */
	uint8_t ID = this -> id;
	uint8_t arrTransmit[9];
	arrTransmit[0] = 0xFF;
	arrTransmit[1] = 0xFF;
	arrTransmit[2] = ID;
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_WRITE_DATA;
	arrTransmit[5] = writeAddr; // Write address for register
	arrTransmit[6] = param1;

	/* Checksum. */
	arrTransmit[7] = (arrSize == 8) ? AX12A::computeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = AX12A::computeChecksum(arrTransmit, arrSize);
	}

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	/* In the future, a lock should be placed here, and it should be unlocked after the status
	 * packet is received. This requires using interrupts for TX/RX and it cannot be implemented
	 * right now because the blocking transmit and receive use timer interrupts for their timeout
	 */

	/* Transmit. */
	HAL_UART_Transmit(this -> uartHandle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);
}

uint16_t AX12A::dataReader(uint8_t readAddr, uint8_t readLength){
	/* Reads data back from the motor passed in by the handle. This process is identical
	 * for all the getters which is why it is encapsulated in this function. Reading data
	 * uses the READ DATA instruction, 0x02, in the motor instruction set.
	 *
	 * The status packet returned will be of them form:
	 * 0xFF 0xFF ID LENGTH ERR PARAM_1...PARAM_N CHECKSUM, where N = readLength
	 *
	 * Arguments: readAddr, the address inside the motor memory table where reading is to begin
	 *			  readLength, the number of bytes to be read. Must be either 1 or 2.
	 *
	 * Returns: a 16-bit value containing 1 or both bytes received, as applicable. The
	 * 			1st byte received will be the LSB and the 2nd byte received will be the MSB
	 */

	/* Do assignments and computations. */
	uint8_t arr[8]; // Define array to be used for transmission and reception
	arr[0] = 0xFF;
	arr[1] = 0xFF;
	arr[2] = this -> id;
	arr[3] = 4; // Length of message minus the obligatory bytes
	arr[4] = INST_READ_DATA;
	arr[5] = readAddr; // Register from which to read
	arr[6] = readLength; // Number of bytes to be read from motor
	arr[7] = AX12A::computeChecksum(arr, 8); // Attach checksum to packet

	// Set data direction for transmit
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	// Transmit
	HAL_UART_Transmit(this -> uartHandle, arr, 8, TRANSMIT_TIMEOUT);

	// Set data direction for receive
	__DYNAMIXEL_RECEIVE(this -> dataDirPort, this -> dataDirPinNum);

	// Call appropriate UART receive function depending on if 1 or 2 bytes are to be read
	if(readLength == 1){
		HAL_UART_Receive(this -> uartHandle, arr, 7, RECEIVE_TIMEOUT);
	}
	else{
		HAL_UART_Receive(this -> uartHandle, arr, 8, RECEIVE_TIMEOUT);
	}

	// Check the status packet received for errors
	if(arr[4] != 0){
		// This is an error condition. In the future, an error handler should be implemented
	}

	if(readLength == 1){
		return (uint16_t)arr[5];
	}
	else{
		return (uint16_t)(arr[5] | (arr[6] << 8));
	}
}

uint8_t AX12A::computeChecksum(uint8_t *arr, int length){
	/* Compute the checksum for data to be transmitted.
	 *
	 * Arguments: arr, the array to be transmitted and ran through the checksum function
	 * 			  length, the total length of the array arr
	 *
	 * Returns: the 1-byte number that is the checksum
	 */

	/* Local variable declaration. */
	uint8_t accumulate = 0;

	/* Loop through the array starting from the 2nd element of the array and finishing before the last
	 * since the last is where the checksum will be stored. */
	for(uint8_t i = 2; i < length - 1; i++){
		accumulate += arr[i];
	}

	return (~accumulate) & 0xFF; // Lower 8 bits of the logical NOT of the sum
}




// Setters (use the WRITE DATA instruction)
// TODO: test setBaudRate
void AX12A::setBaudRate(double baud){
	/* Sets the baud rate of a particular motor. Register address is 0x04 in motor EEPROM.
	 *
	 * Default value: 0x01
	 *
	 * Arguments: baud, the baud rate. Arguments in range [7844, 1000000] are valid
	 *
	 * Returns: none
	 */

	/* Set _baud equal to the hex code corresponding to baud. Default to 1 Mbps. */
	uint8_t baudArg;

	if(baud > 0){
		/* Valid for baud in range [7844, 1000000]. Will be converted to 8-bit resolution. */
		baudArg = (uint8_t)((2000000 / baud) - 1);
		this -> baudRate = baud;
	}
	else{
		/* Default to 1 Mbps. */
		this -> baudRate = 1000000;
		baudArg = DEFAULT_BAUD_RATE;
	}

	/* Write data to motor. */
	AX12A::dataWriter(8, this -> REG_BAUD_RATE, baudArg, 0);
}

void AX12A::setCWComplianceSlope(uint8_t CWcomplianceSlope){
	/* Sets the clockwise compliance slope for the current motor, which sets the level of torque near the goal position.
	 * The higher the value, the more flexibility that is obtained. That is, a high value
	 * means that as the goal position is approached, the torque will be significantly reduced. If a low
	 * value is used, then as the goal position is approached, the torque will not be reduced all that much.
	 *
	 * Instruction register address: 0x1C (RAM)
	 * Default value: 0x20
	 *
	 * Arguments: CWcomplianceSlope, arguments in range [1, 7], with 1 being the least flexible
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
		step = DEFAULT_CW_COMPLIANCE_SLOPE;
	}

	/* Write data to motor. */
	AX12A::dataWriter(8, AX12A_REG_CW_COMPLIANCE_SLOPE, step, 0);
}

void AX12A::setCCWComplianceSlope(uint8_t CCWcomplianceSlope){
	/* Sets the counter-clockwise compliance slope for the current motor, which sets the level of torque near the goal position.
	 * The higher the value, the more flexibility that is obtained. That is, a high value
	 * means that as the goal position is approached, the torque will be significantly reduced. If a low
	 * value is used, then as the goal position is approached, the torque will not be reduced all that much.
	 *
	 * Instruction register address: 0x1D (RAM)
	 * Default value: 0x20
	 *
	 * Arguments: CCWcomplianceSlope, arguments in range [1, 7], with 1 being the least flexible
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
		step = DEFAULT_CCW_COMPLIANCE_SLOPE;
	}

	/* Write data to motor. */
	AX12A::dataWriter(8, AX12A_REG_CCW_COMPLIANCE_SLOPE, step, 0);
}


void AX12A::setStatusReturnLevel(uint8_t status_data){
	/* Sets the conditions under which a status packet will be returned.
	 *
	 * Register address: 0x10 (EEPROM)
	 * Default value: 0x02
	 *
	 * Arguments: status, 0 to return only on ping
	 * 			  		  1 to return only for reads
	 * 			  		  2 to return for all commands
	 *
	 * Returns: none
	 */

	/* Evaluate argument validity. */
	if((status_data < 0) || (status_data > 2)){
		status_data = DEFAULT_STATUS_RETURN_LEVEL;
	}

	/* Write data to motor. */
	AX12A::dataWriter(8, AX12A_REG_STATUS_RETURN_LEVEL, status_data, -1);
}




// Interfaces for previously-defined functions
void AX12A::enterWheelMode(double goalVelocity){
	/* Sets the control registers such that the rotational angle of the motor
	 * is not bounded.
	 *
	 * Arguments: goalVelocity, the desired velocity to use when entering wheel mode
	 *
	 * Returns: none
	 */

	/* When the angle limits are both set to 0, then motor will attempt to
	 * rotate with maximum velocity. To prevent undesired behaviour, the
	 * goal velocity should be set right after calling this function */
	Dynamixel::setMinAngle(0);
	Dynamixel::setMaxAngle(0);
	this -> isJointMode = 0;
	AX12A::setGoalVelocity(goalVelocity);
}

// TODO: Re-test moving motor to nearest valid position
void AX12A::enterJointMode(){
	/* Sets the control registers such that the rotational angle of the motor
	 * is constrained between the default values.
	 *
	 * Arguments: none
	 *
	 * Returns: none
	 */

	// In the future, it would be good to make this robust by returning the motor
	// to the nearest valid position. Otherwise, motor gets confused because it's
	// locked out of its valid boundaries and there you can't instruct it to go anywhere
//	AX12A::getPosition();
//	while((this -> lastPosition < MIN_ANGLE) || (this -> lastPosition > MAX_ANGLE)){
//		if(360 - this -> lastPosition < 15){
//			AX12A::setGoalVelocity(MAX_VELOCITY);
//		}
//		else{
//			AX12A::setGoalVelocity(-MAX_VELOCITY);
//		}
//		AX12A::getPosition();
//	}

	this -> isJointMode = 1;
	Dynamixel::setMinAngle(MIN_ANGLE);
	Dynamixel::setMaxAngle(MAX_ANGLE);
}

void AX12A::setComplianceSlope(uint8_t complianceSlope){
	/* Sets both the clockwise and counterclockwise compliance slopes.
	 * The higher the value, the more flexibility that is obtained. That is, a high value
	 * means that as the goal position is approached, the torque will be significantly reduced. If a low
	 * value is used, then as the goal position is approached, the torque will not be reduced all that much.
	 *
	 *
	 * Arguments: complianceSlope, arguments in range [1, 7], with 1 being the least flexible
	 *
	 */

	AX12A::setCWComplianceSlope(complianceSlope);
	AX12A::setCWComplianceSlope(complianceSlope);
}

void AX12A::setComplianceMargin(uint8_t complianceMargin){
	/* Sets both the clockwise and counterclockwise compliance margins.
	 * The compliance margin is the acceptable error between the current position and goal position.
	 * The greater the value, the more error is acceptable.
	 *
	 * Arguments: complianceMargin, the acceptable error between the current and goal position.
	 * 			  					Arguments in range [0, 255]
	 */

	AX12A::setCWComplianceMargin(complianceMargin);
	AX12A::setCCWComplianceMargin(complianceMargin);
}

