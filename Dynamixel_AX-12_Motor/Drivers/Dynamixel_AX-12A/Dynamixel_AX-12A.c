/********************************* Includes ************************************/
#include "Dynamixel_AX-12A.h"
#include "stm32f4xx_hal_conf.h"

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
/********************************************((*********************************/
// NEEDS TESTING POST-REFACTOR
void Dynamixel_SetID(Dynamixel_HandleTypeDef *hdynamixel, int ID){
	/* Sets the ID (identification number) for the current motor.
	 *
	 * Instruction register address: 0x03 (EEPROM)
	 * Default value: 140
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  ID, the number between 0 and 252 to identify the motor. If 0xFE (254), then
	 * 			  	  any messages broadcasted to that ID will be broadcasted to all motors
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x03, ID, -1);
}

// NEEDS TESTING POST-REFACTOR
void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef *hdynamixel, double baud){
	/* Sets the baud rate of a particular motor. Register address is 0x04 in motor EEPROM.
	 *
	 * Instruction register address: 0x04 (EEPROM)
	 * Default value: 0x01
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  baud, the baud rate. Arguments in range [7844, 1000000] are valid
	 *
	 * Returns: none
	 */

	// Set _baud equal to the hex code corresponding to baud. Default to 1 Mbps
	if(baud > 0){
		// Valid for baud in range [7844, 1000000]. Will be converted to 8-bit resolution
		hdynamixel -> _BaudRate = (uint8_t)((2000000 / baud) - 1);
	}
	else{
		// Default to 1 Mbps
		hdynamixel -> _BaudRate = 0x01;
	}

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x04, hdynamixel -> _BaudRate, -1);
}

// NEEDS TESTING
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef *hdynamixel, double microSec){
	/* Sets the time, in microseconds, that the motor should wait before returning a status packet.
	 *
	 * Instruction register address: 0x05(EEPROM)
	 * Default value: 250 (0xFA)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  microSec, the time in microseconds to delay. Arguments in range [2, 508] are valid. Default: 500
	 *
	 * Returns: none
	 */

	// Compute validity
	uint8_t motor_data;
	if((microSec < 1) || (microSec > 509)){
		motor_data = 250;
	}
	else{
		motor_data = (uint8_t)(microSec / 2);
	}

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x05, motor_data, -1);
}

// NEEDS TESTING POST-REFACTOR
void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef *hdynamixel, double minAngle){
	/* Sets the clockwise angle limit for the current motor.
	 * Register 0x06 in EEPROM for low byte, 0x07 in EEPROM for high byte
	 *
	 * Instruction register address: 0x06 (EEPROM)
	 * Default value: 0x0000
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  minAngle, the minimum angle for all motor operations. Arguments between 0 and 300 are valid
	 *
	 * If maxAngle for CCW angle limit is 0 AND minAngle for CW angle limit is 0,
	 * then motor is in wheel mode where it can continuously rotate. Otherwise, motor is in joint mode where its
	 * motion is constrained between the set bounds.
	 *
	 * Returns: none
	 */

	// Translate the angles from degrees into 8-bit numbers
	int normalized_value = (int)(minAngle / 300 * 1023);

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of CW angle limit
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of CW angle limit

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x06, lowByte, highByte);
}

// NEEDS TESTING POST-REFACTOR
void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef *hdynamixel, double maxAngle){
	/* Sets the counter-clockwise angle limit for the current motor.
	 * Register 0x08 in EEPROM for low byte, 0x09 in EEPROM for high byte
	 *
	 * Instruction register address: 0x08 (EEPROM)
	 * Default value: 0x03FF
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  maxAngle, the maximum angle for all motor operations. Arguments between 0 and 300 are valid.
	 *
	 * If maxAngle for CCW angle limit is 0 AND minAngle for CW angle limit is 0,
	 * then motor is in wheel mode where it can continuously rotate. Otherwise, motor is in joint mode where its
	 * motion is constrained between the set bounds.
	 *
	 * Returns: none
	 */

	// Translate the angles from degrees into 8-bit numbers
	int normalized_value = (int)(maxAngle / 300 * 1023);

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of CCW angle limit
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of CCW angle limit

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x08, lowByte, highByte);
}

// NEEDS TESTING
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef *hdynamixel, double highestVoltage){
	/* Sets the highest operating voltage limit for the current motor.
	 *
	 * Instruction register address: 0x0C (EEPROM)
	 * Default value: 140 (0xBE)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  highestVoltage, the highest operating voltage in volts
	 *
	 * Returns: none
	 */

	// Declare local variable. Initialize to default value
	uint8_t high_voltage_data = 0xBE;

	// Evaluate argument validity and translate into motor data
	if(highestVoltage > 6 && highestVoltage < 14){
		high_voltage_data = (uint8_t)(highestVoltage * 10);
	}

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x0C, high_voltage_data, -1);
}

// NEEDS TESTING
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef *hdynamixel, double lowestVoltage){
	/* Sets the lowest operating voltage limit for the current motor.
	 *
	 * Instruction register address: 0x0D (EEPROM)
	 * Default value: 60 (0x3C)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  lowestVoltage, the lowest operating voltage in volts
	 *
	 * Returns: none
	 */

	// Declare local variable. Initialize to default value
	uint8_t low_voltage_data = 0x3C;

	// Evaluate argument validity and translate into motor data
	if(lowestVoltage > 6 && lowestVoltage < 14){
		low_voltage_data = (uint8_t)(lowestVoltage * 10);
	}

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x0D, low_voltage_data, -1);
}

// NEEDS TESTING
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef *hdynamixel, double maxTorque){
	/* Sets the maximum torque limit for all motor operations.
	 * Low byte is addr 0x0E in motor RAM, high byte is addr 0x0F in motor RAM.
	 *
	 * Register address: 0x0E (EEPROM)
	 * Default value: 0x3FF
	 *
	 * Arguments: hdynamixel, the motor handle
	 *			  maxTorque, the maximum torque as a percentage (max: 100). Gets converted to 10-bit number
	 *
	 * Returns: none
	 */

	// Translate the input from percentage into and 10-bit number
	int normalized_value = (int)(maxTorque / 100 * 1023);

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of max torque
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of max torque

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x0E, lowByte, highByte);
}

// NEEDS TESTING
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef *hdynamixel, uint8_t status_data){
	/* Sets the conditions under which a status packet will be returned.
	 *
	 * Register address: 0x10 (EEPROM)
	 * Default value: 0x02
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  status, 0 to return only on ping
	 * 			  		  1 to return only for reads
	 * 			  		  2 to return for all commands
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x10, status_data, -1);
}

// NEEDS TESTING
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef *hdynamixel, uint8_t alarm_LED_data){
	/* Sets the conditions under which the motor LED will light up.
	 *
	 * Register address: 0x11 (EEPROM)
	 * Default value: 0x24 (0x00100100 -> b7b6b5b4b3b2b1b0)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  alarm_data, bit 7: no function
	 * 			  			  bit 6: flash LED when an instruction error occurs
	 * 			  			  bit 5: flash LED when current load cannot be controlled with the specified maximum torque
	 * 			  			  bit 4: flash LED when the checksum of the transmitted packet is invalid
	 * 			  			  bit 3: flash LED when the command is given beyond the range of usage
	 * 			  			  bit 2: flash LED when the internal temperature exceeds the operating range
	 * 			  			  bit 1: flash LED when goal position exceeds the CW angle limit or CCW angle limit
	 * 			  			  bit 0: flash LED when applied voltage is out of oeprating
	 *
	 * Several of these bits may be set simultaneously.
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x11, alarm_LED_data, -1);
}

// NEEDS TESTING
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef *hdynamixel, uint8_t alarm_shutdown_data){
	/* Sets the conditions under which the motor will turn off its torque.
	 *
	 * Register address: 0x12 (EEPROM)
	 * Default value: 0x24
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  alarm_data, bit 7: no function
	 * 			  			  bit 6: torque off when an instruction error occurs
	 * 			  			  bit 5: torque off when current load cannot be controlled with the specified maximum torque
	 * 			  			  bit 4: torque off when the checksum of the transmitted packet is invalid
	 * 			  			  bit 3: torque off when the command is given beyond the range of usage
	 * 			  			  bit 2: torque off when the internal temperature exceeds the operating range
	 * 			  			  bit 1: torque off when goal position exceeds the CW angle limit or CCW angle limit
	 * 			  			  bit 0: torque off when applied voltage is out of oeprating
	 *
	 * Several of these bits may be set simultaneously.
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x12, alarm_shutdown_data, -1);
}

// NEEDS TESTING
void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef *hdynamixel, uint8_t isEnabled){
	/* Enables or disables torque for current motor.
	 *
	 * Instruction register address: 0x18 (RAM)
	 * Default value: 0x00
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  isEnabled, if 1, then generates torque by impressing power to the motor
	 * 			  			 if 0, then interrupts power to the motor to prevent it from generating torque
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x18, isEnabled, -1);
}

// NEEDS TESTING
void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef *hdynamixel, uint8_t isEnabled){
	/* Toggles the motor LED.
	 *
	 * Register address: 0x19
	 * Default value: 0x00
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  isEnabled, 0 if LED off
	 * 			  			 1 if LED on
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x19, isEnabled, -1);
}

// NEEDS TESTING
void Dynamixel_SetCWComplianceMargin(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CWcomplianceMargin){
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

	// Translate the angle from degrees into an 8-bit number
	uint8_t normalized_value = (uint8_t)(CWcomplianceMargin / 300 * 255);

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x1A, normalized_value, -1);
}

// NEEDS TESTING
void Dynamixel_SetCCWComplianceMargin(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CCWcomplianceMargin){
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

	// Translate the angle from degrees into an 8-bit number
	uint8_t normalized_value = (uint8_t)(CCWcomplianceMargin / 300 * 255);

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x1B, normalized_value, -1);
}

// NEEDS TESTING
void Dynamixel_SetCWComplianceSlope(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CWcomplianceSlope){
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

	// Translate the step into motor data
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
		step = 32; // Default value
	}

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x1C, step, -1);
}

// NEEDS TESTING
void Dynamixel_SetCCWComplianceSlope(Dynamixel_HandleTypeDef *hdynamixel, uint8_t CCWcomplianceSlope){
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

	// Translate the step into motor data
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
		step = 32; // Default value
	}

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x1D, step, -1);
}

// NEEDS TESTING POST-REFACTOR
void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef *hdynamixel, double goalAngle){
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

	// Translate the angle from degrees into a 10-bit number
	int normalized_value = (int)(goalAngle / 300 * 1023); // maximum angle of 1023

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of goal position
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of goal position

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x1e, lowByte, highByte);
}

// NEEDS TESTING POST-REFACTOR
void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef *hdynamixel, double goalVelocity){
	/* Sets the goal velocity of the motor in RAM.
	 * Low byte is 0x20 in motor RAM, high byte is 0x21 in motor RAM.
	 *
	 * Instruction register address: 0x20 (RAM)
	 * Default value: none
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  velocity, the goal velocity in RPM. Arguments of 0-114 are valid. If 0, max is used.
	 *
	 * Returns: none
	 */

	// Translate the position from RPM into a 10-bit number
	int normalized_value = 0;
	if(hdynamixel -> _isJointMode){
		normalized_value = (int)(goalVelocity / 114 * 1023);
	}
	else{
		normalized_value = (int)(goalVelocity / 114 * 2047);
	}

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of goal velocity
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of goal velocity

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x20, lowByte, highByte);
}

// NEEDS TESTING
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef *hdynamixel, double goalTorque){
	/* Sets the goal torque for the current motor. Initial value is taken from 0x0E and 0x0F (max torque in EEPROM)
	 * Low byte is 0x22 in motor RAM, high byte is 0x23 in motor RAM.
	 *
	 * Instruction register address: 0x22 (RAM)
	 * Default value: ADDR14 (low byte) and ADDR15 (high byte)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  goalTorque, the percentage of the maximum possible torque (max: 100). Gets converted into 10-bit number
	 *
	 * Returns: none
	 */

	// Translate the input from percentage into and 10-bit number
	int normalized_value = (int)(goalTorque / 100 * 1023);

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of goal torque
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of goal torque

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x22, lowByte, highByte);
}

// NEEDS TESTING
void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef *hdynamixel, uint8_t isLocked){
	/* Locks the EEPROM of the current motor until the next power cycle.
	 *
	 * Instruction register address: 0x2F (RAM)
	 * Default value: 0x00
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  isLocked, 0 if EEPROM is to be read/write
	 * 			  			1 if EEPROM is to be made read-only
	 *
	 * Returns: none
	 */

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 8, 0x2F, isLocked, -1);
}

// NEEDS TESTING
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef *hdynamixel, double punch){
	/* Sets a quantity proportional to the minimum current supplied to the motor during operation.
	 * Units are not specified in datasheet, and therefore this function is not entirely useful without
	 * sufficient testing.
	 * Low byte at address 0x30 and high byte at address 0x31
	 *
	 * Instruction register address: 0x30
	 * Default value: 0x0020 (maximum: 0x3FF)
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  punch, for now, arguments in range [0, 1023] are valid
	 *
	 * Returns: none
	 */

	// Translate the punch into a 10-bit number
	int normalized_value = punch;

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of punch
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of punch

	// Write data to motor
	Dynamixel_DataWriter(hdynamixel, 9, 0x30, lowByte, highByte);
}




/*******************************************************************************/
/*	Getter helper functions											  		   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/********************************************((*********************************/
// UNIMPLEMENTED
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads addresses 0x24 and 0x25 in the motors RAM to see what the current position
	 * of the motor is.
	 * Writes the results to hdynamixel -> _lastPosition
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */
}

// UNIMPLEMENTED
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads addresses 0x26 and 0x27 in the motor RAM to see what the current velocity
	 * of the motor is.
	 * Writes the results to hdynamixel -> _lastVelocity
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */
}

// UNIMPLEMENTED
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads addresses 0x28 and 0x29 in the motor RAM to see what the current
	 * load is.
	 * Writes the results to hdynamixel -> _lastLoad
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */
}

// UNIMPLEMENTED
void Dynamixel_GetVoltage(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads address 0x2A in the motor RAM to see what the current voltage is.
	 * Value retrieved from motor is 10 times the actual voltage.
	 * Writes the results to hdynamixel -> _lastVoltage
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */
}

// UNIMPLEMENTED
void Dynamixel_GetTemperature(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads address 0x2B in the motor RAM to see what the current temperature is inside the motor.
	 * Results in degrees Celsius.
	 * Writes the results to hdynamixel -> _lastTemperature
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */
}

// UNIMPLEMENTED
uint8_t Dynamixel_GetRegistered(Dynamixel_HandleTypeDef *hdynamixel){
	/* Used to tell if command sent was written to motor registers.
	 * Can also be used to see if instruction in motor register has been executed.
	 *
	 * Read address: 0x2C
	 * Default value: 0x00
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: 1 if there are commands transmitted by REG_WRITE
	 * 			0 otherwise.
	 * 			If ACTION command is executed, the value is changed into 0.
	 */

	return -1;
}

// UNIMPLEMENTED
uint8_t Dynamixel_IsMoving(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads the 0x2E address in motor RAM to see if motor is moving.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: 0 if not moving
	 * 			1 if moving
	 */

	return -1;
}

// UNIMPLEMENTED
uint8_t Dynamixel_IsJointMode(Dynamixel_HandleTypeDef *hdynamixel){
	/* Reads the CW (addr: 0x06) and CCW (addr: 0x07) angle limits. If both are 0, motor is in wheel mode
	 * and can spin indefinitely. Otherwise, motor is in joint mode and has angle limits set
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: 0 if in wheel mode
	 * 			1 if in joint mode
	 */

	return -1;
}




/*******************************************************************************/
/*	Other motor instruction help functions									   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/********************************************((*********************************/
// NEEDS TESTING
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef *hdynamixel, uint8_t arrSize, \
		uint8_t writeAddr, uint8_t param1, uint8_t param2){
	/* Implementation of REG WRITE instruction with 2 parameters.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  writeAddr, thhe starting address for where the data is to be written
	 * 			  param1, the first parameter
	 * 			  param2, the second parameter
	 *
	 * Returns: none
	 */

	// Define arrays for transmission
	uint8_t arrTransmit[arrSize];

	// Do assignments and computations
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = 0x04; // REG WRITE instruction
	arrTransmit[5] = writeAddr;
	arrTransmit[7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = Dynamixel_ComputeChecksum(arrTransmit, arrSize);
	}

	// Set data direction
	__DYNAMIXEL_TRANSMIT();

	// Transmit
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);
}

// NEEDS TESTING
void Dynamixel_Action(Dynamixel_HandleTypeDef *hdynamixel){
	/* Implements the ACTION instruction. This triggers the instruction registered by the REG WRITE
	 * instruction. This way, time delays can be reduced for the concurrent motion of several
	 * motors.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	// Define arrays for transmission and reception
	uint8_t arrTransmit[6];

	// Do assignments and computations
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = 0x05; // ACTION instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	// Set data direction
	__DYNAMIXEL_TRANSMIT();

	// Transmit
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
}




/*******************************************************************************/
/*	Computation helper functions											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/********************************************((*********************************/
uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length){
	/* Compute the checksum for data to be transmitted.
	 *
	 * Arguments: arr, the array to be transmitted and ran through the checksum function
	 * 			  length, the total length of the array arr
	 *
	 * Returns: the 1-byte number that is the checksum
	 */

	// Local variable declaration
	uint8_t accumulate = 0;

	// Loop through the array starting from the 2nd element of the array and finishing before the last
	// since the last is where the checksum will be stored
	for(uint8_t i = 2; i < length - 1; i++){
		accumulate += arr[i];
	}

	return 255 - (accumulate % 256); // Lower 8 bits of the logical NOT of the sum
}




/*******************************************************************************/
/*	Error handling helper functions											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/********************************************((*********************************/
// UNIMPLEMENTED
void Dynamixel_ErrorHandler(uint8_t errCode){
	/* Handles errors raised in error code bytes of status packets.
	 *
	 * Arguments: errCode, the error code returned by the status packet
	 *
	 * Returns: none
	 */

	// TODO: write function
	return;
}




/*******************************************************************************/
/*	Transmit/receive helper functions										   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/********************************************((*********************************/
// NEEDS TESTING
void Dynamixel_Ping(Dynamixel_HandleTypeDef *hdynamixel){
	/* Used only for returning a status packet or checking the existence of a motor
	 * with a specified ID. Does not command any operations.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	// Define arrays for transmission and reception
	uint8_t arrTransmit[6];

	// Do assignments and computations
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = 0x01; // PING instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	// Set data direction
	__DYNAMIXEL_TRANSMIT();

	// Transmit
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
}

// NEEDS TESTING
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef *hdynamixel, uint8_t arrSize, \
						   uint8_t writeAddr, uint8_t param1, uint8_t param2){
	/* Handles sending of data since this nearly identical for all setters.
	 * Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  arrSize, the size of the array to be transmitted. Must be 8 or 9
	 * 			  writeAddr, the address for the parameters to be written to (may be motor EEPROM or RAM)
	 * 			  param1, the first parameter
	 * 			  param2, the second parameter. If arrSize == 8, then param2 is ignored and should be passed as -1
	 *
	 * Returns: none
	 */

	// Check that array size is 8 or 9
	if(arrSize != 8 && arrSize != 9){
		_Error_Handler(__FILE__, __LINE__);
		return;
	}

	// Define array for transmission
	uint8_t arrTransmit[arrSize];

	// Do assignments and computations
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = 0x03; // WRITE DATA instruction
	arrTransmit[5] = writeAddr; // Write address for register
	arrTransmit[6] = param1;

	// Checksum
	arrTransmit[7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = Dynamixel_ComputeChecksum(arrTransmit, arrSize);
	}

	// Set data direction
	__DYNAMIXEL_TRANSMIT();

	// Transmit
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);

	// In the future, it would be good to read a status packet back after the transmit and ensure
	// that there were no errors. If errors occurred, the message could be resent
}

// UNIMPLEMENTED
void Dynamixel_SyncWriter(uint8_t arrSize, uint8_t *params){
	/* Used for sending control signals to several specified Dynamixel actuators concurrently.
	 * Uses the SYNC WRITE instruction, 0x83, in the motor instruction set.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  arrSize, the size of the array of parameters to be transmitted
	 * 			  params, the array holding all the instructions and parameters to be passed to the various actuators
	 *
	 * Returns: none
	 */
}

// NEEDS TESTING
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef *hdynamixel, uint8_t readAddr, uint8_t readLength){
	/* Reads data back from the motor passed in by the handle. This process is identical
	 * for all the getters which is why it is encapsulated in this function. Reading data
	 * uses the READ DATA instruction, 0x02, in the motor instruction set.
	 *
	 * The status packet returned will be of them form:
	 * 0xFF 0xFF ID LENGTH ERR PARAM_1...PARAM_N CHECKSUM, where N = readLength
	 *
	 * Arguments: hdynamixel, the motor handle
	 *			  readAddr, the address inside the motor memory table where reading is to begin
	 *			  readLength, the number of bytes to be read. Must be either 1 or 2.
	 *
	 * Returns: a 16-bit value containing 1 or both bytes received, as applicable. The
	 * 			1st byte received will be the LSB and the 2nd byte received will be the MSB
	 */

	// Define arrays for transmission
	uint8_t arrTransmit[8];

	// Clear array for reception
	for(uint8_t i = 0; i < BUFF_RX_SIZE; i++){
		arrReceive[i] = 0;
	}

	// Do assignments and computations
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = 0x02; // READ DATA instruction
	arrTransmit[5] = readAddr; // Write address for register
	arrTransmit[6] = readLength; // Number of bytes to be read from motor
	arrTransmit[7] = Dynamixel_ComputeChecksum(arrTransmit, 8);

	// Ensure that received data has valid checksum. If it does not, send data request again
	uint8_t valid = 0;
	while(!valid){
		// Set data direction for transmit
		__DYNAMIXEL_TRANSMIT();

		// Transmit
		HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 8, TRANSMIT_TIMEOUT);

		// Set data direction for receive
		__DYNAMIXEL_RECEIVE();

		// Call appropriate UART receive function depending on if 1 or 2 bytes are to be read
		if(readLength == 1){
			HAL_UART_Receive(hdynamixel -> _UART_Handle, arrReceive, 8, RECEIVE_TIMEOUT);
			valid = (Dynamixel_ComputeChecksum(arrReceive, 8) == arrReceive[7]); // Verify checksums match
		}
		else{
			HAL_UART_Receive(hdynamixel -> _UART_Handle, arrReceive, 8, RECEIVE_TIMEOUT);
			valid = (Dynamixel_ComputeChecksum(arrReceive, 8) == arrReceive[8]); // Verify checksums match
		}
	}

	// Check the status packet received for errors
	if(arrReceive[5] != 0){
		Dynamixel_ErrorHandler(arrReceive[5]);
	}

	if(readLength == 1){
		return (uint16_t)arrReceive[5];
	}
	else{
		return (uint16_t)(arrReceive[5] & (arrReceive[6] << 8));
	}
}




/*******************************************************************************/
/*	Initialization helper functions											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/********************************************((*********************************/
void Dynamixel_Init(Dynamixel_HandleTypeDef *hdynamixel, uint8_t ID, UART_HandleTypeDef *UART_Handle)
{
	hdynamixel -> _ID = ID; // Motor ID (unique or global)
	hdynamixel -> _BaudRate = 1000000; // In future, can initialize this accurately by reading from EEPROM. Or, take a baud rate argument and at bottom of this function, set the EEPROM
	hdynamixel -> _lastPosition = 0; // In future, initialize this accurately
	hdynamixel -> _lastVelocity = 0; // In future, initialize this accurately
	hdynamixel -> _lastLoad = 0; // In future, initialize this accurately
	hdynamixel -> _lastVoltage = 0; // In future, initialize this accurately
	hdynamixel -> _lastTemperature = 0; // In future, initialize this accurately
	hdynamixel -> _isJointMode = 1; // In future, initialize this accurately
	hdynamixel -> _UART_Handle = UART_Handle; // For UART TX and RX
}

// NEEDS TESTING
void Dynamixel_Reset(Dynamixel_HandleTypeDef *hdynamixel){
	/* Resets the control table values of the motor to the Factory Default Value settings.
	 * Note that post-reset, motor ID will be 1. Thus, if several motors with ID 1 are
	 * connected on the same bus, there will not be a way to assign them unique IDs without
	 * first disconnecting them.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	// Define arrays for transmission and reception
	uint8_t arrTransmit[6];

	// Do assignments and computations
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = 0x06; // READ DATA instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	// Set data direction
	__DYNAMIXEL_TRANSMIT();

	// Transmit
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
}
