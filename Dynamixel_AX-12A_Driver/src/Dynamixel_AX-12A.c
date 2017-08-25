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
/*******************************************************************************/
void Dynamixel_SetID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID){
	/* Sets the ID (identification number) for the current motor.
	 * Note that the instruction will be broadcasted using the current ID.
	 * As such, if the ID is not known, the motor ID should be initialized
	 * to the broadcast ID (0xFE) in the Dynamixel_Init function.
	 *
	 * Instruction register address: 0x03 (EEPROM)
	 * Default value: 1
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  ID, the number between 0 and 252 or equal to 254 to identify the motor.
	 * 			  	If 0xFE (254), any messages broadcasted to that ID will be broadcasted
	 * 			  	to all motors.
	 *
	 * Returns: none
	 */


	/* Compute validity. */
	if((ID < 0) || (ID == 253) || (ID > 254)){
		ID = DEFAULT_ID;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_ID, ID, -1);
	hdynamixel->_ID = ID;
}

// TODO: Test
void Dynamixel_SetBaudRate(Dynamixel_HandleTypeDef* hdynamixel, double baud){
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

	/* Set _baud equal to the hex code corresponding to baud. Default to 1 Mbps. */
	if(baud > 0){
		/* Valid for baud in range [7844, 1000000]. Will be converted to 8-bit resolution. */
		hdynamixel -> _BaudRate = (uint8_t)((2000000 / baud) - 1);
	}
	else{
		/* Default to 1 Mbps. */
		hdynamixel -> _BaudRate = DEFAULT_BAUD_RATE;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_BAUD_RATE, hdynamixel -> _BaudRate, -1);
}

// TODO: Test
void Dynamixel_SetReturnDelayTime(Dynamixel_HandleTypeDef* hdynamixel, double microSec){
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

	/* Compute validity. */
	uint8_t motor_data;
	if((microSec < 1) || (microSec > 508)){
		motor_data = DEFAULT_RETURN_DELAY;
	}
	else{
		motor_data = (uint8_t)(microSec / 2);
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_RETURN_DELAY_TIME, motor_data, -1);
}

void Dynamixel_SetCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, double minAngle){
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

	/* Initialize local variable to default value. */
	uint16_t normalized_value = DEFAULT_CW_ANGLE_LIMIT;

	/* Evaluate argument validity. Optimize for edge case minAngle == MIN_ANGLE. */
	if((minAngle > MIN_ANGLE) && (minAngle <= MAX_ANGLE)){

		/* Translate the angle from degrees into a 10-bit number. */
		normalized_value = (uint16_t)(minAngle / MAX_ANGLE * 1023);
	}

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of CW angle limit
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of CW angle limit

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, REG_CW_ANGLE_LIMIT, lowByte, highByte);
}

void Dynamixel_SetCCWAngleLimit(Dynamixel_HandleTypeDef* hdynamixel, double maxAngle){
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

	/* Initialize local variable to default value. */
	uint16_t normalized_value = DEFAULT_CCW_ANGLE_LIMIT;

	/* Evaluate argument validity. Optimize for edge case maxAngle = MAX_ANGLE. */
	if((maxAngle >= MIN_ANGLE) && (maxAngle < MAX_ANGLE)){

		/* Translate the angle from degrees into a 10-bit number. */
		normalized_value = (uint16_t)(maxAngle / MAX_ANGLE * 1023);
	}

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of CCW angle limit
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of CCW angle limit

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, REG_CCW_ANGLE_LIMIT, lowByte, highByte);
}

// TODO: Test
void Dynamixel_SetHighestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, double highestVoltage){
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

	/* Declare local variable. Initialize to default value. */
	uint8_t high_voltage_data = DEFAULT_HIGH_VOLTAGE_LIMIT;

	/* Evaluate argument validity and translate into motor data. Optimize for highestVoltage = MAX_VOLTAGE. */
	if((highestVoltage >= MIN_VOLTAGE) && (highestVoltage < MAX_VOLTAGE)){
		high_voltage_data = (uint8_t)(highestVoltage * 10);
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_HIGH_VOLTAGE_LIMIT, high_voltage_data, -1);
}

// TODO: Test
void Dynamixel_SetLowestVoltageLimit(Dynamixel_HandleTypeDef* hdynamixel, double lowestVoltage){
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

	/* Declare local variable. Initialize to default value. */
	uint8_t low_voltage_data = DEFAULT_LOW_VOLTAGE_LIMIT;

	/* Evaluate argument validity and translate into motor data. Optimize for lowestVoltage = MIN_VOLTAGE. */
	if((lowestVoltage > MIN_VOLTAGE) && (lowestVoltage <= MAX_VOLTAGE)){

		/* Translate into format that motor can understand. */
		low_voltage_data = (uint8_t)(lowestVoltage * 10);
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_LOW_VOLTAGE_LIMIT, low_voltage_data, -1);
}

// TODO: Test
void Dynamixel_SetMaxTorque(Dynamixel_HandleTypeDef* hdynamixel, double maxTorque){
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

	/* Initialize to default value. */
	uint16_t normalized_value = DEFAULT_MAXIMUM_TORQUE;

	/* Evaluate argument validity and optimize for edge case maxTorque = MAX_TORQUE. */
	if((maxTorque >= MIN_TORQUE) && (maxTorque < MAX_TORQUE)){

		/* Translate the input from percentage into a 10-bit number. */
		normalized_value = (uint16_t)(maxTorque / 100 * 1023);
	}

	uint8_t lowByte = normalized_value & 0xFF; // Low byte of max torque
	uint8_t highByte = (normalized_value >> 8) & 0xFF; // High byte of max torque

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, REG_MAX_TORQUE, lowByte, highByte);
}

// TODO: Test
void Dynamixel_SetStatusReturnLevel(Dynamixel_HandleTypeDef* hdynamixel, uint8_t status_data){
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

	/* Evaluate argument validity. */
	if((status_data < 0) || (status_data > 2)){
		status_data = DEFAULT_STATUS_RETURN_LEVEL;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_STATUS_RETURN_LEVEL, status_data, -1);
}

// TODO: Test
void Dynamixel_SetAlarmLED(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_LED_data){
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
	 * 			  			  bit 0: flash LED when applied voltage is out of operating range
	 *
	 * Several of these bits may be set simultaneously.
	 *
	 * Returns: none
	 */

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_ALARM_LED, alarm_LED_data, -1);
}

// TODO: Test
void Dynamixel_SetAlarmShutdown(Dynamixel_HandleTypeDef* hdynamixel, uint8_t alarm_shutdown_data){
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
	 * 			  			  bit 0: torque off when applied voltage is out of operating range
	 *
	 * Several of these bits may be set simultaneously.
	 *
	 * Returns: none
	 */

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_ALARM_SHUTDOWN, alarm_shutdown_data, -1);
}

// TODO: Test
void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
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

	/* Evaluate argument validity. */
	if((isEnabled != 1) && (isEnabled != 0)){
		isEnabled = DEFAULT_TORQUE_ENABLE;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_TORQUE_ENABLE, isEnabled, -1);
}

void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
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

	/* Evaluate argument validity. */
	if((isEnabled != 1) && (isEnabled != 0)){
		isEnabled = DEFAULT_LED_ENABLE;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_LED_ENABLE, isEnabled, -1);
}

// TODO: Test
void Dynamixel_SetCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceMargin){
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
	Dynamixel_DataWriter(hdynamixel, 8, REG_CW_COMPLIANCE_MARGIN, CWcomplianceMargin, -1);
}

// TODO: Test
void Dynamixel_SetCCWComplianceMargin(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceMargin){
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
	Dynamixel_DataWriter(hdynamixel, 8, REG_CCW_COMPLIANCE_MARGIN, CCWcomplianceMargin, -1);
}

// TODO: Test
void Dynamixel_SetCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CWcomplianceSlope){
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
		step = DEFAULT_CW_COMPLIANCE_SLOPE;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_CW_COMPLIANCE_SLOPE, step, -1);
}

// TODO: Test
void Dynamixel_SetCCWComplianceSlope(Dynamixel_HandleTypeDef* hdynamixel, uint8_t CCWcomplianceSlope){
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
		step = DEFAULT_CCW_COMPLIANCE_SLOPE;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_CCW_COMPLIANCE_SLOPE, step, -1);
}

void Dynamixel_SetGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, double goalAngle){
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
	uint16_t normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 1023);

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal position
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal position

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, REG_GOAL_POSITION, lowByte, highByte);
}

void Dynamixel_SetGoalVelocity(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity){
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
	if(hdynamixel -> _isJointMode){

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
	Dynamixel_DataWriter(hdynamixel, 9, REG_GOAL_VELOCITY, lowByte, highByte);
}

// TODO: Test
void Dynamixel_SetGoalTorque(Dynamixel_HandleTypeDef* hdynamixel, double goalTorque){
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

	/* Translate the input from percentage into and 10-bit number. */
	uint16_t normalized_value = (uint16_t)(goalTorque / 100 * 1023);

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal torque
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal torque

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, REG_GOAL_TORQUE, lowByte, highByte);
}

void Dynamixel_LockEEPROM(Dynamixel_HandleTypeDef* hdynamixel){
	/* Locks the EEPROM of the current motor until the next power cycle.
	 *
	 * Instruction register address: 0x2F (RAM)
	 * Default value: 0x00
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, REG_LOCK_EEPROM, 1, -1);
}

// TODO: Test
void Dynamixel_SetPunch(Dynamixel_HandleTypeDef* hdynamixel, double punch){
	/* Sets a quantity proportional to the minimum current supplied to the motor during operation.
	 * Units are not specified in datasheet, and therefore this function is not entirely useful without
	 * sufficient testing.
	 *
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

	/* Evaluate argument validity. */
	if((punch < MIN_PUNCH) || (punch > MAX_PUNCH)){
		if(punch < MIN_PUNCH){
			punch = MIN_PUNCH;
		}
		else{
			punch = MAX_PUNCH;
		}
	}

	/* Translate the punch into a 10-bit number. */
	uint16_t normalized_value = punch;

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of punch
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of punch

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, REG_PUNCH, lowByte, highByte);
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
/*******************************************************************************/
// TODO: Test
void Dynamixel_GetPosition(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads addresses 0x24 and 0x25 in the motors RAM to see what the current position
	 * of the motor is.
	 * Writes the results to hdynamixel -> _lastPosition
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_POSITION, 2);

	/* Parse data and write it into motor handle. */
	hdynamixel -> _lastPosition = (uint16_t)(retVal * 300 / 1023);
}

// TODO: Test
void Dynamixel_GetVelocity(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads addresses 0x26 and 0x27 in the motor RAM to see what the current velocity
	 * of the motor is.
	 * Writes the results to hdynamixel -> _lastVelocity
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_VELOCITY, 2);

	/* Parse data and write it into motor handle. */
	uint16_t modifier;
	if(hdynamixel -> _isJointMode){
		modifier = 1023;
	}
	else{
		modifier = 2047;
	}
	hdynamixel -> _lastVelocity = (float)(retVal / modifier * 114);
}

// TODO: Test
void Dynamixel_GetLoad(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads addresses 0x28 and 0x29 in the motor RAM to see what the current
	 * load is. Load is a percentage of the maximum torque. Value of 0-1023
	 * gets translated into a counterclockwise load, and value of 1024-2047
	 * gets translated into a clockwise load.
	 *
	 * Writes the torque percentage to hdynamixel -> _lastLoad
	 * Writes the torque direction to hdynamixel -> _lastLoadDirection
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint16_t retVal = Dynamixel_DataReader(hdynamixel, REG_CURRENT_LOAD, 2);

	/* Parse data and write it into motor handle. */
	hdynamixel -> _lastLoadDirection = (retVal & 0x02FF);
	if(retVal > 1023){
		retVal = retVal - 1023;
	}
	hdynamixel -> _lastLoad = (uint8_t)(retVal / 1023 * 100);
}

// TODO: Test
void Dynamixel_GetVoltage(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads address 0x2A in the motor RAM to see what the current voltage is.
	 * Value retrieved from motor is 10 times the actual voltage.
	 * Writes the results to hdynamixel -> _lastVoltage
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Read data from motor. */
	uint8_t retVal = (uint8_t)Dynamixel_DataReader(hdynamixel, REG_CURRENT_VOLTAGE, 1);

	/* Parse data and write it into motor handle. */
	hdynamixel -> _lastVoltage = (float)(retVal / 10);
}

// TODO: Test
void Dynamixel_GetTemperature(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads address 0x2B in the motor RAM to see what the current temperature is inside the motor.
	 * Results in degrees Celsius.
	 * Writes the results to hdynamixel -> _lastTemperature
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Read data from motor straight into the motor handle. */
	hdynamixel -> _lastTemperature = (uint8_t)Dynamixel_DataReader(hdynamixel, REG_CURRENT_TEMPERATURE, 1);
}

// TODO: Test
uint8_t Dynamixel_IsRegistered(Dynamixel_HandleTypeDef* hdynamixel){
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

	/* Return the data read from the motor. */
	return((uint8_t)Dynamixel_DataReader(hdynamixel, REG_REGISTERED, 1));
}

// TODO: Test
uint8_t Dynamixel_IsMoving(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads the 0x2E address in motor RAM to see if motor is moving.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: 0 if not moving
	 * 			1 if moving
	 */

	/* Return the data from the motor. */
	return((uint8_t)Dynamixel_DataReader(hdynamixel, REG_MOVING, 1));
}

// TODO: Test
uint8_t Dynamixel_IsJointMode(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reads the CW (addr: 0x06) and CCW (addr: 0x08) angle limits. If both are 0, motor is in wheel mode
	 * and can spin indefinitely. Otherwise, motor is in joint mode and has angle limits set
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: 0 if in wheel mode
	 * 			1 if in joint mode
	 */

	/* Read data from motor. */
	uint16_t retValCW = Dynamixel_DataReader(hdynamixel, REG_CW_ANGLE_LIMIT, 2);
	uint16_t retValCCW = Dynamixel_DataReader(hdynamixel, REG_CCW_ANGLE_LIMIT, 2);

	/* Parse data and write it into motor handle. */
	hdynamixel -> _isJointMode = (uint8_t)((retValCW | retValCCW) != 0);

	/* Return. */
	return(hdynamixel -> _isJointMode);
}




/*******************************************************************************/
/*	Other motor instruction helper functions								   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
// TODO: Test
void Dynamixel_RegWrite(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, \
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

	/* Define arrays for transmission. */
	uint8_t arrTransmit[arrSize];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_REG_WRITE; // REG WRITE instruction
	arrTransmit[5] = writeAddr;
	arrTransmit[7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = Dynamixel_ComputeChecksum(arrTransmit, arrSize);
	}

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT();

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);
}

// TODO: Test
void Dynamixel_Action(Dynamixel_HandleTypeDef* hdynamixel){
	/* Implements the ACTION instruction. This triggers the instruction registered by the REG WRITE
	 * instruction. This way, time delays can be reduced for the concurrent motion of several
	 * motors.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Define arrays for transmission and reception. */
	uint8_t arrTransmit[6];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_ACTION; // ACTION instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT();

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
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
/*******************************************************************************/
//TODO: Test
uint8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel){
	/* Used only for returning a status packet or checking the existence of a motor
	 * with a specified ID. Does not command any operations.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: motor ID seen in status packet
	 */

	/* Define arrays for transmission and reception. */
	uint8_t arrTransmit[6];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_PING; // PING instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT();

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);

	/* Set data direction for receive. */
	__DYNAMIXEL_RECEIVE();

	/* Receive. */
	HAL_UART_Receive(hdynamixel -> _UART_Handle, arrTransmit, 6, RECEIVE_TIMEOUT);
	return(arrTransmit[2]);
}

/* TODO: Discuss locking the critical sections, using interrupts, and whether it is
 * desired to return a status packet and evaluate it for communication validity. */
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arrSize, \
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

	/* Check that array size is 8 or 9. */
	if(!((arrSize == 8) || (arrSize == 9))){
		_Error_Handler(__FILE__, __LINE__);
		return;
	}

	/* Do assignments and computations. */
	uint8_t ID = hdynamixel -> _ID;
	arrTransmit[ID][3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[ID][4] = INST_WRITE_DATA; // WRITE DATA instruction
	arrTransmit[ID][5] = writeAddr; // Write address for register
	arrTransmit[ID][6] = param1;

	/* Checksum. */
	arrTransmit[ID][7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit[ID], arrSize): param2;
	if(arrSize == 9){
		arrTransmit[ID][8] = Dynamixel_ComputeChecksum(arrTransmit[ID], arrSize);
	}

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT();

	/* In the future, a lock should be placed here, and it should be unlocked after the status
	 * packet is received. This requires using interrupts for TX/RX and it cannot be implemented
	 * right now because the blocking transmit and receive use timer interrupts for their timeout
	 */

	/* Transmit. */
	#if TRANSMIT_IT
		HAL_UART_Transmit_IT(hdynamixel -> _UART_Handle, arrTransmit[ID], arrSize);
	#else
		HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit[ID], arrSize, TRANSMIT_TIMEOUT);
	#endif
}

// UNIMPLEMENTED
void Dynamixel_SyncWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t uartIndex, uint8_t numParams, uint8_t *params){
	/* Used for sending control signals to several specified Dynamixel actuators concurrently.
	 * Uses the SYNC WRITE instruction, 0x83, in the motor instruction set. Note that the
	 * broadcast ID is used, so all motors attached to the same UART will have to spend time
	 * processing the command packet, even if they are not addressed in it.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  uartIndex, the index of the UART which the motors to be communicated with are routed to
	 * 			  numParams, the number of parameters to be transmitted from the arrSyncWrite buffer
	 * 			  params, the array holding all the instructions and parameters to be passed to the various actuators
	 *
	 * Params must be in the following format:
	 * 		1st element --> starting address of dynamixel control table where writing should begin
	 * 		2nd element --> Length of data to be written to EACH motor (not total length)
	 * 		3rd element --> ID of 1st motor
	 * 		4th element --> 1st parameter for 1st motor...
	 * 		L+3 element --> Lth parameter for 1st motor...
	 * 		L+4 element --> ID of 2nd motor
	 * 		L+5 element -->	1st parameter for 2nd motor...
	 * 		2L+4 element --> Lth parameter for 2nd motor...
	 * 		etc.
	 *
	 * Returns: none
	 */

	/* Write packet length into transmission array. */
	arrSyncWrite[uartIndex][3] = numParams + 4; // Length of packet + len(INST_SYNC_WRITE, arrSyncWrite[4], arrSyncWrite[5], checksum)

	/* Copy parameters into transmission array. */
	for(uint8_t i = 0; i < numParams; i++){
		arrSyncWrite[uartIndex][i + 5] = params[i]; // Start writing into arrSyncWrite at 5th element
	}

	/* Compute checksum. */
	arrSyncWrite[uartIndex][numParams + 4] = Dynamixel_ComputeChecksum(arrSyncWrite[uartIndex], numParams + 5);

	/* Transmit data. Number of bytes to be transmitted is numParams + len(0xFF, 0xFF, 0xFE, length, 0x83, checksum). */
	#if TRANSMIT_IT
		HAL_UART_Transmit_IT(hdynamixel -> _UART_Handle, arrSyncWrite[uartIndex], numParams + 6);
	#else
		HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrSyncWrite[uartIndex], numParams + 6, TRANSMIT_TIMEOUT);
	#endif
}

// TODO: Fix the checksum verification so that there are no stalls
uint16_t Dynamixel_DataReader(Dynamixel_HandleTypeDef* hdynamixel, uint8_t readAddr, uint8_t readLength){
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

	/* Clear array for reception. */
	uint8_t ID = hdynamixel -> _ID;
	for(uint8_t i = 0; i < BUFF_SIZE_RX; i++){
		arrReceive[ID][i] = 0;
	}

	/* Do assignments and computations. */
	arrTransmit[ID][3] = 4; // Length of message minus the obligatory bytes
	arrTransmit[ID][4] = INST_READ_DATA; // READ DATA instruction
	arrTransmit[ID][5] = readAddr; // Write address for register
	arrTransmit[ID][6] = readLength; // Number of bytes to be read from motor
	arrTransmit[ID][7] = Dynamixel_ComputeChecksum(arrTransmit[ID], 8);

	// Ensure that received data has valid checksum. If it does not, send data request again
//	uint8_t valid = 0;
//	while(!valid){
		// Set data direction for transmit
		__DYNAMIXEL_TRANSMIT();

		// Transmit
	#if TRANSMIT_IT
		HAL_UART_Transmit_IT(hdynamixel -> _UART_Handle, arrTransmit[ID], 8);
	#else
		HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit[ID], 8, TRANSMIT_TIMEOUT);
	#endif

		// Set data direction for receive
		__DYNAMIXEL_RECEIVE();

		// Call appropriate UART receive function depending on if 1 or 2 bytes are to be read
		if(readLength == 1){
			HAL_UART_Receive(hdynamixel -> _UART_Handle, arrReceive[ID], 7, RECEIVE_TIMEOUT);
			//valid = (Dynamixel_ComputeChecksum(arrReceive[ID], 7) == arrReceive[ID][6]); // Verify checksums match
		}
		else{
			HAL_UART_Receive(hdynamixel -> _UART_Handle, arrReceive[ID], 8, RECEIVE_TIMEOUT);
//			valid = (Dynamixel_ComputeChecksum(arrReceive[ID], 8) == arrReceive[ID][7]); // Verify checksums match
		}
//	}

	// Check the status packet received for errors
	if(arrReceive[ID][5] != 0){

		/* Call the error handler with the error code as the argument. */
		Dynamixel_ErrorHandler(arrReceive[ID][5]);
	}

	if(readLength == 1){
		return (uint16_t)arrReceive[ID][5];
	}
	else{
		return (uint16_t)(arrReceive[ID][5] | (arrReceive[ID][6] << 8));
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
/*******************************************************************************/
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef *UART_Handle){
	/* Initializes the motor handle.
	 *
	 * Arguments: hdynamixel, the motor handle to be initialized
	 * 			  ID, the ID the motor has. Note that this function will not set
	 * 			  	  the ID in case there are multiple actuators on the same bus
	 * 			  UART_Handle, the handle to the UART that will be used to
	 * 			      communicate with this motor
	 *
	 * Returns: none
	 */

	/* Set fields in motor handle. */
	hdynamixel -> _ID = ID; // Motor ID (unique or global)
	hdynamixel -> _BaudRate = 1000000; // Number of bytes per second transmitted by the UART
	hdynamixel -> _lastPosition = -1; // In future, could initialize this accurately
	hdynamixel -> _lastVelocity = -1; // In future, could initialize this accurately
	hdynamixel -> _lastLoad = -1; // In future, could initialize this accurately
	hdynamixel -> _lastLoadDirection = -1; // In future, could initialize this accurately
	hdynamixel -> _lastVoltage = -1; // In future, could initialize this accurately
	hdynamixel -> _lastTemperature = -1; // In future, could initialize this accurately
	hdynamixel -> _isJointMode = 1; // In future, could initialize this accurately
	hdynamixel -> _UART_Handle = UART_Handle; // For UART TX and RX

	/* Motor buffer initialization. */
	/* ----> Sync write buffer <---- */
	arrSyncWrite[ID][0] = 0xFF;
	arrSyncWrite[ID][1] = 0xFF;
	arrSyncWrite[ID][2] = 0xFE;
	arrSyncWrite[ID][4] = INST_SYNC_WRITE;

	/* Configure motor to return status packets only for read commands. */
	Dynamixel_SetStatusReturnLevel(hdynamixel, 1);
}

void Dynamixel_Reset(Dynamixel_HandleTypeDef* hdynamixel){
	/* Resets the control table values of the motor to the Factory Default Value settings.
	 * Note that post-reset, motor ID will be 1. Thus, if several motors with ID 1 are
	 * connected on the same bus, there will not be a way to assign them unique IDs without
	 * first disconnecting them.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	/* Define array for transmission. */
	uint8_t arrTransmit[6];

	/* Do assignments and computations. */
	arrTransmit[0] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xff; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_RESET; // Reset instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT();

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
	hdynamixel -> _ID = DEFAULT_ID;

	/* Wait for motor to finish resetting. */
	HAL_Delay(500); // Should find a way of polling the motor here
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
/*******************************************************************************/
// TODO: Write function
void Dynamixel_ErrorHandler(uint8_t errCode){
	/* Handles errors raised in error code bytes of status packets.
	 *
	 * Arguments: errCode, the error code returned by the status packet
	 *
	 * Returns: none
	 */

	/* TODO: write function. */
	return;
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
void Dynamixel_Revive(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID){
	/* Resets the motor corresponding to the handle, sets its ID to a custom value,
	 * and demonstrates the validity of the previous actions by rotating
	 * through its full angle span (joint mode).
	 *
	 * This function should be used when a motor is unresponsive, and the
	 * health of the motor wants to be verified. Basically, it's used
	 * to see whether a motor is bricked.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  ID, the desired motor ID
	 *
	 * Returns: none
	 */

	Dynamixel_Reset(hdynamixel);
	Dynamixel_SetID(hdynamixel, ID);
	Dynamixel_SetGoalVelocity(hdynamixel, MAX_VELOCITY * 0.75);
	Dynamixel_SetGoalPosition(hdynamixel, 300);
	HAL_Delay(1500);
	Dynamixel_SetGoalPosition(hdynamixel, 0);
}

void Dynamixel_BroadcastRevive(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID){
	/* Resets all motors on the bus, sets ID to a custom value,
	 * and demonstrates the validity of the previous actions by rotating
	 * through its full angle span (joint mode).
	 *
	 * This function should be used when the health of all motors connected to
	 * the bus are to be checked by a standardized testing procedure.
	 * Basically, it's used to see whether any motor connected to the bus is bricked.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  ID, the desired motor ID
	 *
	 * Returns: none
	 */

	hdynamixel -> _ID = 0xFE;
	Dynamixel_Revive(hdynamixel, ID);
}

void Dynamixel_EnterWheelMode(Dynamixel_HandleTypeDef* hdynamixel, double goalVelocity){
	/* Sets the control registers such that the rotational angle of the motor
	 * is not bounded.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  goalVelocity, the desired velocity to use when entering wheel mode
	 *
	 * Returns: none
	 */

	/* When the angle limits are both set to 0, then motor will attempt to
	 * rotate with maximum velocity. To prevent undesired behaviour, the
	 * goal velocity should be set right after calling this function */
	Dynamixel_SetCWAngleLimit(hdynamixel, 0);
	Dynamixel_SetCCWAngleLimit(hdynamixel, 0);
	hdynamixel -> _isJointMode = 0;
	Dynamixel_SetGoalVelocity(hdynamixel, goalVelocity);
}

// TODO: Re-test moving motor to nearest valid position
void Dynamixel_EnterJointMode(Dynamixel_HandleTypeDef* hdynamixel){
	/* Sets the control registers such that the rotational angle of the motor
	 * is constrained between the default values.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	// In the future, it would be good to make this robust by returning the motor
	// to the nearest valid position. Otherwise, motor gets confused because it's
	// locked out of its valid boundaries and there you can't instruct it to go anywhere
//	Dynamixel_GetPosition(hdynamixel);
//	while((hdynamixel -> _lastPosition < MIN_ANGLE) || (hdynamixel -> _lastPosition > MAX_ANGLE)){
//		if(360 - hdynamixel -> _lastPosition < 15){
//			Dynamixel_SetGoalVelocity(hdynamixel, MAX_VELOCITY);
//		}
//		else{
//			Dynamixel_SetGoalVelocity(hdynamixel, -MAX_VELOCITY);
//		}
//		Dynamixel_GetPosition(hdynamixel);
//	}

	hdynamixel -> _isJointMode = 1;
	Dynamixel_SetCWAngleLimit(hdynamixel, MIN_ANGLE);
	Dynamixel_SetCCWAngleLimit(hdynamixel, MAX_ANGLE);
}




/*******************************************************************************/
/*	Test/demonstration functions											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
// TODO: Finish writing tests
void Dynamixel_TestAll(Dynamixel_HandleTypeDef** arrHdynamixel, uint8_t arrSize){
	/* Demonstrates the various functions available. The functionality this
	 * function demonstrates can be used as an indicator for the integrity of
	 * the code.
	 *
	 * Note that for all functions to pass, the motor(s) passed in should
	 * be in their reset state and should have different IDs, as applicable
	 *
	 * Arguments: arrHdynamixel, the array handles for the connected motors
	 *
	 * Returns: none
	 */

	/* Local variable declaration and initialization. */
	Dynamixel_HandleTypeDef MASTER_MOTOR_CONTROL;
	Dynamixel_HandleTypeDef* Motor1 = arrHdynamixel[0];

	/* Initialize master UART handle to that of the first motor handle. */
	Dynamixel_Init(&MASTER_MOTOR_CONTROL, 0xFE, arrHdynamixel[0]->_UART_Handle);


	/* Test 1: Dynamixel_LEDEnable
	 * Pass condition: Motor LED(s) blink(s) on then off
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_LEDEnable(arrHdynamixel[i], 1);
	}
	HAL_Delay(500);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_LEDEnable(arrHdynamixel[i], 0);
	}
	HAL_Delay(500);


	/* Test 2: Dynamixel_SetID
	 * Pass condition: Motor LED blinks on then off
	 */
	uint8_t oldID = Motor1 -> _ID; // Save ID for later restoration
	Dynamixel_SetID(Motor1, (Motor1 ->_ID + 20) % 0xFE); // Different, valid ID
	Dynamixel_LEDEnable(Motor1, 1); // Set LED on with new motor ID
	HAL_Delay(500);
	Dynamixel_LEDEnable(Motor1, 0); // Set LED off with new motor ID
	Dynamixel_SetID(Motor1, oldID); // Restore previous state
	HAL_Delay(500);


	/* Test 3: Dynamixel_SetGoalPosition
	 * Pass condition: Motor(s) attached to TX/RX line are seen rotating
	 * between positions of 0 degrees and 300 degrees (roughly)
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MAX_ANGLE);
	}
	HAL_Delay(1500);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MIN_ANGLE);
	}
	HAL_Delay(1500);


	/* Test 4: Dynamixel_SetGoalVelocity
	 * Pass condition: All motors connected move at different speeds
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalVelocity(arrHdynamixel[i], MAX_VELOCITY / (i*2 + 1));
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MAX_ANGLE);
	}
	HAL_Delay(1500);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalVelocity(arrHdynamixel[i], MAX_VELOCITY / (i*2 + 1));
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MIN_ANGLE);
	}
	HAL_Delay(1500);
	/* Set motors to middle position with identical velocities. */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalVelocity(arrHdynamixel[i], MAX_VELOCITY);
		Dynamixel_SetGoalPosition(arrHdynamixel[i], (MIN_ANGLE + MAX_ANGLE) / 2);
	}
	HAL_Delay(1500);


	/* ODD NOTE: with the current code, if the below line is uncommented,
	 * then Motor2 will rotate in the CW direction in the last half of test 7.
	 * Otherwise, if Motor1's EEPROM is not locked, Motor2 doesn't respond
	 * properly to to the CW direction instruction...
	 *
	 * A possible reason for this is that the data writer does not take the time to
	 * read a status packet back, and thus its data might be getting mixed with the
	 * status packet signals the motor is trying to send back
	 */
//	Dynamixel_LockEEPROM(arrHdynamixel[0], 1); 		// If uncommented, the effects of
													// settings angle limits will be clearly
													// visible
//	for(int i = 0; i < arrSize; i++){
//		Dynamixel_SetStatusReturnLevel(arrHdynamixel[i], 1);
//	}


	/* Test 5: Dynamixel_SetCWAngleLimit
	 * Pass condition: Motor(s) move between 150 and 300 degrees when
	 * instructed to move between 0 and 300 degrees
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetCWAngleLimit(arrHdynamixel[i], 150);
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MAX_ANGLE);
	}
	HAL_Delay(750);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetCWAngleLimit(arrHdynamixel[i], 150);
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MIN_ANGLE);
	}
	HAL_Delay(750);
	/* Set back to defaults. */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetCWAngleLimit(arrHdynamixel[i], 0);
	}
	HAL_Delay(375);


	/* Test 6: Dynamixel_SetCCWAngleLimit
	 * Pass condition: Motor(s) move between 0 and 150 degrees when
	 * instructed to move between 0 and 300 degrees
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetCCWAngleLimit(arrHdynamixel[i], 150);
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MIN_ANGLE);
	}
	HAL_Delay(750);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetCCWAngleLimit(arrHdynamixel[i], 150);
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MAX_ANGLE);
	}
	HAL_Delay(750);
	/* Set back to defaults. */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetCCWAngleLimit(arrHdynamixel[i], 300);
	}
	HAL_Delay(375);


	/* Test 7: Dynamixel_EnterWheelMode
	 * Pass: Motor(s) rotate through over 360 degrees, first in the CCW direction,
	 * then in the counterclockwise direction
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_EnterWheelMode(arrHdynamixel[i], MAX_VELOCITY);
		Dynamixel_SetGoalVelocity(arrHdynamixel[i], MAX_VELOCITY);
	}
	HAL_Delay(1500);
	for(int i = 0; i < arrSize; i++){

		/* Motor should ignore the goal position command */
		//Dynamixel_SetGoalPosition(arrHdynamixel[i], MIN_ANGLE);
		Dynamixel_SetGoalVelocity(arrHdynamixel[i], -MAX_VELOCITY);
		HAL_Delay(10);
	}
	HAL_Delay(1500);


	/* Test 8: Dynamixel_EnterJointMode
	 * Pass: Motor(s) rotate between 0 and 300 degrees, then return to middle position
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_EnterJointMode(arrHdynamixel[i]);
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MIN_ANGLE);
	}
	HAL_Delay(1500);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalPosition(arrHdynamixel[i], MAX_ANGLE);
	}
	HAL_Delay(1500);
	/* Return to middle position. */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_SetGoalPosition(arrHdynamixel[i], (MIN_ANGLE + MAX_ANGLE) / 2);
	}
	HAL_Delay(1500);


	/*FAILS*/
	/* Test 9: Dynamixel_IsJointMode
	 * Pass: Motor LEDs are brightened for 1 second, then go off for 1 second,
	 * then go on for 1 second, then go off.
	 */
	for(int i = 0; i < arrSize; i++){
		Dynamixel_LEDEnable(arrHdynamixel[i], Dynamixel_IsJointMode(arrHdynamixel[i]));
	}
	HAL_Delay(1000);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_LEDEnable(arrHdynamixel[i], 0);
	}
	HAL_Delay(1000);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_EnterWheelMode(arrHdynamixel[i], 0);
		Dynamixel_LEDEnable(arrHdynamixel[i], (Dynamixel_IsJointMode(arrHdynamixel[i]) == 0));
	}
	HAL_Delay(1000);
	for(int i = 0; i < arrSize; i++){
		Dynamixel_EnterJointMode(arrHdynamixel[i]);
		Dynamixel_SetGoalVelocity(arrHdynamixel[i], MAX_VELOCITY);
		Dynamixel_LEDEnable(arrHdynamixel[i], 0);
	}
}
