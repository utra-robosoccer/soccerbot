

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
	HAL_Delay(10);
	Dynamixel_SetGoalVelocity(hdynamixel, MAX_VELOCITY * 0.5);
	HAL_Delay(10);
	Dynamixel_SetGoalPosition(hdynamixel, 150);
	HAL_Delay(10);
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
