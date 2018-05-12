/********************************* Includes ************************************/
#include "AX12A.h"

const uint8_t TRANSMIT_TIMEOUT = 10; 	// Timeout for blocking UART transmissions, in milliseconds
const uint8_t RECEIVE_TIMEOUT = 10;		// Timeout for blocking UART receptions, in milliseconds

const double MAX_VELOCITY = 114;	// Maximum angular velocity (RPM)
const double MIN_VELOCITY = 1;		// Minimum angular velocity (RPM)
const uint16_t MAX_ANGLE = 300;		// Maximum angular position (joint mode)
const uint8_t MIN_ANGLE = 0;		// Minimum angular position (joint mode)


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


	uint8_t arrTransmit[9];

	/* Do assignments and computations. */
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[5] = writeAddr; // Write address for register
	arrTransmit[6] = param1;

	/* Checksum. */
	arrTransmit[7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = Dynamixel_ComputeChecksum(arrTransmit, arrSize);
	}

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* In the future, a lock should be placed here, and it should be unlocked after the status
	 * packet is received. This requires using interrupts for TX/RX and it cannot be implemented
	 * right now because the blocking transmit and receive use timer interrupts for their timeout
	 */

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);
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
void Dynamixel_Init(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID, UART_HandleTypeDef *UART_Handle,\
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum){
	/* Initializes the motor handle.
	 *
	 * Arguments: hdynamixel, the motor handle to be initialized
	 * 			  ID, the ID the motor has. Note that this function will not set
	 * 			  	  the ID in case there are multiple actuators on the same bus
	 * 			  UART_Handle, the handle to the UART that will be used to
	 * 			      communicate with this motor
	 * 			  DataDirPort, the pointer to the port that the data direction pin
	 * 			  	  for the motor is on
	 * 			  DataDirPinNum, the number corresponding to the pin that controls
	 * 			      data direction (a power of two, e.g. 2^0 for pin 0, 2^15 for pin 15)
	 *
	 * Returns: none
	 */

	/* Set fields in motor handle. */
	hdynamixel -> _ID = ID; 					// Motor ID (unique or global)
	hdynamixel -> _BaudRate = 1000000; 			// Number of bytes per second transmitted by the UART
	hdynamixel -> _lastPosition = 0; 			// In future, could initialize this accurately
	hdynamixel -> _lastVelocity = -1; 			// In future, could initialize this accurately
	hdynamixel -> _lastLoad = -1; 				// In future, could initialize this accurately
	hdynamixel -> _lastLoadDirection = -1; 		// In future, could initialize this accurately
	hdynamixel -> _lastVoltage = -1; 			// In future, could initialize this accurately
	hdynamixel -> _lastTemperature = -1; 		// In future, could initialize this accurately
	hdynamixel -> _isJointMode = 1; 			// In future, could initialize this accurately
	hdynamixel -> _UART_Handle = UART_Handle; 	// For UART TX and RX
	hdynamixel -> _dataDirPort = DataDirPort;
	hdynamixel -> _dataDirPinNum = DataDirPinNum;
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
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
	hdynamixel -> _ID = DEFAULT_ID;

	/* Wait for motor to finish resetting. */
	HAL_Delay(500); // Should find a way of polling the motor here
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

	Dynamixel_Reset(hdynamixel);
	Dynamixel_SetID(hdynamixel, ID);
	HAL_Delay(10);
	Dynamixel_SetGoalVelocity(hdynamixel, MAX_VELOCITY * 0.5);
	HAL_Delay(10);
	Dynamixel_SetGoalPosition(hdynamixel, 150);
	HAL_Delay(10);
}
