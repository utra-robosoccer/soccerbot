/********************************* Includes ************************************/
#include "MX28.h"




/********************************* Constants *********************************/
const uint8_t TRANSMIT_TIMEOUT = 10;
const uint8_t RECEIVE_TIMEOUT = 10;




/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = {{0}};

/* Bytes to be transmitted to motors are written to this array. */
uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE] = {
	{0xFF, 0xFF, 0xFE, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 1, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 2, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 3, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 4, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 5, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 6, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 7, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 8, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 9, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 10, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 11, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 12, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 13, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 14, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 15, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 16, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 17, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 18, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00}
};




/******************************* Private Variables *******************************/
uint16_t crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};



/************************ Private Function Prototypes **************************/
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length);
uint16_t update_crc(uint16_t crc_accum, uint8_t* data_blk_ptr, uint16_t data_blk_size);


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
void setID(Dynamixel_HandleTypeDef* hdynamixel, uint8_t ID){
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
	if((ID == 253) || (ID == 255)){
		ID = DEFAULT_ID;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, MX28_REG_ID, ID, UNUSEDARGUMENT);
	hdynamixel->_ID = ID;
}

void setBaudRate(Dynamixel_HandleTypeDef* hdynamixel, long baud){
	/* Sets the baud rate of a particular MX28 motor. Default baud rate is 57,600 symbols/s.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  baud, the baud rate. Arguments in range [9600, 4500000] are valid
	 *
	 * Returns: none
	 */

	/* Set _baud equal to the hex code corresponding to baud. Default to 1 Mbps. */
	if(baud >= 9600 && baud <= 4500000){
		if(baud >= 9600 && baud < 57600){
			hdynamixel -> _BaudRate = 0x00;
		}
		else if(baud >= 57600 && baud < 115200){
			hdynamixel -> _BaudRate = 0x01;
		}
		else if(baud >= 115200 && baud < 1000000){
			hdynamixel -> _BaudRate = 0x02;
		}
		else if(baud >= 1000000 && baud < 2000000){
			hdynamixel -> _BaudRate = 0x03;
		}
		else if(baud >= 2000000 && baud < 3000000){
			hdynamixel -> _BaudRate = 0x04;
		}
		else if(baud >= 3000000 && baud < 4000000){
			hdynamixel -> _BaudRate = 0x05;
		}
		else if(baud >= 4000000 && baud < 4500000){
			hdynamixel -> _BaudRate = 0x06;
		}
		else{
			hdynamixel -> _BaudRate = 0x07;
		}
		
	}
	else{
		/* Default to 1 Mbps. */
		hdynamixel -> _BaudRate = DEFAULT_BAUD_RATE;
	}

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 8, MX28_REG_BAUD_RATE, hdynamixel -> _BaudRate, UNUSEDARGUMENT);
}

void setGoalPosition(Dynamixel_HandleTypeDef* hdynamixel, double goalAngle){
	/* Takes a double between 0 and 300, encodes this position in an
	 * upper and low hex byte pair (with a maximum of 4095 as defined in the MX28
	 * user manual), and sends this information (along with requisites) over UART.
	 *
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
	uint16_t normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 4095);

	uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal position
	uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal position

	/* Write data to motor. */
	Dynamixel_DataWriter(hdynamixel, 9, MX28_REG_GOAL_POSITION, lowByte, highByte);
}



void Dynamixel_TorqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
	/* Enables or disables torque for current motor.
	 *
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
	Dynamixel_DataWriter(hdynamixel, 8, MX28_REG_TORQUE_ENABLE, isEnabled, UNUSEDARGUMENT);
}

void torqueEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
	/* Enables or disables torque for a MX28.
	 *
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
	uint8_t args[2] = {MX28_REG_TORQUE_ENABLE, 1};
	MX28_DataWriter(hdynamixel, args, sizeof(args));
}

void Dynamixel_LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
	/* Updates the motor LED state.
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
	Dynamixel_DataWriter(hdynamixel, 8, MX28_REG_LED_ENABLE, isEnabled, UNUSEDARGUMENT);
}

void LEDEnable(Dynamixel_HandleTypeDef* hdynamixel, uint8_t isEnabled){
	/* Updates the motor LED state for MX28.
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
	uint8_t args[3] = {MX28_REG_LED_ENABLE & 0xFF, (MX28_REG_LED_ENABLE >> 8) & 0xFF, isEnabled};
	MX28_DataWriter(hdynamixel, args, sizeof(args));
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

	if(hdynamixel -> _motorType == AX12ATYPE || (hdynamixel -> _motorType == MX28TYPE && hdynamixel -> _protocolVersion == 1)){
		/* Do assignments and computations. */
		uint8_t ID = hdynamixel -> _ID;
		arrTransmit[ID][3] = arrSize - 4; // Length of message minus the obligatory bytes
		arrTransmit[ID][5] = writeAddr; // Write address for register
		arrTransmit[ID][6] = param1;

		/* Checksum. */
		arrTransmit[ID][7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit[ID], arrSize): param2;
		if(arrSize == 9){
			arrTransmit[ID][8] = Dynamixel_ComputeChecksum(arrTransmit[ID], arrSize);
		}

		/* Set data direction for transmit. */
		__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

		/* Transmit. */
		HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit[ID], arrSize, TRANSMIT_TIMEOUT);
	}
}

void MX28_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs){
	/* Datawriter for MX28 using protocol version 2.0.
	 *
	 * args is an array of arguments of the form
	 * 	{ADDR_L, ADDR_H, PARAM1_L, PARAM1_H, ...}
	 *
	 * and numArgs must be equal to sizeof(args)	 *
	 */

	/* Do assignments and computations. */
//	uint8_t arrTransmit[12 + numArgs];
	uint8_t arrTransmit[20];
	arrTransmit[0] = 0xFF;
	arrTransmit[1] = 0xFF;
	arrTransmit[2] = 0xFD;
	arrTransmit[3] = 0x00;
	arrTransmit[4] = hdynamixel -> _ID;
	arrTransmit[5] = (3 + numArgs) & 0xFF; //  3 for instruction and CRC, 2 for register address, numArgs for arguments
	arrTransmit[6] = ((3 + numArgs) >> 8) & 0xFF;
	arrTransmit[7] = INST_WRITE_DATA;

	for(uint16_t i = 0; i < numArgs; i++){
		arrTransmit[8 + i] = args[i];
	}

	uint16_t myCRC = update_crc(0, arrTransmit, 5 + arrTransmit[5] + arrTransmit[6]);

	arrTransmit[7 + numArgs + 1] = myCRC & 0x00FF;
	arrTransmit[7 + numArgs + 2] = (myCRC >> 8) & 0x00FF;

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 10 + numArgs, TRANSMIT_TIMEOUT);
}

void setProtocolTo1(Dynamixel_HandleTypeDef* hdynamixel){
	/* Sets motor protocol version to version 1.0 (relevant for MX28).
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	uint8_t args[3] = {MX28_REG_PROTOCOL_VERSION, 0, 1};
	MX28_DataWriter(hdynamixel, args, sizeof(args));

	hdynamixel -> _protocolVersion = 1;
}

uint8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel){
	/* Used only for returning a status packet or checking the existence of a motor
	 * with a specified ID. Does not command any operations.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: motor ID seen in status packet
	 */

	/* Define arrays for transmission and reception. */
	uint8_t arr[6];

	/* Do assignments and computations. */
	arr[0] = 0xff; // Obligatory bytes for starting communication
	arr[1] = 0xff; // Obligatory bytes for starting communication
	arr[2] = hdynamixel -> _ID; // Motor ID
	arr[3] = 2; // Length of message minus the obligatory bytes
	arr[4] = INST_PING; // PING instruction
	arr[5] = Dynamixel_ComputeChecksum(arr, 6);

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arr, 6, TRANSMIT_TIMEOUT);

	/* Set data direction for receive. */
	__DYNAMIXEL_RECEIVE(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Receive. */
	HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, 6, RECEIVE_TIMEOUT);
	return(arr[2]);
}

uint8_t Ping(Dynamixel_HandleTypeDef* hdynamixel){
	/* Used only for returning a status packet or checking the existence of a motor
	 * with a specified ID. Does not command any operations.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: error field in status packet
	 */

	/* Define arrays for transmission and reception. */
	uint8_t arr[14] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arr, 10, TRANSMIT_TIMEOUT);

	/* Set data direction for receive. */
	__DYNAMIXEL_RECEIVE(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Receive. */
	HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, 4, RECEIVE_TIMEOUT);
//	HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, 4, 100000);
	return(arr[8]);
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
		GPIO_TypeDef* DataDirPort, uint16_t DataDirPinNum, enum motorTypes_e motorType){
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
	 * 			  motorType, indicates whether motor is AX12A or MX28
	 *
	 * Returns: none
	 */

	/* Set fields in motor handle. */
	hdynamixel -> _motorType = motorType;
	hdynamixel -> _protocolVersion = 2;
	hdynamixel -> _ID = ID; 					// Motor ID (unique or global)
	hdynamixel -> _BaudRate = 57600; 			// Number of bytes per second transmitted by the UART
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
	 * first disconnecting them. Need to wait around 500 ms before motor becomes valid again.
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
}

void Reset(Dynamixel_HandleTypeDef* hdynamixel, uint8_t arg){
	/* Resets the control table values of the motor to the Factory Default Value settings.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  arg, 0xFF -> reset all control table entries, 0x01 -> resets all entries except ID,
	 * 			  	   0x02 -> resets all entries except ID and baud rate
	 *
	 * Returns: none
	 */

	uint8_t arrTransmit[11];
	arrTransmit[0] = 0xFF;
	arrTransmit[1] = 0xFF;
	arrTransmit[2] = 0xFD;
	arrTransmit[3] = 0x00;
	arrTransmit[4] = hdynamixel -> _ID;
	arrTransmit[5] = 0x04;
	arrTransmit[6] = 0;
	arrTransmit[7] = INST_RESET;
	arrTransmit[8] = arg;

	uint16_t myCRC = update_crc(0, arrTransmit, 5 + arrTransmit[5] + arrTransmit[6]);

	arrTransmit[7 + 1 + 1] = myCRC & 0x00FF;
	arrTransmit[7 + 1 + 2] = (myCRC >> 8) & 0x00FF;

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 10 + 1, TRANSMIT_TIMEOUT);

	hdynamixel -> _ID = DEFAULT_ID;
}

void Reboot(Dynamixel_HandleTypeDef* hdynamixel){
	/* Reboots a MX28.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: none
	 */

	uint8_t arrTransmit[10];
	arrTransmit[0] = 0xFF;
	arrTransmit[1] = 0xFF;
	arrTransmit[2] = 0xFD;
	arrTransmit[3] = 0x00;
	arrTransmit[4] = hdynamixel -> _ID;
	arrTransmit[5] = 0x04;
	arrTransmit[6] = 0;
	arrTransmit[7] = V2_INST_REBOOT;

	uint16_t myCRC = update_crc(0, arrTransmit, 5 + arrTransmit[5] + arrTransmit[6]);

	arrTransmit[7 + 1] = myCRC & 0x00FF;
	arrTransmit[7 + 2] = (myCRC >> 8) & 0x00FF;

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 10, TRANSMIT_TIMEOUT);

	hdynamixel -> _ID = DEFAULT_ID;
}

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

	// Set data direction for transmit
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	// Transmit
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit[ID], 8, TRANSMIT_TIMEOUT);

	// Set data direction for receive
	__DYNAMIXEL_RECEIVE(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Set the instruction back to INST_WRITE_DATA in the buffer because writing
	 * is expected to be more common than reading. */
	arrTransmit[ID][4] = INST_WRITE_DATA; // WRITE DATA instruction

	// Call appropriate UART receive function depending on if 1 or 2 bytes are to be read
	if(readLength == 1){
		HAL_UART_Receive(hdynamixel -> _UART_Handle, arrReceive[ID], 7, RECEIVE_TIMEOUT);
	}
	else{
		HAL_UART_Receive(hdynamixel -> _UART_Handle, arrReceive[ID], 8, RECEIVE_TIMEOUT);
	}

	if(readLength == 1){
		return (uint16_t)arrReceive[ID][5];
	}
	else{
		return (uint16_t)(arrReceive[ID][5] | (arrReceive[ID][6] << 8));
	}
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





/*******************************************************************************/
/*	Computation-based helper functions										   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*								 											   */
/*******************************************************************************/
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length){
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

uint16_t update_crc(uint16_t crc_accum, uint8_t* data_blk_ptr, uint16_t data_blk_size){
	uint16_t i, j;

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
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
