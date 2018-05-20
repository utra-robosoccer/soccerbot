/* This file is for IO-related functions and data structures. This is the
 * bottom "layer" of the Dynamixel library, as this file implements the instruction
 * set and provides wrappers for it that upper layers can use.
 *
 * Author: Tyler
 */

/********************************** Includes **********************************/
#include "DynamixelV1IO.h"

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

/************************ Private Function Prototypes **************************/
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length);

/******************************** Functions ************************************/
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
void Dynamixel_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs){
	/* Handles sending of data since this nearly identical for all setters.
	 * Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
	 *
	 * Arguments: hdynamixel, the motor handle
	 * 			  args is an array of arguments of the form
	 * 						{ADDR, PARAM1, ...}
	 *			  numArgs must be equal to sizeof(args), and must either be 2 or 3
	 *
	 * Returns: none
	 */

	/* Check validity so that we don't accidentally write something invalid */
	if(numArgs <= 3){
		/* Do assignments and computations. */
		uint8_t ID = hdynamixel -> _ID;
		arrTransmit[ID][3] = 2 + numArgs; // Length of message following length argument (arguments & checksum)

		for(uint8_t i = 0; i < numArgs; i ++){
			arrTransmit[ID][5 + i] = args[i];
		}

		/* Checksum. */
		arrTransmit[ID][4 + numArgs + 1] = Dynamixel_ComputeChecksum(arrTransmit[ID], 4 + numArgs + 2);

		/* Set data direction for transmit. */
		__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

		/* Transmit. */
		HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit[ID], 4 + numArgs + 2, TRANSMIT_TIMEOUT);
	}
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
	arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_REG_WRITE; // REG WRITE instruction
	arrTransmit[5] = writeAddr;
	arrTransmit[7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit, arrSize): param2;
	if(arrSize == 9){
		arrTransmit[8] = Dynamixel_ComputeChecksum(arrTransmit, arrSize);
	}

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, arrSize, TRANSMIT_TIMEOUT);
}

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
	arrTransmit[0] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[1] = 0xFF; // Obligatory bytes for starting communication
	arrTransmit[2] = hdynamixel -> _ID; // Motor ID
	arrTransmit[3] = 2; // Length of message minus the obligatory bytes
	arrTransmit[4] = INST_ACTION; // ACTION instruction
	arrTransmit[5] = Dynamixel_ComputeChecksum(arrTransmit, 6);

	/* Set data direction. */
	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);

	/* Transmit. */
	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 6, TRANSMIT_TIMEOUT);
}

int8_t Dynamixel_Ping(Dynamixel_HandleTypeDef* hdynamixel){
	/* Used only for returning a status packet or checking the existence of a motor
	 * with a specified ID. Does not command any operations.
	 *
	 * Arguments: hdynamixel, the motor handle
	 *
	 * Returns: motor ID seen in status packet if received a valid status packet,
	 * 				otherwise -1
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
	if(HAL_UART_Receive(hdynamixel -> _UART_Handle, arr, 6, RECEIVE_TIMEOUT)){
		return(arr[2]);
	}
	else{
		return -1;
	}
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
