/*
 * AX12A.cpp
 *
 *  Created on: Mar 16, 2018
 *      Author: Admin
 */

#include "AX12A.h"

AX12A::AX12A() {
	// TODO Auto-generated constructor stub
	this -> isJointMode = 1;
}

AX12A::~AX12A() {
	// TODO Auto-generated destructor stub
}

void AX12A::dataWriter(uint8_t arrSize, \
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

	/* Do assignments and computations. */
	uint8_t ID = this -> id;
	arrTransmit[ID][3] = arrSize - 4; // Length of message minus the obligatory bytes
	arrTransmit[ID][5] = writeAddr; // Write address for register
	arrTransmit[ID][6] = param1;

	/* Checksum. */
	arrTransmit[ID][7] = (arrSize == 8) ? Dynamixel_ComputeChecksum(arrTransmit[ID], arrSize): param2;
	if(arrSize == 9){
		arrTransmit[ID][8] = Dynamixel_ComputeChecksum(arrTransmit[ID], arrSize);
	}

	/* Set data direction for transmit. */
	__DYNAMIXEL_TRANSMIT(this -> dataDirPort, this -> dataDirPinNum);

	/* In the future, a lock should be placed here, and it should be unlocked after the status
	 * packet is received. This requires using interrupts for TX/RX and it cannot be implemented
	 * right now because the blocking transmit and receive use timer interrupts for their timeout
	 */

	/* Transmit. */
	#if TRANSMIT_IT
		HAL_UART_Transmit_IT(this -> uartHandle, arrTransmit[ID], arrSize);
	#else
		HAL_UART_Transmit(this -> uartHandle, arrTransmit[ID], arrSize, TRANSMIT_TIMEOUT);
	#endif
}
