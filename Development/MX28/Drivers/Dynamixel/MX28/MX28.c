/********************************* Includes ************************************/
#include "MX28.h"

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
// TODO (all)
void MX28_SetMultiTurnOffset(Dynamixel_HandleTypeDef* hdynamixel, int16_t offset){

}

void MX28_SetResolutionDivider(Dynamixel_HandleTypeDef* hdynamixel, uint8_t divider){

}

void MX28_SetDGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t DGain){

}

void MX28_SetIGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t IGain){

}

void MX28_SetPGain(Dynamixel_HandleTypeDef* hdynamixel, uint8_t PGain){

}

void MX28_SetGoalAcceleration(Dynamixel_HandleTypeDef* hdynamixel, double goalAcceleration){

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
void MX28_EnterMultiTurnMode(Dynamixel_HandleTypeDef* hdynamixel){

}

//void MX28_DataWriter(Dynamixel_HandleTypeDef* hdynamixel, uint8_t* args, uint8_t numArgs){
//	/* Datawriter for MX28 using protocol version 2.0.
//	 *
//	 * args is an array of arguments of the form
//	 * 	{ADDR_L, ADDR_H, PARAM1_L, PARAM1_H, ...}
//	 *
//	 * and numArgs must be equal to sizeof(args)	 *
//	 */
//
//	/* Do assignments and computations. */
////	uint8_t arrTransmit[12 + numArgs];
//	uint8_t arrTransmit[20];
//	arrTransmit[0] = 0xFF;
//	arrTransmit[1] = 0xFF;
//	arrTransmit[2] = 0xFD;
//	arrTransmit[3] = 0x00;
//	arrTransmit[4] = hdynamixel -> _ID;
//	arrTransmit[5] = (3 + numArgs) & 0xFF; //  3 for instruction and CRC, 2 for register address, numArgs for arguments
//	arrTransmit[6] = ((3 + numArgs) >> 8) & 0xFF;
//	arrTransmit[7] = INST_WRITE_DATA;
//
//	for(uint16_t i = 0; i < numArgs; i++){
//		arrTransmit[8 + i] = args[i];
//	}
//
//	uint16_t myCRC = update_crc(0, arrTransmit, 5 + arrTransmit[5] + arrTransmit[6]);
//
//	arrTransmit[7 + numArgs + 1] = myCRC & 0x00FF;
//	arrTransmit[7 + numArgs + 2] = (myCRC >> 8) & 0x00FF;
//
//	/* Set data direction for transmit. */
//	__DYNAMIXEL_TRANSMIT(hdynamixel -> _dataDirPort, hdynamixel -> _dataDirPinNum);
//
//	/* Transmit. */
//	HAL_UART_Transmit(hdynamixel -> _UART_Handle, arrTransmit, 10 + numArgs, TRANSMIT_TIMEOUT);
//}

