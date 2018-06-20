/*
 * UART_Handler.c
 *
 *  Created on: Jun 19, 2018
 *      Author: Tyler
 */

#include "UART_Handler.h"

extern osMessageQId UART_rxHandle;

void UART_ProcessEvent(UARTcmd_t* cmdPtr, TXData_t* DataToSend){
	/* The UART event processor calls the low-level libraries
	 * to execute reads and writes.
	 *
	 * Arguments: cmdPtr, the handle for the command structure containing all relevant fields
	 *
	 * Returns: none
	 */

	switch(cmdPtr->type){
		  case cmdReadPosition:
			  Dynamixel_GetPosition(cmdPtr->motorHandle);
			  DataToSend->pData = cmdPtr->motorHandle;
			  xQueueSend(UART_rxHandle, DataToSend, 0);
			  break;
		  case cmdWritePosition:
			  Dynamixel_SetGoalPosition(cmdPtr->motorHandle, cmdPtr->value);
			  break;
		  case cmdWriteTorque:
			  Dynamixel_TorqueEnable(cmdPtr->motorHandle, cmdPtr->value);
			  break;
		  default:
			  break;
	  }
}
