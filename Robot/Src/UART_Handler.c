/**
  ******************************************************************************
  * @file    UART_Handler.c
  * @author  Tyler
  * @brief   This file implements a generic event processor for UART events,
  *          which occur commands for the motors need to be distributed
  ******************************************************************************
  */

#include "UART_Handler.h"

extern osMessageQId UART_rxHandle;

/**
 * @brief  The UART event processor calls the low-level libraries to execute
 *         reads and writes for motors
 * @param  cmdPtr the handle for the command structure containing all relevant
 *         data fields
 * @param  DataToSend the handle for the data structure to copy into the sensor
 *         queue if a read is executed
 * @retval None
 */
void UART_ProcessEvent(UARTcmd_t* cmdPtr, TXData_t* DataToSend){
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
