/**
  ******************************************************************************
  * @file    UART_Handler.c
  * @author  Tyler
  * @brief   This file implements a generic event processor for UART events,
  *          which occur when commands for the motors need to be distributed
  *
  * @defgroup UART_Handler UART Handler
  * @brief    Event processor for motor commands
  * @{
  ******************************************************************************
  */




/********************************* Includes **********************************/
#include "UART_Handler.h"




/********************************* Externs ***********************************/
/**
 * Sensor data queue. This module writes current positions of motors into this
 * queue
 */
extern osMessageQId UART_rxHandle;




/******************************** Functions **********************************/
/**
 * @brief  The UART event processor calls the low-level libraries to execute
 *         reads and writes for motors
 * @param  cmdPtr the handle for the command structure containing all relevant
 *         data fields
 * @param  DataToSend the handle for the data structure to copy into the sensor
 *         queue if a read is executed
 * @return None
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

/**
 * @}
 */
/* end UART_Handler */
