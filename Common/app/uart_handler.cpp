/**
  ******************************************************************************
  * @file    uart_handler.c
  * @author  Tyler
  * @brief   This file implements a generic event processor for UART events,
  *          which occur when commands for the motors need to be distributed
  *
  * @defgroup uart_handler UART Handler
  * @brief    Event processor for motor commands
  * @{
  ******************************************************************************
  */




/********************************* Includes **********************************/
#include "uart_handler.h"




/********************************* Externs ***********************************/
/**
 * Sensor data queue. This module writes current positions of motors into this
 * queue
 */
extern osMessageQId TXQueueHandle;




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
    MotorData_t data;
    float pos = 0;
    DataToSend->pData = &data;

    bool status = false;
    switch(cmdPtr->type){
        case cmdReadPosition:
            status = cmdPtr->motorHandle->getPosition(pos);

            data.id = cmdPtr->motorHandle->id();
            data.payload = &pos;
            data.type = MotorData_t::T_FLOAT;

            xQueueSend(TXQueueHandle, DataToSend, 0);
            break;
        case cmdWritePosition:
            status = cmdPtr->motorHandle->setGoalPosition(cmdPtr->value);
            break;
        case cmdWriteTorque:
            status = cmdPtr->motorHandle->enableTorque(cmdPtr->value);
            break;
        default:
            break;
    }
}

/**
 * @}
 */
/* end UART_Handler */
