/**
  ******************************************************************************
  * @file
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
#include <math.h>




/********************************* Externs ***********************************/
/**
 * Sensor data queue. This module writes current positions of motors into this
 * queue
 */
extern osMessageQId BufferWriteQueueHandle;




/******************************** Functions **********************************/
/**
 * @brief  The UART event processor calls the low-level libraries to execute
 *         reads and writes for motors
 * @param  p_cmd the handle for the command structure containing all relevant
 *         data fields
 * @return None
 */
void UART_ProcessEvent(UartCmd_t* p_cmd){
    float pos;
    bool success;
    MotorData_t data;
    TxData_t data_to_send;
    data_to_send.eDataType = eMotorData;
    data_to_send.pData = &data;

    switch(p_cmd->type){
        case cmdReadPosition:
            success = p_cmd->motorHandle->getPosition(pos);

            // issue #130: send NAN upon read failure
            data.payload = success ? pos : NAN;
            data.id = p_cmd->motorHandle->id();
            data.type = MotorData_t::T_FLOAT;

            xQueueSend(BufferWriteQueueHandle, &data_to_send, 0);
            break;
        case cmdWritePosition:
            p_cmd->motorHandle->setGoalPosition(p_cmd->value);
            break;
        case cmdWriteTorque:
            p_cmd->motorHandle->enableTorque(p_cmd->value);
            break;
        default:
            break;
    }
}

/**
 * @}
 */
/* end UART_Handler */
