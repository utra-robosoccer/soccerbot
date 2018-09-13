/**
  *****************************************************************************
  * @file    UartInterface.cpp
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup UartInterface
  * @brief    TODO -- description of module
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "UartInterface.h"




namespace uart{
/***************************** UartInterface ******************************/
// Public
// ----------------------------------------------------------------------------
UartInterface::UartInterface(UART_HandleTypeDef* uartHandlePtr){
    this->uartHandlePtr = uartHandlePtr;
}

HAL_StatusTypeDef UartInterface::transmitPoll(uint8_t* arrTransmit, size_t numBytes, uint32_t timeout){
    HAL_StatusTypeDef status =  HAL_UART_Transmit(
        uartHandlePtr,
        arrTransmit,
        numBytes,
        timeout
    );

    return status;
}

HAL_StatusTypeDef UartInterface::receivePoll(uint8_t* arrReceive, size_t numBytes, uint32_t timeout){
    HAL_StatusTypeDef status =  HAL_UART_Receive(
        uartHandlePtr,
        arrReceive,
        numBytes,
        timeout
    );

    return status;
}

HAL_StatusTypeDef UartInterface::transmitIT(uint8_t* arrTransmit, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Transmit_IT(
        uartHandlePtr,
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef UartInterface::receiveIT(uint8_t* arrReceive, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Receive_IT(
        uartHandlePtr,
        arrReceive,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef UartInterface::transmitDMA(uint8_t* arrTransmit, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Transmit_DMA(
        uartHandlePtr,
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef UartInterface::receiveDMA(uint8_t* arrReceive, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Receive_DMA(
        uartHandlePtr,
        arrReceive,
        numBytes
    );

    return status;
}

void UartInterface::abortTransmit(){
    HAL_UART_AbortTransmit(uartHandlePtr);
}

void UartInterface::abortReceive(){
    HAL_UART_AbortReceive(uartHandlePtr);
}




/******************************* UartDriver *******************************/
// Public
// ----------------------------------------------------------------------------
UartDriver::UartDriver(UART_HandleTypeDef* uartHandlePtr)
  : hw_if(uartHandlePtr)
{
    this->io_type = IO_Type::POLL;
}

void UartDriver::setIOType(IO_Type io_type){
    this->io_type = io_type;
}

IO_Type UartDriver::getIOType(void) const{
    return this->io_type;
}

bool UartDriver::transmit(uint8_t* arrTransmit, size_t numBytes){
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    switch(io_type) {
#if defined(THREADED)
        case IO_Type::DMA:
            hw_if.transmitDMA(arrTransmit, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
        case IO_Type::IT:
            hw_if.transmitIT(arrTransmit, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_Type::POLL:
        default:
            retval = (hw_if.transmitPoll(arrTransmit, numBytes, TRANSFER_TIMEOUT) == HAL_OK);
            break;
    }

    if(retval != HAL_OK){
        hw_if.abortTransmit();
    }

    return retval;
}

bool UartDriver::receive(uint8_t* arrReceive, size_t numBytes){
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    switch(io_type) {
#if defined(THREADED)
        case IO_Type::DMA:
            hw_if.receiveDMA(arrReceive, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
        case IO_Type::IT:
            hw_if.receiveIT(arrReceive, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_Type::POLL:
        default:
            retval = (hw_if.receivePoll(arrReceive, numBytes, TRANSFER_TIMEOUT) == HAL_OK);
            break;
    }

    if(retval != HAL_OK){
        hw_if.abortReceive();
    }

    return retval;
}

}

/**
 * @}
 */
/* end - UartInterface */
