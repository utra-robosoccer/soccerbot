/**
  *****************************************************************************
  * @file    UartInterface.cpp
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup UART
  * @brief    TODO -- description of module
  *
  * @defgroup UartInterface
  * @ingroup  UART
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

}

/**
 * @}
 */
/* end - UartInterface */
