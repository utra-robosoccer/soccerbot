/**
  *****************************************************************************
  * @file    HalUartInterface.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup HalUartInterface
  * @brief    Implements UartInterface using HAL functions
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "HalUartInterface.h"




namespace UART{
/***************************** HalUartInterface ******************************/
// Public
// ----------------------------------------------------------------------------
HalUartInterface::HalUartInterface(){

}

HalUartInterface::~HalUartInterface(){

}

void HalUartInterface::setUartPtr(UART_HandleTypeDef* uartHandlePtr){
    this->uartHandlePtr = uartHandlePtr;
}

HAL_StatusTypeDef HalUartInterface::transmitPoll(
    uint8_t* arrTransmit,
    size_t numBytes,
    uint32_t timeout
)
{
    HAL_StatusTypeDef status =  HAL_UART_Transmit(
        uartHandlePtr,
        arrTransmit,
        numBytes,
        timeout
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receivePoll(
    uint8_t* arrReceive,
    size_t numBytes,
    uint32_t timeout
)
{
    HAL_StatusTypeDef status =  HAL_UART_Receive(
        uartHandlePtr,
        arrReceive,
        numBytes,
        timeout
    );

    return status;
}

#if defined(THREADED)
HAL_StatusTypeDef HalUartInterface::transmitIT(uint8_t* arrTransmit, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Transmit_IT(
        uartHandlePtr,
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receiveIT(uint8_t* arrReceive, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Receive_IT(
        uartHandlePtr,
        arrReceive,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::transmitDMA(uint8_t* arrTransmit, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Transmit_DMA(
        uartHandlePtr,
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receiveDMA(uint8_t* arrReceive, size_t numBytes){
    HAL_StatusTypeDef status =  HAL_UART_Receive_DMA(
        uartHandlePtr,
        arrReceive,
        numBytes
    );

    return status;
}

// extern declaration needed here for F767 build
// TODO: clean this up when refactoring Dynamixel
extern HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
extern HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);

void HalUartInterface::abortTransmit(){
    HAL_UART_AbortTransmit(uartHandlePtr);
}

void HalUartInterface::abortReceive(){
    HAL_UART_AbortReceive(uartHandlePtr);
}
#endif

}

/**
 * @}
 */
/* end - HalUartInterface */
