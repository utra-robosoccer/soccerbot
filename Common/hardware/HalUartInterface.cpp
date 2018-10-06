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




/********************************** Externs **********************************/
// extern declaration needed here for F767 build
// TODO: clean this up when refactoring Dynamixel
#ifdef STM32F767xx
extern HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
extern HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
#endif




namespace uart{
/***************************** HalUartInterface ******************************/
// Public
// ----------------------------------------------------------------------------
HalUartInterface::HalUartInterface(){

}

HalUartInterface::~HalUartInterface(){

}

HAL_StatusTypeDef HalUartInterface::transmitPoll(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrTransmit,
    size_t numBytes,
    uint32_t timeout
) const
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
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrReceive,
    size_t numBytes,
    uint32_t timeout
) const
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
HAL_StatusTypeDef HalUartInterface::transmitIT(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrTransmit,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Transmit_IT(
        uartHandlePtr,
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receiveIT(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrReceive,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Receive_IT(
        uartHandlePtr,
        arrReceive,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::transmitDMA(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrTransmit,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Transmit_DMA(
        uartHandlePtr,
        arrTransmit,
        numBytes
    );

    return status;
}

HAL_StatusTypeDef HalUartInterface::receiveDMA(
    const UART_HandleTypeDef* uartHandlePtr,
    uint8_t* arrReceive,
    size_t numBytes
) const
{
    HAL_StatusTypeDef status =  HAL_UART_Receive_DMA(
        uartHandlePtr,
        arrReceive,
        numBytes
    );

    return status;
}

void HalUartInterface::abortTransmit(
    const UART_HandleTypeDef* uartHandlePtr
) const
{
    HAL_UART_AbortTransmit(uartHandlePtr);
}

void HalUartInterface::abortReceive(
    const UART_HandleTypeDef* uartHandlePtr
) const
{
    HAL_UART_AbortReceive(uartHandlePtr);
}
#endif

}

/**
 * @}
 */
/* end - HalUartInterface */
