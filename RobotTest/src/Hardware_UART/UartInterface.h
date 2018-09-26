/**
  *****************************************************************************
  * @file    UartInterface.h
  * @author  Tyler Gamvrelis
  * @brief   Interface implemented by concrete & mock UART objects
  *
  * @defgroup UartInterface
  * @{
  *****************************************************************************
  */




#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H




/********************************* Includes **********************************/
#include <cstdint>

#define STM32F446xx
#include <stm32f446xx.h>
#include "usart.h"




/******************************* uart_interface ******************************/
namespace uart{
// Classes and structs
// ----------------------------------------------------------------------------
// TODO: could make these functions return err_t
class UartInterface{
public:
    virtual ~UartInterface() {}

    virtual void setUartPtr(UART_HandleTypeDef* uartHandlePtr) = 0;

    virtual HAL_StatusTypeDef transmitPoll(
        uint8_t* arrTransmit,
        size_t numBytes,
        uint32_t timeout
    ) = 0;

    virtual HAL_StatusTypeDef receivePoll(
        uint8_t* arrReceive,
        size_t numBytes,
        uint32_t timeout
    ) = 0;

#ifdef THREADED
    virtual HAL_StatusTypeDef transmitIT(uint8_t* arrTransmit, size_t numBytes) = 0;
    virtual HAL_StatusTypeDef receiveIT(uint8_t* arrReceive, size_t numBytes) = 0;
    virtual HAL_StatusTypeDef transmitDMA(uint8_t* arrTransmit, size_t numBytes) = 0;
    virtual HAL_StatusTypeDef receiveDMA(uint8_t* arrReceive, size_t numBytes) = 0;

    virtual void abortTransmit() = 0;
    virtual void abortReceive() = 0;
#endif
};

} // end namespace uart




/**
 * @}
 */
/* end - UartInterface */

#endif /* UART_INTERFACE_H */
