/**
  *****************************************************************************
  * @file    UartInterface.h
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup Header
  * @ingroup  UartInterface
  * @{
  *****************************************************************************
  */




#ifndef __UART_INTERFACE_H__
#define __UART_INTERFACE_H__




/********************************** Macros ***********************************/
#define THREADED 1




/********************************* Includes **********************************/
#include <cstdint>

#define STM32F446xx
#include <stm32f446xx.h>

#ifdef THREADED
#include <cmsis_os.h>
#include "Notification.h"
#endif

#include <usart.h>




/******************************* uart_interface ******************************/
namespace uart{
// Types & enums
// ----------------------------------------------------------------------------
enum class IO_Type{
    POLL,
    IT,
    DMA
};




// Classes and structs
// ----------------------------------------------------------------------------
class UartInterface{
public:
    UartInterface(UART_HandleTypeDef* uartHandlePtr);
    virtual ~UartInterface() {}

    virtual HAL_StatusTypeDef transmitPoll(uint8_t* arrTransmit, size_t numBytes, uint32_t timeout);
    virtual HAL_StatusTypeDef receivePoll(uint8_t* arrReceive, size_t numBytes, uint32_t timeout);
    virtual HAL_StatusTypeDef transmitIT(uint8_t* arrTransmit, size_t numBytes);
    virtual HAL_StatusTypeDef receiveIT(uint8_t* arrReceive, size_t numBytes);
    virtual HAL_StatusTypeDef transmitDMA(uint8_t* arrTransmit, size_t numBytes);
    virtual HAL_StatusTypeDef receiveDMA(uint8_t* arrReceive, size_t numBytes);

    virtual void abortTransmit();
    virtual void abortReceive();

private:
    UART_HandleTypeDef* uartHandlePtr;
};

} // end namespace uart_interface




/**
 * @}
 */
/* end - Header */

#endif /* __UART_INTERFACE_H__ */
