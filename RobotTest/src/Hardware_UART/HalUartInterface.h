/**
  *****************************************************************************
  * @file    HalUartInterface.h
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup Header
  * @ingroup  HalUartInterface
  * @{
  *****************************************************************************
  */




#ifndef HAL_UART_INTERFACE_H
#define HAL_UART_INTERFACE_H




/********************************** Macros ***********************************/
#define THREADED 1




/********************************* Includes **********************************/
#include <cstdint>

#define STM32F446xx
#include <stm32f446xx.h>
#include "usart.h"




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
class HalUartInterface{
public:
    HalUartInterface();
    ~HalUartInterface() {}

    void setUartPtr(UART_HandleTypeDef* uartHandlePtr);

    HAL_StatusTypeDef transmitPoll(
        uint8_t* arrTransmit,
        size_t numBytes,
        uint32_t timeout
    );
    HAL_StatusTypeDef receivePoll(
        uint8_t* arrReceive,
        size_t numBytes,
        uint32_t timeout
    );
    HAL_StatusTypeDef transmitIT(uint8_t* arrTransmit, size_t numBytes);
    HAL_StatusTypeDef receiveIT(uint8_t* arrReceive, size_t numBytes);
    HAL_StatusTypeDef transmitDMA(uint8_t* arrTransmit, size_t numBytes);
    HAL_StatusTypeDef receiveDMA(uint8_t* arrReceive, size_t numBytes);

    void abortTransmit();
    void abortReceive();

private:
    UART_HandleTypeDef* uartHandlePtr = nullptr;
};

} // end namespace uart




/**
 * @}
 */
/* end - Header */

#endif /* HAL_UART_INTERFACE_H */
