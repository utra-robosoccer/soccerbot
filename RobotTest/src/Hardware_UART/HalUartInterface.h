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




/********************************* Includes **********************************/
#include <cstdint>

#define STM32F446xx
#include <stm32f446xx.h>
#include "usart.h"
#include "UartInterface.h"




/******************************* uart_interface ******************************/
namespace uart{
// Types & enums
// ----------------------------------------------------------------------------
enum class IO_Type{
    POLL
#ifdef THREADED
    ,
    IT,
    DMA
#endif
};




// Classes and structs
// ----------------------------------------------------------------------------
class HalUartInterface : UartInterface{
public:
    HalUartInterface();
    ~HalUartInterface() {}

    void setUartPtr(UART_HandleTypeDef* uartHandlePtr) override final;

    HAL_StatusTypeDef transmitPoll(
        uint8_t* arrTransmit,
        size_t numBytes,
        uint32_t timeout
    ) override final;

    HAL_StatusTypeDef receivePoll(
        uint8_t* arrReceive,
        size_t numBytes,
        uint32_t timeout
    ) override final;

#ifdef THREADED
    HAL_StatusTypeDef transmitIT(uint8_t* arrTransmit, size_t numBytes) override final;
    HAL_StatusTypeDef receiveIT(uint8_t* arrReceive, size_t numBytes) override final;
    HAL_StatusTypeDef transmitDMA(uint8_t* arrTransmit, size_t numBytes) override final;
    HAL_StatusTypeDef receiveDMA(uint8_t* arrReceive, size_t numBytes) override final;

    void abortTransmit() override final;
    void abortReceive() override final;
#endif

private:
    UART_HandleTypeDef* uartHandlePtr = nullptr;
};

} // end namespace uart




/**
 * @}
 */
/* end - Header */

#endif /* HAL_UART_INTERFACE_H */
