/**
  *****************************************************************************
  * @file    UartInterface.h
  * @author  Tyler Gamvrelis
  * @brief   UART
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
// TODO: separate these classes into different files. Then only expose the virtual
// UartInterface to the test framework

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

class UartDriver{
public:
    UartDriver(UART_HandleTypeDef* uartHandlePtr);
    ~UartDriver() {}

    void setIOType(IO_Type io_type);
    IO_Type getIOType(void) const;

    // TODO: if we have an err_t type, we could use it to indicate both HAL and
    // FreeRTOS issues, so here that would be good as the ideal return type would
    // be able to indicate both an issue with the hardware AND with the OS (e.g. a
    // timeout)
    bool transmit(uint8_t* arrTransmit, size_t numBytes);
    bool receive(uint8_t* arrReceive, size_t numBytes);

private:
    IO_Type io_type;
    UartInterface hw_if;
#ifdef THREADED
    static constexpr uint32_t TRANSFER_TIMEOUT = pdMS_TO_TICKS(2);
    static constexpr TickType_t MAX_BLOCK_TIME = pdMS_TO_TICKS(2);
#endif
#ifndef THREADED
    constexpr uint32_t TRANSFER_TIMEOUT = 2;
#endif
};

} // end namespace uart_interface




/**
 * @}
 */
/* end - Header */

#endif /* __UART_INTERFACE_H__ */
