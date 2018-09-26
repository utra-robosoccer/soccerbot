/**
  *****************************************************************************
  * @file    UartDriver.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup Header
  * @ingroup  UartDriver
  * @{
  *****************************************************************************
  */




#ifndef UART_DRIVER_H
#define UART_DRIVER_H




/********************************** Macros ***********************************/
// Flag for whether or not to use the non-blocking driver features supported
// by the OS (FreeRTOS). This is resolved at compile time
#define THREADED 1




/********************************* Includes **********************************/
#include "UartInterface.h"

#ifdef THREADED
#include "cmsis_os.h"
#include "Notification.h"
#include "FreeRTOSInterface.h"
using namespace FreeRTOS_Interface;
#endif




/******************************** UartDriver *********************************/
namespace UART{
// Classes and structs
// ----------------------------------------------------------------------------
class UartDriver{
public:
    UartDriver();
    ~UartDriver() {}

    void setUartInterface(UartInterface* hw_if, UART_HandleTypeDef* uartHandlePtr);
#ifdef THREADED
    void setOSInterface(FreeRTOSInterface* os_if);
#endif
    void setIOType(IO_Type io_type);
    IO_Type getIOType(void) const;

    // TODO: if we have an err_t type, we could use it to indicate both HAL and
    // FreeRTOS issues, so here that would be good as the ideal return type would
    // be able to indicate both an issue with the hardware AND with the OS (e.g. a
    // timeout). Also, I think these functions can be const-qualified
    bool transmit(uint8_t* arrTransmit, size_t numBytes);
    bool receive(uint8_t* arrReceive, size_t numBytes);

private:
    /**
     * @brief IO Type used by the driver, i.e. whether the driver uses polled,
     *        interrupt-driven, or DMA-driven IO
     */
    IO_Type io_type = IO_Type::POLL;

    /**
     * @brief Pointer to the object handling direct calls to the UART hardware
     */
    UartInterface* hw_if = nullptr;

    /**
     * @brief true if the UartInterface has been set and its UART_HandleTypeDef
     *        pointer has been set, otherwise false
     */
    bool hw_is_initialized = false;
#ifdef THREADED
    /** @brief Maximum time allowed for a polled IO transfer */
    static constexpr uint32_t POLLED_TRANSFER_TIMEOUT = pdMS_TO_TICKS(2);

    /**
     * @brief Maximum time allowed for a thread to block on an asynchronous
     *        transfer
     */
    static constexpr TickType_t MAX_BLOCK_TIME = pdMS_TO_TICKS(2);

    /** @brief Pointer to the object handling system calls to the OS */
    FreeRTOSInterface* os_if = nullptr;
#endif
#ifndef THREADED
    /** @brief Maximum time allowed for a polled IO transfer */
    constexpr uint32_t POLLED_TRANSFER_TIMEOUT = 2;
#endif
};

} // end namespace UART




/**
 * @}
 */
/* end - Header */

#endif /* UART_DRIVER_H */
