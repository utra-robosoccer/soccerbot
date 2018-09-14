/**
  *****************************************************************************
  * @file    UartDriver.h
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup Header
  * @ingroup  UART
  * @{
  *****************************************************************************
  */




#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__




/********************************* Includes **********************************/
#include "UartInterface.h"




/******************************* uart_interface ******************************/
namespace uart{
// Classes and structs
// ----------------------------------------------------------------------------
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

#endif /* __UART_DRIVER_H__ */
