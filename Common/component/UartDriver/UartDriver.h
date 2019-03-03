/**
  *****************************************************************************
  * @file    UartDriver.h
  * @author  Tyler Gamvrelis
  * @author  Robert Fairley
  *
  * @defgroup Header
  * @ingroup UartDriver
  * @{
  *****************************************************************************
  */




#ifndef UART_DRIVER_H
#define UART_DRIVER_H




/********************************* Includes **********************************/
#include "UartInterface.h"

#if defined(THREADED)
#include "cmsis_os.h"
#include "OsInterface.h"
using cmsis::OsInterface;
using hal::IO_Type;
using hal::UartInterface;
#endif




/******************************** UartDriver *********************************/
namespace uart{

// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class UartDriver Manages the usage of a particular UART, taking care of
 *        hardware-level calls, OS-level calls, and passing up statuses. OS
 *        support is selected at compile time based on whether the THREADED
 *        macro is defined
 */
class UartDriver{
public:
    UartDriver();

#if defined(THREADED)
    /**
     * @brief Initializes the handle to the low-level hardware routines,
     *        associates a particular UART module on the board with this
     *        driver, and initializes the handle to the OS for system calls
     * @param m_os_if Pointer to the object handling the calls to the OS
     * @param m_hw_if Pointer to the hardware-facing object handling the
     *        low-level UART routines
     * @param m_uart_handle_ptr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    UartDriver(
        OsInterface* m_os_if,
        UartInterface* m_hw_if,
        UART_HandleTypeDef* m_uart_handle_ptr
    );
#else
    /**
     * @brief Initializes the handle to the low-level hardware routines, and
     *        associates a particular UART module on the board with this driver
     * @param m_hw_if Pointer to the hardware-facing object handling the
     *        low-level UART routines
     * @param m_uart_handle_ptr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    UartDriver(
        UartInterface* m_hw_if,
        UART_HandleTypeDef* m_uart_handle_ptr
    );
#endif

    ~UartDriver() {}

    /**
     * @brief Sets data transfer timeout for this driver instance
     * @param timeout The maximum time the caller will block on a data transfer
     */
    void setMaxBlockTime(uint32_t timeout);

    /**
     * @brief Configures the driver to use a particular IO type. This is used
     *        to change it between using blocking and asynchronous transfers
     * @param io_type The type of IO to be used by the driver
     */
    void setIOType(IO_Type io_type);

    /**
     * @brief Returns the IO type currently being used by the driver
     * @return The IO type currently being used by the driver
     */
    IO_Type getIOType(void) const;

    /**
     * @brief  Instruct the driver to transmit data from the UART that was set
     *         by setUartInterface, and using the IO transfer mode set by
     *         setIOType.
     * @param  arr_transmit The byte array to be sent
     * @param  num_bytes The number of bytes to be sent from the array
     * @return True if the transfer succeeded, otherwise false
     * @note   Some reasons why false may be returned include:
     *           -# Incomplete initialization of the driver
     *           -# UartInterface returns an error upon requesting a transfer
     *           -# OS block time is exceeded
     */
    bool transmit(uint8_t* arr_transmit, size_t num_bytes) const;

    /**
     * @brief  Instruct the driver to receive data from the UART that was set
     *         by setUartInterface, and using the IO transfer mode set by
     *         setIOType.
     * @param  arr_receive The byte array which the received data is to be
     *         written into
     * @param  num_bytes The number of bytes to be received
     * @return True if the transfer succeeded, otherwise false
     * @note   Some reasons why false may be returned include:
     *           -# Incomplete initialization of the driver
     *           -# UartInterface returns an error upon requesting a transfer
     *           -# OS block time is exceeded
     */
    bool receive(uint8_t* arr_receive, size_t num_bytes) const;

private:
    /**
     * @brief IO Type used by the driver, i.e. whether the driver uses polled,
     *        interrupt-driven, or DMA-driven IO
     */
    IO_Type m_io_type = IO_Type::POLL;

#if defined(THREADED)
    /** @brief Pointer to the object handling system calls to the OS */
    const OsInterface* m_os_if = nullptr;
#endif

    /**
     * @brief Pointer to the object handling direct calls to the UART hardware
     */
    const UartInterface* m_hw_if = nullptr;

    /**
     * @brief Address of the container for the UART module associated with this
     *        object
     */
    const UART_HandleTypeDef* m_uart_handle_ptr = nullptr;

    /**
     * @brief true if the UartInterface has been set and its UART_HandleTypeDef
     *        pointer has been set, otherwise false
     */
    bool m_hw_is_initialized = false;

    /** @brief Maximum permitted time for blocking on a data transfer */
    TickType_t m_max_block_time;
};

} // end namespace uart




/**
 * @}
 */
/* end - Header */

#endif /* UART_DRIVER_H */
