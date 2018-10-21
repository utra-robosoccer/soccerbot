/**
  *****************************************************************************
  * @file    UartDriver.h
  * @author  Tyler Gamvrelis
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
using os::OsInterface;
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
     * @param os_if Pointer to the object handling the calls to the OS
     * @param hw_if Pointer to the hardware-facing object handling the
     *        low-level UART routines
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    UartDriver(
        OsInterface* os_if,
        UartInterface* hw_if,
        UART_HandleTypeDef* uartHandlePtr
    );
#else
    /**
     * @brief Initializes the handle to the low-level hardware routines, and
     *        associates a particular UART module on the board with this driver
     * @param hw_if Pointer to the hardware-facing object handling the
     *        low-level UART routines
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    UartDriver(
        UartInterface* hw_if,
        UART_HandleTypeDef* uartHandlePtr
    );
#endif

    ~UartDriver() {}

    /**
     * @brief Configures the driver to use a particular IO type. This is used
     *        to change it between using blocking and asynchronous transfers
     * @param io_type The type of IO to be used by the driver
     */
    void setIOType(IO_Type io_type);

    /**
     * @brief  Returns the IO type currently being used by the driver
     * @return The IO type currently being used by the driver
     */
    IO_Type getIOType(void) const;

    /**
     * @brief  Set up the driver for the specified IO_Type, including initialization
     *         of internal data members.
     * @return true if setup succeeded, false if error
     */
    bool setup(void);

    /**
     * @brief  Instruct the driver to transmit data from the UART that was set
     *         by setUartInterface, and using the IO transfer mode set by
     *         setIOType.
     * @param  arrTransmit The byte array to be sent
     * @param  numBytes The number of bytes to be sent from the array
     * @return True if the transfer succeeded, otherwise false
     * @note   Some reasons why false may be returned include:
     *           -# Incomplete initialization of the driver
     *           -# UartInterface returns an error upon requesting a transfer
     *           -# OS block time is exceeded
     */
    bool transmit(uint8_t* arrTransmit, size_t numBytes) const;

    /**
     * @brief  Instruct the driver to receive data from the UART that was set
     *         by setUartInterface, and using the IO transfer mode set by
     *         setIOType.
     * @param  arrReceive The byte array which the received data is to be
     *         written into
     * @param  numBytes The number of bytes to be received
     * @return True if the transfer succeeded, otherwise false
     * @note   Some reasons why false may be returned include:
     *           -# Incomplete initialization of the driver
     *           -# UartInterface returns an error upon requesting a transfer
     *           -# OS block time is exceeded
     */
    bool receive(uint8_t* arrReceive, size_t numBytes) const;

private:
    /**
     * @brief IO Type used by the driver, i.e. whether the driver uses polled,
     *        interrupt-driven, or DMA-driven IO
     */
    IO_Type io_type = IO_Type::POLL;

    /** @brief Pointer to the object handling system calls to the OS */
    const OsInterface* os_if = nullptr;

    /**
     * @brief Pointer to the object handling direct calls to the UART hardware
     */
    const UartInterface* hw_if = nullptr;

    /**
     * @brief Address of the container for the UART module associated with this
     *        object
     */
    const UART_HandleTypeDef* uartHandlePtr = nullptr;

    /**
     * @brief true if the UartInterface has been set and its UART_HandleTypeDef
     *        pointer has been set, otherwise false
     */
    bool hw_is_initialized = false;
#if defined(THREADED)
    /** @brief Maximum time allowed for a polled IO transfer */
    static constexpr uint32_t POLLED_TRANSFER_TIMEOUT = pdMS_TO_TICKS(2);

    /**
     * @brief Maximum time allowed for a thread to block on an asynchronous
     *        transfer
     */
    static constexpr TickType_t MAX_BLOCK_TIME = pdMS_TO_TICKS(2);

    mutable osMutexId uartResourceMutex;
    mutable osStaticMutexDef_t uartResourceMutexControlBlock;

#endif
#if !defined(THREADED)
    /** @brief Maximum time allowed for a polled IO transfer */
    constexpr uint32_t POLLED_TRANSFER_TIMEOUT = 2;
#endif
};

} // end namespace uart




/**
 * @}
 */
/* end - Header */

#endif /* UART_DRIVER_H */
