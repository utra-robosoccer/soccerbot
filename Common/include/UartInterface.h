/**
  *****************************************************************************
  * @file    UartInterface.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup UartInterface
  * @ingroup UART
  * @brief Interface implemented by concrete & mock UART objects
  * @{
  *****************************************************************************
  */




#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H




/********************************* Includes **********************************/
#include <cstdint>
#include "SystemConf.h"
#include "usart.h"




/******************************* UartInterface *******************************/
namespace uart{
// Types & enums
// ----------------------------------------------------------------------------
/**
 * @brief Enumerates the IO types supported by a class which implements the
 *        UartInterface class
 */
enum class IO_Type{
    POLL /**< Polled IO          */
#if defined(THREADED)
    ,
    IT, /**< Interrupt-driven IO */
    DMA /**< DMA-driven IO       */
#endif
};




// Classes and structs
// ----------------------------------------------------------------------------
// TODO: could make these functions return err_t
/**
 * @class UartInterface Abstract class defining the interface that a
 *        hardware-facing UART object must have. This object takes care of
 *        directly interfacing with the hardware as it is instructed, without
 *        knowledge of the application logic
 */
class UartInterface{
public:
    virtual ~UartInterface() {}

    /**
     * @brief  Polled (blocking) transmission
     * @param  uartHandlePtr Pointer to a structure that contains
     *         the configuration information for the desired UART module
     * @param  arrTransmit Pointer to the array of bytes to be sent
     * @param  numBytes The number of bytes to be sent
     * @param  timeout The number of ms to wait before the data transfer to
     *         finish
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef transmitPoll(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrTransmit,
        size_t numBytes,
        uint32_t timeout
    ) const = 0;

    /**
     * @brief  Polled (blocking) reception
     * @param  uartHandlePtr Pointer to a structure that contains
     *         the configuration information for the desired UART module
     * @param  arrReceive Pointer to the receive buffer
     * @param  numBytes The number of bytes to be received
     * @param  timeout The number of ms to wait before the data transfer to
     *         finish
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef receivePoll(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrReceive,
        size_t numBytes,
        uint32_t timeout
    ) const = 0;

#if defined(THREADED)
    /**
     * @brief  Interrupt-driven transmission. Can only be stopped upon the
     *         transfer finishing or by calling abortTransmit()
     * @param  uartHandlePtr Pointer to a structure that contains
     *         the configuration information for the desired UART module
     * @param  arrTransmit Pointer to the array of bytes to be sent
     * @param  numBytes The number of bytes to be sent
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef transmitIT(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrTransmit,
        size_t numBytes
    ) const = 0;

    /**
     * @brief  Interrupt-driven reception. Can only be stopped upon the
     *         transfer finishing or by calling abortTransmit()
     * @param  uartHandlePtr Pointer to a structure that contains
     *         the configuration information for the desired UART module
     * @param  arrTransmit Pointer to the receive buffer
     * @param  numBytes The number of bytes to be received
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef receiveIT(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrReceive,
        size_t numBytes
    ) const = 0;

    /**
     * @brief   DMA-driven transmission. Can only be stopped upon the transfer
     *          finishing or by calling abortTransmit()
     * @param   arrTransmit Pointer to the array of bytes to be sent
     * @param   numBytes The number of bytes to be sent
     * @return  0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef transmitDMA(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrTransmit,
        size_t numBytes
    ) const = 0;

    /**
     * @brief  DMA-driven reception. Can only be stopped upon the transfer
     *         finishing or by calling abortTransmit()
     * @param  uartHandlePtr Pointer to a structure that contains
     *         the configuration information for the desired UART module
     * @param  arrTransmit Pointer to the receive buffer
     * @param  numBytes The number of bytes to be received
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef receiveDMA(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrReceive,
        size_t numBytes
    ) const = 0;

    /**
     * @brief Aborts an asynchronous transmission
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    virtual void abortTransmit(
        const UART_HandleTypeDef* uartHandlePtr
    ) const = 0;

    /**
     * @brief Aborts an asynchronous reception
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    virtual void abortReceive(
        const UART_HandleTypeDef* uartHandlePtr
    ) const = 0;
#endif
};

} // end namespace uart




/**
 * @}
 */
/* end - UartInterface */

#endif /* UART_INTERFACE_H */
