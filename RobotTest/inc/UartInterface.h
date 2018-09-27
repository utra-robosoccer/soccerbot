/**
  *****************************************************************************
  * @file    UartInterface.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup   UartInterface
  * @addtogroup UART
  * @brief      Interface implemented by concrete & mock UART objects
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




/******************************* UartInterface *******************************/
namespace UART{
// Types & enums
// ----------------------------------------------------------------------------
/**
 * @brief Enumerates the IO types supported by a class which implements the
 *        UartInterface class
 */
enum class IO_Type{
    POLL /**< Polled IO          */
#ifdef THREADED
    ,
    IT, /**< Interrupt-driven IO */
    DMA /**< DMA-driven IO       */
#endif
};




// Classes and structs
// ----------------------------------------------------------------------------
// TODO: could make these functions return err_t
/**
 * @class Abstract class defining the interface that a hardware-facing UART
 *        object must have. This object takes care of directly interfacing
 *        with the hardware as it is instructed, without knowledge of the
 *        application logic
 */
class UartInterface{
public:
    virtual ~UartInterface() {}

    /**
     * @brief Associates the class with a particular UART module on the MCU
     * @param uartHandlePtr Pointer to a structure that contains
     *        the configuration information for the desired UART module
     */
    virtual void setUartPtr(UART_HandleTypeDef* uartHandlePtr) = 0;

    /**
     * @brief  Polled (blocking) transmission
     * @param  arrTransmit Pointer to the array of bytes to be sent
     * @param  numBytes The number of bytes to be sent
     * @param  timeout The number of ms to wait before the data transfer to
     *         finish
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef transmitPoll(
        uint8_t* arrTransmit,
        size_t numBytes,
        uint32_t timeout
    ) = 0;

    /**
     * @brief  Polled (blocking) reception
     * @param  arrReceive Pointer to the receive buffer
     * @param  numBytes The number of bytes to be received
     * @param  timeout The number of ms to wait before the data transfer to
     *         finish
     * @return 0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef receivePoll(
        uint8_t* arrReceive,
        size_t numBytes,
        uint32_t timeout
    ) = 0;

#ifdef THREADED
    /**
     * @brief   Interrupt-driven transmission. Can only be stopped upon the
     *          transfer finishing or by calling abortTransmit()
     * @param   arrTransmit Pointer to the array of bytes to be sent
     * @param   numBytes The number of bytes to be sent
     * @return  0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef transmitIT(uint8_t* arrTransmit, size_t numBytes) = 0;

    /**
     * @brief   Interrupt-driven reception. Can only be stopped upon the
     *          transfer finishing or by calling abortTransmit()
     * @param   arrTransmit Pointer to the receive buffer
     * @param   numBytes The number of bytes to be received
     * @return  0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef receiveIT(uint8_t* arrReceive, size_t numBytes) = 0;

    /**
     * @brief   DMA-driven transmission. Can only be stopped upon the transfer
     *          finishing or by calling abortTransmit()
     * @param   arrTransmit Pointer to the array of bytes to be sent
     * @param   numBytes The number of bytes to be sent
     * @return  0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef transmitDMA(uint8_t* arrTransmit, size_t numBytes) = 0;

    /**
     * @brief   DMA-driven reception. Can only be stopped upon the transfer
     *          finishing or by calling abortTransmit()
     * @param   arrTransmit Pointer to the receive buffer
     * @param   numBytes The number of bytes to be received
     * @return  0 if success, otherwise an error code from 1 to 3
     */
    virtual HAL_StatusTypeDef receiveDMA(uint8_t* arrReceive, size_t numBytes) = 0;

    /**
     * @brief Aborts an asynchronous transmission
     */
    virtual void abortTransmit() = 0;

    /**
     * @brief Aborts an asynchronous reception
     */
    virtual void abortReceive() = 0;
#endif
};

} // end namespace UART




/**
 * @}
 */
/* end - UartInterface */

#endif /* UART_INTERFACE_H */
