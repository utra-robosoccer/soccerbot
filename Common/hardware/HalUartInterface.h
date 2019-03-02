/**
  *****************************************************************************
  * @file    HalUartInterface.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup   HalUartInterface
  * @addtogroup UART
  * @{
  *****************************************************************************
  */




#ifndef HAL_UART_INTERFACE_H
#define HAL_UART_INTERFACE_H




/********************************* Includes **********************************/
#include "UartInterface.h"




/****************************** HalUartInterface *****************************/
namespace hal{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class Concrete implementation of the abstract UartInterface class, to be
 *        used in production builds
 */
class HalUartInterface : public UartInterface{
public:
    HalUartInterface();
    ~HalUartInterface();

    HAL_StatusTypeDef transmitPoll(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrTransmit,
        size_t numBytes,
        uint32_t timeout
    ) const override final;

    HAL_StatusTypeDef receivePoll(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrReceive,
        size_t numBytes,
        uint32_t timeout
    ) const override final;

#if defined(THREADED)
    HAL_StatusTypeDef transmitIT(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrTransmit,
        size_t numBytes
    ) const override final;

    HAL_StatusTypeDef receiveIT(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrReceive,
        size_t numBytes
    ) const override final;

    HAL_StatusTypeDef transmitDMA(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrTransmit,
        size_t numBytes
    ) const override final;

    HAL_StatusTypeDef receiveDMA(
        const UART_HandleTypeDef* uartHandlePtr,
        uint8_t* arrReceive,
        size_t numBytes
    ) const override final;

    __IO uint32_t getDmaRxInstanceNDTR(
        const UART_HandleTypeDef* uartHandlePtr
    ) const override final;

#endif

    void abortTransmit(
        const UART_HandleTypeDef* uartHandlePtr
    ) const override final;

    void abortReceive(
        const UART_HandleTypeDef* uartHandlePtr
    ) const override final;

    __IO uint32_t getErrorCode(
        const UART_HandleTypeDef* uartHandlePtr
    ) const override final;
};

} // end namespace uart




/**
 * @}
 */
/* end - Header */

#endif /* HAL_UART_INTERFACE_H */
