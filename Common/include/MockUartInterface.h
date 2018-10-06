/**
  *****************************************************************************
  * @file    MockUartInterface.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup MockUartInterface
  * @ingroup Mocks
  * @{
  *****************************************************************************
  */




#ifndef MOCK_UART_INTERFACE_H
#define MOCK_UART_INTERFACE_H




/********************************* Includes **********************************/
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "UartInterface.h"
using namespace uart;




/***************************** Test_UartInterface ****************************/
namespace mocks{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class MockUartInterface Implements UartInterface for unit testing purposes
 */
class MockUartInterface : public UartInterface{
public:
    MOCK_CONST_METHOD4(
        transmitPoll,
        HAL_StatusTypeDef(
            const UART_HandleTypeDef*,
            uint8_t*,
            size_t,
            uint32_t
        )
    );

    MOCK_CONST_METHOD4(
        receivePoll,
        HAL_StatusTypeDef(
            const UART_HandleTypeDef*,
            uint8_t*,
            size_t,
            uint32_t
        )
    );

    MOCK_CONST_METHOD3(
        transmitDMA,
        HAL_StatusTypeDef(const UART_HandleTypeDef*, uint8_t*, size_t)
    );

    MOCK_CONST_METHOD3(
        receiveDMA,
        HAL_StatusTypeDef(const UART_HandleTypeDef*, uint8_t*, size_t)
    );

    MOCK_CONST_METHOD3(
        transmitIT,
        HAL_StatusTypeDef(const UART_HandleTypeDef*, uint8_t*, size_t)
    );

    MOCK_CONST_METHOD3(
        receiveIT,
        HAL_StatusTypeDef(const UART_HandleTypeDef*, uint8_t*, size_t)
    );

    MOCK_CONST_METHOD1(abortTransmit, void(const UART_HandleTypeDef*));
    MOCK_CONST_METHOD1(abortReceive, void(const UART_HandleTypeDef*));
};

} // end namespace Test_FreeRTOS_Interface




/**
 * @}
 */
/* end - MockUartInterface */

#endif /* MOCK_UART_INTERFACE_H */
