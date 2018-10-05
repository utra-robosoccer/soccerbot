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
using namespace UART;




/***************************** Test_UartInterface ****************************/
namespace MOCKS{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class MockUartInterface Implements UartInterface for unit testing purposes
 */
class MockUartInterface : public UartInterface{
public:
    MOCK_METHOD1(setUartPtr, void(UART_HandleTypeDef*));
    MOCK_METHOD3(transmitPoll, HAL_StatusTypeDef(uint8_t*, size_t, uint32_t));
    MOCK_METHOD3(receivePoll, HAL_StatusTypeDef(uint8_t*, size_t, uint32_t));
    MOCK_METHOD2(transmitDMA, HAL_StatusTypeDef(uint8_t*, size_t));
    MOCK_METHOD2(receiveDMA, HAL_StatusTypeDef(uint8_t*, size_t));
    MOCK_METHOD2(transmitIT, HAL_StatusTypeDef(uint8_t*, size_t));
    MOCK_METHOD2(receiveIT, HAL_StatusTypeDef(uint8_t*, size_t));
    MOCK_METHOD0(abortTransmit, void());
    MOCK_METHOD0(abortReceive, void());
};

} // end namespace Test_FreeRTOS_Interface




/**
 * @}
 */
/* end - MockUartInterface */

#endif /* MOCK_UART_INTERFACE_H */
