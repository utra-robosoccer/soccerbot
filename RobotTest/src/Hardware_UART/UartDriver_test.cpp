/**
  *****************************************************************************
  * @file    UartDriver_test.cpp
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup UartDriver_test
  * @ingroup  UART
  * @brief    TODO -- description of module
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "UartDriver.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::Return;
using ::testing::_;




/******************************** File-local *********************************/
namespace{
// Classes and structs
// ----------------------------------------------------------------------------
class MockUartInterface{
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





// Functions
// ----------------------------------------------------------------------------
TEST(UartInterfaceTests, IOTypeDefaultsToPolled) {
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;

    UART_HandleTypeDef UARTx;
    UARTxDriver.setUartPtr(&UARTx);

    ASSERT_EQ(uart::IO_Type::POLL, UARTxDriver.getIOType());
}

} // end anonymous namespace

/**
 * @}
 */
/* end - UartDriver_test */
