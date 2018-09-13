/**
  *****************************************************************************
  * @file    UartInterface_test.cpp
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup UartInterface_test
  * @brief    TODO -- description of module
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#define STM32F446xx
#include <stm32f4xx.h>
#include "UartInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::Return;
using ::testing::_;




/******************************** File-local *********************************/
namespace{
// Classes and structs
// ----------------------------------------------------------------------------
class MockUartInterface : public uart::UartInterface {
public:
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
    UART_HandleTypeDef UARTx;
    uart::UartDriver UARTxDriver(&UARTx);
    ASSERT_EQ(uart::IO_Type::POLL, UARTxDriver.getIOType());
}

} // end anonymous namespace

/**
 * @}
 */
/* end - HalUartInterface_test */
