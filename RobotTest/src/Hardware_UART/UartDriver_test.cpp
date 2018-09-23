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
TEST(UartInterfaceTests, CanSetUartInterface){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;

    UARTxDriver.setUartInterface(&UartInterface);
}

TEST(UartInterfaceTests, CanSetUartPtr){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};

    EXPECT_CALL(UartInterface, setUartPtr(_)).Times(1);
    UARTxDriver.setUartPtr(&UARTx);
}

TEST(UartInterfaceTests, CanGetIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;

    ASSERT_EQ(uart::IO_Type::POLL, UARTxDriver.getIOType());
}

TEST(UartInterfaceTests, CanSetIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    ASSERT_EQ(uart::IO_Type::DMA, UARTxDriver.getIOType());
}

TEST(UartInterfaceTests, MockFunctionCallsTransmitPollForPollIOType) {
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

//// Failing; needs OS mocks
//TEST(UartInterfaceTests, MockFunctionCallsTransmitITForITIOType) {
//    MockUartInterface UartInterface;
//    uart::UartDriver<MockUartInterface> UARTxDriver;
//    UARTxDriver.setUartInterface(&UartInterface);
//
//    UART_HandleTypeDef UARTx = {0};
//    UARTxDriver.setUartPtr(&UARTx);
//
//    UARTxDriver.setIOType(uart::IO_Type::IT);
//
//    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
//
//    uint8_t arr[10] = {0};
//    bool success = UARTxDriver.transmit(arr, sizeof(arr));
//    ASSERT_TRUE(success);
//}
//
//// Failing; needs OS mocks
//TEST(UartInterfaceTests, MockFunctionCallsTransmitDMAForDMAIOType) {
//    MockUartInterface UartInterface;
//    uart::UartDriver<MockUartInterface> UARTxDriver;
//    UARTxDriver.setUartInterface(&UartInterface);
//
//    UART_HandleTypeDef UARTx = {0};
//    UARTxDriver.setUartPtr(&UARTx);
//
//    UARTxDriver.setIOType(uart::IO_Type::DMA);
//
//    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
//
//    uint8_t arr[10] = {0};
//    bool success = UARTxDriver.transmit(arr, sizeof(arr));
//    ASSERT_TRUE(success);
//}

} // end anonymous namespace

/**
 * @}
 */
/* end - UartDriver_test */
