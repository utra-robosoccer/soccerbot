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
class MockUartInterface : public uart::UartInterface{
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

TEST(UartInterfaceTests, MockFunctionCallsTransmitPollForPollIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, MockFunctionCallsTransmitITForITIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, MockFunctionCallsTransmitDMAForDMAIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMATransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollTransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITTransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMATransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollTransmitCanSucceed){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITTransmitCanSucceed){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMATransmitCanSucceed){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, MockFunctionCallsReceivePollForPollIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsReceiveITForITIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsReceiveDMAForDMAIOType){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(UartInterface, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMAReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(UartInterface, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMAReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(UartInterface, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollReceiveCanSucceed){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITReceiveCanSucceed){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMAReceiveCanSucceed){
    MockUartInterface UartInterface;
    uart::UartDriver<MockUartInterface> UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

} // end anonymous namespace

/**
 * @}
 */
/* end - UartDriver_test */
