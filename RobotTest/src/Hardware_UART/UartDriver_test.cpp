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
#include "FreeRTOSInterface.h"

#include "MockUartInterface.h"
#include "MockFreeRTOSInterface.h"


#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::Return;
using ::testing::_;


using namespace UART;
using namespace Test_UartInterface;
using namespace Test_FreeRTOS_Interface;




/******************************** File-local *********************************/
namespace{
// Functions
// ----------------------------------------------------------------------------
TEST(UartInterfaceTests, CanSetUartInterface){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;

    UARTxDriver.setUartInterface(&UartInterface);
}

TEST(UartInterfaceTests, CanSetUartPtr){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};

    EXPECT_CALL(UartInterface, setUartPtr(_)).Times(1);
    UARTxDriver.setUartPtr(&UARTx);
}

TEST(UartInterfaceTests, CanGetIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;

    ASSERT_EQ(UART::IO_Type::POLL, UARTxDriver.getIOType());
}

TEST(UartInterfaceTests, CanSetIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;

    UARTxDriver.setIOType(UART::IO_Type::DMA);

    ASSERT_EQ(UART::IO_Type::DMA, UARTxDriver.getIOType());
}

TEST(UartInterfaceTests, MockFunctionCallsTransmitPollForPollIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, MockFunctionCallsTransmitITForITIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, MockFunctionCallsTransmitDMAForDMAIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMATransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollTransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

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
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::IT);

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
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::DMA);

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
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITTransmitCanSucceed){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(UartInterface, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMATransmitCanSucceed){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(UartInterface, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, MockFunctionCallsReceivePollForPollIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsReceiveITForITIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsReceiveDMAForDMAIOType){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(UartInterface, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMAReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(UartInterface, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(UartInterface, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, PollReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

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
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::IT);

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
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::DMA);

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
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, ITReceiveCanSucceed){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(UartInterface, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(UartInterface, TODO:OS).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

// Failing; needs OS mocks
TEST(UartInterfaceTests, DMAReceiveCanSucceed){
    MockUartInterface UartInterface;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&UartInterface);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

    UARTxDriver.setIOType(UART::IO_Type::POLL);

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
