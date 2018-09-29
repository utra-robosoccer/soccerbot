/**
  *****************************************************************************
  * @file    UartDriver_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup   UartDriver_test
  * @addtogroup UART
  * @brief      UartDriver unit tests
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


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;


using namespace UART;
using namespace MOCKS;




/******************************** File-local *********************************/
namespace{
// Functions
// ----------------------------------------------------------------------------
TEST(UartInterfaceTests, CanSetUartInterface){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
}

TEST(UartInterfaceTests, CanGetIOType){
    UartDriver UARTxDriver;

    ASSERT_EQ(UART::IO_Type::POLL, UARTxDriver.getIOType());
}

TEST(UartInterfaceTests, CanSetIOType){
    UartDriver UARTxDriver;

    UARTxDriver.setIOType(UART::IO_Type::DMA);

    ASSERT_EQ(UART::IO_Type::DMA, UARTxDriver.getIOType());
}

TEST(UartInterfaceTests, TransmitFailsOnNullUartInterface){
    UartDriver UARTxDriver;

    UARTxDriver.setUartInterface(nullptr, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, ReceiveFailsOnNullUartInterface){
    UartDriver UARTxDriver;

    UARTxDriver.setUartInterface(nullptr, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartInterfaceTests, transmitFailsWhensetUartInterfaceIsNotCalled){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartInterfaceTests, receiveFailsWhensetUartInterfaceIsNotCalled){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartInterfaceTests, TransmitITFailsOnNullFreeRTOSInterface){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartInterfaceTests, TransmitDMAFailsOnNullFreeRTOSInterface){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartInterfaceTests, ReceiveITFailsOnNullFreeRTOSInterface){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartInterfaceTests, ReceiveDMAFailsOnNullFreeRTOSInterface){
    MockUartInterface uart;
    UartDriver UARTxDriver;

    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, MockFunctionCallsTransmitPollForPollIOType){
    MockUartInterface uart;
    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsTransmitITForITIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsTransmitDMAForDMAIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartInterfaceTests, PollTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, ITTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, DMATransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, ITTransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, DMATransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortTransmit()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, PollTransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, ITTransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_TX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, DMATransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_TX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, MockFunctionCallsReceivePollForPollIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsReceiveITForITIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, MockFunctionCallsReceiveDMAForDMAIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartInterfaceTests, PollReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, ITReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, DMAReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, ITReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, DMAReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortReceive()).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartInterfaceTests, PollReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setIOType(UART::IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, ITReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_RX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, DMAReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart, &UARTx);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_RX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

} // end anonymous namespace

/**
 * @}
 */
/* end - UartDriver_test */
