/**
  *****************************************************************************
  * @file    UartDriver_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup UartDriver_test
  * @ingroup UART
  * @brief UartDriver unit tests
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "UartDriver.h"
#include "FreeRTOSInterface.h"
#include "Notification.h"

#include "MockUartInterface.h"
#include "MockFreeRTOSInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;


using namespace uart;
using namespace mocks;




/******************************** File-local *********************************/
namespace{
// Functions
// ----------------------------------------------------------------------------
TEST(UartDriver, CanGetIOType){
    UartDriver UARTxDriver;

    ASSERT_EQ(uart::IO_Type::POLL, UARTxDriver.getIOType());
}

TEST(UartDriver, CanSetIOType){
    UartDriver UARTxDriver;
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    ASSERT_EQ(uart::IO_Type::DMA, UARTxDriver.getIOType());
}

TEST(UartDriver, ShouldFailTransmitWithNullInitializers){
    UartDriver UARTxDriver(nullptr, nullptr, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailReceiveWithNullInitializers){
    UartDriver UARTxDriver(nullptr, nullptr, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitWithNullUartInterface){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, nullptr, &UARTx);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveWithNullUartInterface){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, nullptr, &UARTx);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitITWithNullOSInterface){
    MockUartInterface uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitDMAWithNullOSInterface){
    MockUartInterface uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveITWithNullOSInterface){
    MockUartInterface uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveDMAWithNullOSInterface){
    MockUartInterface uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldCallTransmitPollForPollIOType){
    MockUartInterface uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartDriver, MockFunctionCallsTransmitITForITIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartDriver, MockFunctionCallsTransmitDMAForDMAIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartDriver, PollTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ITTransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, DMATransmitFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ITTransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, DMATransmitFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, PollTransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, ITTransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_TX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, DMATransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_TX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, MockFunctionCallsReceivePollForPollIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartDriver, MockFunctionCallsReceiveITForITIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartDriver, MockFunctionCallsReceiveDMAForDMAIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartDriver, PollReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ITReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, DMAReceiveFailsAndIsAbortedWhenUartInterfaceFails){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(Return(HAL_ERROR));
    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ITReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, DMAReceiveFailsAndIsAbortedWhenOSBlockTimesOut){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdFALSE));
    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, PollReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _, _)).Times(1).WillOnce(Return(HAL_OK));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, ITReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_))
        .Times(1).WillOnce(DoAll(SetArgPointee<2>(NOTIFIED_FROM_RX_ISR),Return(pdTRUE)));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, DMAReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(uart::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));
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
