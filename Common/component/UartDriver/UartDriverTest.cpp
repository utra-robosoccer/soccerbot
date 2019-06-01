/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  *
  * @defgroup UartDriver_Test
  * @ingroup UART
  * @brief UartDriver test driver
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "UartInterface.h"
#include "UartDriver/UartDriver.h"
#include "Notification.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "osInterfaceMock.h"
#include "UartInterfaceMock.h"


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;
using ::testing::NiceMock;


using uart::UartDriver;
using cmsis::gmock::OsInterfaceMock;
using hal::gmock::UartInterfaceMock;
using hal::IO_Type;




/******************************** File-local *********************************/
namespace{
// Functions
// ----------------------------------------------------------------------------
TEST(UartDriver, CanGetIOType){
    UartDriver UARTxDriver;

    ASSERT_EQ(IO_Type::POLL, UARTxDriver.getIOType());
}

TEST(UartDriver, CanSetIOType){
    UartDriver UARTxDriver;
    UARTxDriver.setIOType(IO_Type::DMA);

    ASSERT_EQ(IO_Type::DMA, UARTxDriver.getIOType());
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitWithNullInitializers){
    UartDriver UARTxDriver(nullptr, nullptr, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveWithNullInitializers){
    UartDriver UARTxDriver(nullptr, nullptr, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitWithNullUartInterface){
    NiceMock<OsInterfaceMock> os;

    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, nullptr, &UARTx);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveWithNullUartInterface){
    NiceMock<OsInterfaceMock> os;

    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, nullptr, &UARTx);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitWithNullUartHandle){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;

    UartDriver UARTxDriver(&os, &uart, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveWithNullUartHandle){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;

    UartDriver UARTxDriver(&os, &uart, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitITWithNullOSInterface){
    NiceMock<UartInterfaceMock> uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitDMAWithNullOSInterface){
    NiceMock<UartInterfaceMock> uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveITWithNullOSInterface){
    NiceMock<UartInterfaceMock> uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveDMAWithNullOSInterface){
    NiceMock<UartInterfaceMock> uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldCallTransmitPollForPollIOType){
    NiceMock<UartInterfaceMock> uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartDriver, ShouldCallTransmitITPollForITIOType){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartDriver, ShouldCallTransmitDMAForDMAIOType){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    UARTxDriver.transmit(arr, sizeof(arr));
}

TEST(UartDriver, ShouldFailAndAbortTransmitPollWhenHardwareCallFails){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _, _)).Times(1).WillOnce(
        Return(HAL_ERROR)
    );

    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortTransmitITWhenHardwareCallFails){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(
        Return(HAL_ERROR)
    );

    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortTransmitDMAWhenHardwareCallFails){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(
        Return(HAL_ERROR)
    );

    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortTransmitITWhenOSBlockTimesOut){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        Return(pdFALSE)
    );

    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortTransmitDMAWhenOSBlockTimesOut){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        Return(pdFALSE)
    );

    EXPECT_CALL(uart, abortTransmit(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, PollTransmitCanSucceed){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::POLL);

    EXPECT_CALL(uart, transmitPoll(_, _, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, ITTransmitCanSucceed){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        DoAll(SetArgPointee<2>(NOTIFIED_FROM_TX_ISR), Return(pdTRUE))
    );

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, DMATransmitCanSucceed){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        DoAll(SetArgPointee<2>(NOTIFIED_FROM_TX_ISR), Return(pdTRUE))
    );

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, ShouldCallReceivePollForPollIOType){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartDriver, ShouldCallReceiveITForITIOType){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartDriver, ShouldCallReceiveDMAForDMAIOType){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    UARTxDriver.receive(arr, sizeof(arr));
}

TEST(UartDriver, ShouldFailAndAbortReceivePollWhenHardwareCallFails){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _, _)).Times(1).WillOnce(
        Return(HAL_ERROR)
    );

    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortReceiveITWhenHardwareCallFails){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(
        Return(HAL_ERROR)
    );

    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortReceiveDMAWhenHardwareCallFails){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(
        Return(HAL_ERROR)
    );

    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortReceiveITWhenOSBlockTimesOut){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        Return(pdFALSE)
    );

    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldFailAndAbortReceiveDMAWhenOSBlockTimesOut){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        Return(pdFALSE)
    );

    EXPECT_CALL(uart, abortReceive(_)).Times(1);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_FALSE(success);
}

TEST(UartDriver, PollReceiveCanSucceed){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::POLL);

    EXPECT_CALL(uart, receivePoll(_, _, _, _)).Times(1).WillOnce(
        Return(HAL_OK)
    );

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, ITReceiveCanSucceed){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        DoAll(SetArgPointee<2>(NOTIFIED_FROM_RX_ISR), Return(pdTRUE))
    );

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartDriver, DMAReceiveCanSucceed){
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _, _)).Times(1).WillOnce(Return(HAL_OK));

    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(
        DoAll(SetArgPointee<2>(NOTIFIED_FROM_RX_ISR), Return(pdTRUE))
    );

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

} // end anonymous namespace

/**
 * @}
 */
/* end - UartDriver_Test */
