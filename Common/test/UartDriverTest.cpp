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

#include "OsInterfaceMock.h"
#include "UartInterfaceMock.h"


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;


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
    OsInterfaceMock os;

    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, nullptr, &UARTx);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveWithNullUartInterface){
    OsInterfaceMock os;

    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(&os, nullptr, &UARTx);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitWithNullUartHandle){
    UartInterfaceMock uart;
    OsInterfaceMock os;

    UartDriver UARTxDriver(&os, &uart, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailReceiveWithNullUartHandle){
    UartInterfaceMock uart;
    OsInterfaceMock os;

    UartDriver UARTxDriver(&os, &uart, nullptr);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

// This is needed so that the driver won't work unless it's initialized
// properly
TEST(UartDriver, ShouldFailTransmitITWithNullOSInterface){
    UartInterfaceMock uart;
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
    UartInterfaceMock uart;
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
    UartInterfaceMock uart;
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
    UartInterfaceMock uart;
    UART_HandleTypeDef UARTx = {0};

    UartDriver UARTxDriver(nullptr, &uart, &UARTx);
    UARTxDriver.setIOType(IO_Type::DMA);

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));

    ASSERT_FALSE(success);
}

TEST(UartDriver, ShouldCallTransmitPollForPollIOType){
    UartInterfaceMock uart;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
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
