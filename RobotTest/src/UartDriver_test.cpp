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
    MockUartInterface uart;
    UartDriver UARTxDriver;

    UARTxDriver.setUartInterface(&uart);
}

TEST(UartInterfaceTests, CanSetUartPtr){
    MockUartInterface uart;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&uart);

    UART_HandleTypeDef UARTx = {0};

    EXPECT_CALL(uart, setUartPtr(_)).Times(1);
    UARTxDriver.setUartPtr(&UARTx);
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

TEST(UartInterfaceTests, MockFunctionCallsTransmitPollForPollIOType){
    MockUartInterface uart;
    UartDriver UARTxDriver;
    UARTxDriver.setUartInterface(&uart);

    UART_HandleTypeDef UARTx = {0};
    UARTxDriver.setUartPtr(&UARTx);

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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, transmitIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, DMATransmitCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, transmitDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.transmit(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, MockFunctionCallsReceivePollForPollIOType){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
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

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
    UARTxDriver.setIOType(UART::IO_Type::IT);

    EXPECT_CALL(uart, receiveIT(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

TEST(UartInterfaceTests, DMAReceiveCanSucceed){
    MockUartInterface uart;
    MockFreeRTOSInterface os;

    UartDriver UARTxDriver;
    UART_HandleTypeDef UARTx = {0};

    UARTxDriver.setUartInterface(&uart);
    UARTxDriver.setOSInterface(&os);
    UARTxDriver.setUartPtr(&UARTx);
    UARTxDriver.setIOType(UART::IO_Type::DMA);

    EXPECT_CALL(uart, receiveDMA(_, _)).Times(1).WillOnce(Return(HAL_OK));
    EXPECT_CALL(os, OS_xTaskNotifyWait(_,_,_,_)).Times(1).WillOnce(Return(pdTRUE));

    uint8_t arr[10] = {0};
    bool success = UARTxDriver.receive(arr, sizeof(arr));
    ASSERT_TRUE(success);
}

} // end anonymous namespace

/**
 * @}
 */
/* end - UartDriver_test */
