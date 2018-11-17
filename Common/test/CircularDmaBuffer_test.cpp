/**
 *****************************************************************************
 * @file   CircularDmaBuffer_test.cpp
 * @author Robert
 *
 * @defgroup circular_dma_buffer_test
 * @ingroup  circular_dma_buffer
 * @brief    Unit tests for the CircularDmaBuffer class.
 * @{
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "CircularDmaBuffer.h"

#include "MockUartInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using uart::CircularDmaBuffer;
using mocks::MockUartInterface;

using ::testing::Return;
using ::testing::_;

/******************************** File-local *********************************/
namespace {
// Variables
// ----------------------------------------------------------------------------
MockUartInterface uart_if;

// Functions
// ----------------------------------------------------------------------------
TEST(CircularDmaBufferShould, InitializeMembersZeroWithDefaultConstructor) {
    CircularDmaBuffer buff;

    EXPECT_EQ(buff.getUartHandle(), nullptr);
    EXPECT_EQ(buff.getHwIf(), nullptr);
    EXPECT_EQ(buff.getTransmissionSize(), 0);
    EXPECT_EQ(buff.getBuffSize(), 0);
    EXPECT_EQ(buff.getBuffP(), nullptr);
    EXPECT_EQ(buff.getBuffHead(), 0);
    EXPECT_EQ(buff.getBuffTail(), 0);
}

TEST(CircularDmaBufferShould, ConstInitializeMembersWithParameterizedConstructor) {
    constexpr size_t BUFFER_SIZE = 20;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE] = { };
    const uint16_t transmission_size = 10;
    const size_t buffer_size = BUFFER_SIZE;

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size,
            raw_buff);

    EXPECT_EQ(buff.getUartHandle(), &huart);
    EXPECT_EQ(buff.getHwIf(), &uart_if);
    EXPECT_EQ(buff.getTransmissionSize(), transmission_size);
    EXPECT_EQ(buff.getBuffSize(), buffer_size);
    EXPECT_EQ(buff.getBuffP(), raw_buff);
    EXPECT_EQ(buff.getBuffHead(), 0);
    EXPECT_EQ(buff.getBuffTail(), 0);
}

TEST(CircularDmaBufferShould, SucceedSelfCheck) {
    constexpr size_t BUFFER_SIZE = 20;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE] = { };
    const uint16_t transmission_size = BUFFER_SIZE;
    const size_t buffer_size = BUFFER_SIZE;

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size,
            raw_buff);

    EXPECT_TRUE(buff.selfCheck());
}

TEST(CircularDmaBufferShould, FailSelfCheck) {
    constexpr size_t BUFFER_SIZE = 8;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE] = { };
    const uint16_t transmission_size = 10;
    const size_t buffer_size = BUFFER_SIZE;

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size,
            raw_buff);

    EXPECT_FALSE(buff.selfCheck());
}

TEST(CircularDmaBufferShould, UpdateHeadWhenNDTRChanged) {
    constexpr size_t BUFFER_SIZE = 100;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE];
    const uint16_t transmission_size = BUFFER_SIZE;
    const size_t buffer_size = BUFFER_SIZE;

    EXPECT_CALL(uart_if, getDmaRxInstanceNDTR(_)).Times(2).WillOnce(Return(transmission_size)).WillOnce(Return(32));
    EXPECT_CALL(uart_if, receiveDMA(_, _, _)).Times(1);

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size, raw_buff);

    ASSERT_EQ(buff.getBuffSize(), BUFFER_SIZE);
    ASSERT_EQ(buff.getTransmissionSize(), BUFFER_SIZE);
    ASSERT_EQ(buff.getBuffHead(), 0);
    EXPECT_EQ(buff.getBuffTail(), 0);

    /* Initiate a transfer. Assume the initiation completed successfully. */
    buff.initiate();

    /* NDTR did not change and stays at transmission_size (100). */
    EXPECT_EQ(buff.updateHead(), 0);

    ASSERT_EQ(buff.getBuffHead(), 0);
    EXPECT_EQ(buff.getBuffTail(), 0);

    EXPECT_EQ(buff.updateHead(), 68);

    /* NDTR has changed to 32, so 68 bytes were received. */
    EXPECT_EQ(buff.getBuffHead(), 68);

    EXPECT_EQ(buff.getBuffTail(), 0);
}

} // end anonymous namespace

/**
 * @}
 */
/* end - circular_dma_buffer */
