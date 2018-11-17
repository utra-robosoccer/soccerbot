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
using ::testing::Test;
using ::testing::Le;
using ::testing::Lt;
using ::testing::Eq;
using ::testing::AllOf;

/******************************** File-local *********************************/
namespace {
// Variables
// ----------------------------------------------------------------------------
constexpr size_t BUFFER_SIZE_TEST = 100;
MockUartInterface uart_if;

// Classes & structs
// ----------------------------------------------------------------------------
class CircularDmaBufferTest : public ::testing::Test {
protected:
    void SetUp() override {
        buff_ = new CircularDmaBuffer(&huart_, &uart_if, transmission_size_, buffer_size_, raw_buff_);
    }

    void TearDown() override {
        delete buff_;
    }

    UART_HandleTypeDef huart_;
    uint8_t raw_buff_[BUFFER_SIZE_TEST] = { };
    const uint16_t transmission_size_ = BUFFER_SIZE_TEST;
    const size_t buffer_size_ = BUFFER_SIZE_TEST;
    CircularDmaBuffer * buff_ = nullptr;
};

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

TEST_F(CircularDmaBufferTest, UpdateHeadAfterNDTRChanged) {
    constexpr size_t num_bytes_received = 68;
    ASSERT_THAT(num_bytes_received, Le(transmission_size_));

    EXPECT_CALL(uart_if, getDmaRxInstanceNDTR(_)).Times(2).WillOnce(Return(transmission_size_)).WillOnce(Return(transmission_size_ - num_bytes_received));

    /* NDTR did not change, so head does not change. */
    EXPECT_EQ(buff_->updateHead(), 0);
    ASSERT_EQ(buff_->getBuffHead(), 0);

    /* NDTR changed, head should reflect the number of bytes received. */
    EXPECT_EQ(buff_->updateHead(), num_bytes_received);
    EXPECT_EQ(buff_->getBuffHead(), num_bytes_received);

    /* Tail should not have changed. */
    EXPECT_EQ(buff_->getBuffTail(), 0);
}

TEST_F(CircularDmaBufferTest, CatchUpTailAfterNDTRChanged) {
    constexpr size_t num_bytes_received = 68;
    ASSERT_THAT(num_bytes_received, Le(transmission_size_));
    EXPECT_THAT(buff_->getBuffTail(), AllOf(Eq(buff_->getBuffTail()), Eq(0)));

    EXPECT_CALL(uart_if, getDmaRxInstanceNDTR(_)).Times(1)
                                                 .WillOnce(Return(transmission_size_ - num_bytes_received));

    /* NDTR changed, head should be ahead of tail. */
    EXPECT_EQ(buff_->updateHead(), num_bytes_received);
    EXPECT_THAT(buff_->getBuffTail(), Lt(buff_->getBuffHead()));

    /* After catching up tail, tail should equal head. */
    EXPECT_EQ(buff_->catchupTail(), num_bytes_received);
    EXPECT_EQ(buff_->getBuffTail(), buff_->getBuffHead());
}

TEST_F(CircularDmaBufferTest, DataAvailAfterNDTRChanged) {
    constexpr size_t num_bytes_received = 68;
    ASSERT_THAT(num_bytes_received, Le(transmission_size_));

    EXPECT_CALL(uart_if, getDmaRxInstanceNDTR(_)).Times(2).WillOnce(Return(transmission_size_)).WillOnce(Return(transmission_size_ - num_bytes_received));

    /* NDTR did not change, data should not show as available. */
    EXPECT_EQ(buff_->updateHead(), 0);
    ASSERT_FALSE(buff_->dataAvail());

    /* NDTR changed, data should show as available. */
    EXPECT_EQ(buff_->updateHead(), num_bytes_received);
    EXPECT_TRUE(buff_->dataAvail());

    /* Tail should not have changed. */
    EXPECT_EQ(buff_->getBuffTail(), 0);
}

} // end anonymous namespace

/**
 * @}
 */
/* end - circular_dma_buffer */
