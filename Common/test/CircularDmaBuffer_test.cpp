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
using ::testing::Ge;




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

// TODO: define different test fixtures with variants of m_transmission_size vs. m_buffer_size comparison?

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

TEST(CircularDmaBufferShould, Initiate) {
    constexpr size_t BUFFER_SIZE = 20;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE] = { };
    const uint16_t transmission_size = BUFFER_SIZE;
    const size_t buffer_size = BUFFER_SIZE;

    EXPECT_CALL(uart_if, receiveDMA(_, _, _)).Times(1);

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size,
            raw_buff);

    buff.initiate();

    EXPECT_TRUE(buff.selfCheck());
}

TEST(CircularDmaBufferShould, AbortReinitiateIfError) {
    constexpr size_t BUFFER_SIZE = 20;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE] = { };
    const uint16_t transmission_size = BUFFER_SIZE;
    const size_t buffer_size = BUFFER_SIZE;

    EXPECT_CALL(uart_if, receiveDMA(_, _, _)).Times(1);
    EXPECT_CALL(uart_if, abortReceive(_)).Times(1);
    EXPECT_CALL(uart_if, getErrorCode(_)).Times(1).WillOnce(Return(HAL_UART_ERROR_DMA));

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size,
            raw_buff);

    buff.reinitiateIfError();
}

TEST(CircularDmaBufferShould, NotAbortReinitiateIfNoError) {
    constexpr size_t BUFFER_SIZE = 20;
    UART_HandleTypeDef huart;
    uint8_t raw_buff[BUFFER_SIZE] = { };
    const uint16_t transmission_size = BUFFER_SIZE;
    const size_t buffer_size = BUFFER_SIZE;

    EXPECT_CALL(uart_if, receiveDMA(_, _, _)).Times(0);
    EXPECT_CALL(uart_if, abortReceive(_)).Times(0);
    EXPECT_CALL(uart_if, getErrorCode(_)).Times(1).WillOnce(Return(HAL_UART_ERROR_NONE));

    CircularDmaBuffer buff(&huart, &uart_if, transmission_size, buffer_size,
            raw_buff);

    buff.reinitiateIfError();
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

TEST_F(CircularDmaBufferTest, PeekBuff) {
    uint8_t process_buff[BUFFER_SIZE_TEST] = {};

    const char* message_a = "Hello World";
    const char* message_b = "A long message that contains 96 characters A long message that contains 96 characters A long mes";
    const size_t num_bytes_received[2] = {11, 96};

    ASSERT_THAT(transmission_size_, AllOf(Ge(num_bytes_received[0]), Ge(num_bytes_received[1])));

    /* To set up test, write message_a into the raw buffer. */
    for (size_t i = 0; i < num_bytes_received[0]; i++) {
        raw_buff_[i] = message_a[i];
    }
    EXPECT_CALL(uart_if, getDmaRxInstanceNDTR(_)).Times(2)
                                                 .WillOnce(Return(transmission_size_ - num_bytes_received[0]))

                                                 /* Check with the second message causing head to wrap around. */
                                                 .WillOnce(Return(transmission_size_ - ((num_bytes_received[0] + num_bytes_received[1]) % transmission_size_)));

    /* Update head and confirm that data is detected as available. */
    EXPECT_EQ(buff_->updateHead(), num_bytes_received[0]);
    EXPECT_TRUE(buff_->dataAvail());

    EXPECT_EQ(buff_->peekBuff(process_buff), num_bytes_received[0]);

    /* Check that process_buff ends up holding message_a. */
    for (size_t i = 0; i < num_bytes_received[0]; i++) {
        EXPECT_EQ(process_buff[i], message_a[i]);
    }

    /* Tail should not have changed. */
    EXPECT_EQ(buff_->getBuffTail(), 0);

    /* Manually catch up tail to head in preparation for next test. */
    EXPECT_EQ(buff_->catchupTail(), num_bytes_received[0]);

    /* To set up test, write message_b into the raw buffer, by wrapping around. */
    for (size_t i = 0; i < num_bytes_received[1]; i++) {
        raw_buff_[(i + num_bytes_received[0]) % transmission_size_] = message_b[i];
    }

    /* Update head and confirm that data is detected as available. */
    EXPECT_EQ(buff_->updateHead(), (num_bytes_received[0] + num_bytes_received[1]) % transmission_size_);
    EXPECT_TRUE(buff_->dataAvail());

    EXPECT_EQ(buff_->peekBuff(process_buff), num_bytes_received[1]);

    /* Check that process_buff ends up holding message_b. */
    for (size_t i = 0; i < num_bytes_received[1]; i++) {
        EXPECT_EQ(process_buff[i], message_b[i]);
    }

    /* Tail should not have changed since catching up. */
    EXPECT_EQ(buff_->getBuffTail(), num_bytes_received[0]);
}

TEST_F(CircularDmaBufferTest, ReadBuff) {
    uint8_t process_buff[BUFFER_SIZE_TEST] = {};

    const char* message_a = "Hello World";
    const char* message_b = "A long message that contains 96 characters A long message that contains 96 characters A long mes";
    const size_t num_bytes_received[2] = {11, 96};

    ASSERT_THAT(transmission_size_, AllOf(Ge(num_bytes_received[0]), Ge(num_bytes_received[1])));
    ASSERT_EQ(transmission_size_, buffer_size_);

    /* To set up test, write message_a into the raw buffer. */
    for (size_t i = 0; i < num_bytes_received[0]; i++) {
        raw_buff_[i] = message_a[i];
    }
    EXPECT_CALL(uart_if, getDmaRxInstanceNDTR(_)).Times(2)
                                                 .WillOnce(Return(transmission_size_ - num_bytes_received[0]))

                                                 /* Check with the second message causing head to wrap around. */
                                                 .WillOnce(Return(transmission_size_ - ((num_bytes_received[0] + num_bytes_received[1]) % transmission_size_)));

    /* Update head and confirm that data is detected as available. */
    EXPECT_EQ(buff_->updateHead(), num_bytes_received[0]);
    EXPECT_TRUE(buff_->dataAvail());

    EXPECT_EQ(buff_->readBuff(process_buff), num_bytes_received[0]);

    /* Check that process_buff ends up holding message_a. */
    for (size_t i = 0; i < num_bytes_received[0]; i++) {
        EXPECT_EQ(process_buff[i], message_a[i]);
    }

    /* Tail should have changed. */
    EXPECT_EQ(buff_->getBuffTail(), num_bytes_received[0]);

    /* To set up test, write message_b into the raw buffer, by wrapping around. */
    for (size_t i = 0; i < num_bytes_received[1]; i++) {
        raw_buff_[(i + num_bytes_received[0]) % transmission_size_] = message_b[i];
    }

    /* Update head and confirm that data is detected as available. */
    EXPECT_EQ(buff_->updateHead(), (num_bytes_received[0] + num_bytes_received[1]) % transmission_size_);
    EXPECT_TRUE(buff_->dataAvail());

    EXPECT_EQ(buff_->readBuff(process_buff), num_bytes_received[1]);

    /* Check that process_buff ends up holding message_b. */
    for (size_t i = 0; i < num_bytes_received[1]; i++) {
        EXPECT_EQ(process_buff[i], message_b[i]);
    }

    /* Tail should have changed. */
    EXPECT_EQ(buff_->getBuffTail(), (num_bytes_received[0] + num_bytes_received[1]) % transmission_size_);
}

// TODO (future): tests with edge cases like m_transmission_size != m_buffer_size.

} // end anonymous namespace

/**
 * @}
 */
/* end - circular_dma_buffer */
