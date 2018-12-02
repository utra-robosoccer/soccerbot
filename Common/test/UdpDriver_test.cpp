/**
  *****************************************************************************
  * @file    UdpDriver_test.cpp
  * @author  Robert Fairley
  * @brief   Unit tests for UDP driver.
  *
  * @defgroup udp_driver_test
  * @brief    Unit tests for the UdpDriver class.
  * @{
  *****************************************************************************
  */

/* TODO: investigate threaded tests. */

#include <MockOsInterface.h>
#include <MockUdpInterface.h>
#include <UdpDriver.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using udp_driver::UdpDriver;
using mocks::MockUdpInterface;
using mocks::MockOsInterface;

using ::testing::Return;
using ::testing::_;




bool operator==(const ip_addr_t& lhs, const ip_addr_t& rhs) {
    return lhs.addr == rhs.addr;
}

namespace {

MockUdpInterface udp_if;
MockOsInterface os_if;
const ip_addr_t ZERO_IP_ADDR_T = {0x0};
struct pbuf rxPbuf;

// Classes & structs
// ----------------------------------------------------------------------------
class UdpDriverTest : public ::testing::Test {
protected:
    void SetUp() {
        udpDriver_ = new UdpDriver(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, (u16_t) 0, (u16_t) 0,
                &udp_if, &os_if);

    }

    UdpDriver* udpDriver_;
};

}

TEST(UdpDriverShould, DefaultInitializeMembersToZero) {
    UdpDriver udpDriverUnderTest;

    EXPECT_EQ(udpDriverUnderTest.getIpaddr(), ZERO_IP_ADDR_T);
    EXPECT_EQ(udpDriverUnderTest.getIpaddrPc(), ZERO_IP_ADDR_T);
    EXPECT_EQ(udpDriverUnderTest.getPort(), (u16_t) 0);
    EXPECT_EQ(udpDriverUnderTest.getPortPc(), (u16_t) 0);
    EXPECT_EQ(udpDriverUnderTest.getUdpInterface(), nullptr);
    EXPECT_EQ(udpDriverUnderTest.getOsInterface(), nullptr);
    EXPECT_EQ(udpDriverUnderTest.getPcb(), nullptr);
    EXPECT_EQ(udpDriverUnderTest.getRxPbuf(), nullptr);
    EXPECT_EQ(udpDriverUnderTest.getTxPbuf(), nullptr);
}

TEST(UdpDriverShould, InitializeMembersWithParameterizedConstructor) {
    const ip_addr_t TEST_IP_ADDR = {0xC0A80008};
    const ip_addr_t TEST_IP_ADDR_PC = {0xC0A80002};

    UdpDriver udpDriverUnderTest(TEST_IP_ADDR, TEST_IP_ADDR_PC, (u16_t) 7,
            (u16_t) 6340, &udp_if, &os_if);

    EXPECT_EQ(TEST_IP_ADDR, udpDriverUnderTest.getIpaddr());
    EXPECT_EQ(TEST_IP_ADDR_PC, udpDriverUnderTest.getIpaddrPc());
    EXPECT_EQ((u16_t) 7, udpDriverUnderTest.getPort());
    EXPECT_EQ((u16_t) 6340, udpDriverUnderTest.getPortPc());
    EXPECT_EQ(&udp_if, udpDriverUnderTest.getUdpInterface());
    EXPECT_EQ(&os_if, udpDriverUnderTest.getOsInterface());
}


TEST(UdpDriverShould, SucceedSetup) {
    struct udp_pcb udpPcb;

    EXPECT_CALL(udp_if, udpRecv(_, _, _)).Times(1);
    EXPECT_CALL(udp_if, udpBind(_, _, _)).Times(1).WillOnce(Return(ERR_OK));
    EXPECT_CALL(udp_if, udpNew()).Times(1).WillOnce(Return(&udpPcb));
    EXPECT_CALL(os_if, OS_osSemaphoreCreate(_, _)).Times(1).WillOnce(Return((osSemaphoreId) 1));

    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, (u16_t) 0, (u16_t) 0,
            &udp_if, &os_if);
    ASSERT_TRUE(udpDriverUnderTest.setup(nullptr));
}

TEST(UdpDriverShould, FailSetupOnOsSemaphoreCreate) {

    EXPECT_CALL(os_if, OS_osSemaphoreCreate(_, _)).Times(1).WillOnce(Return((osSemaphoreId) 0));

    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, (u16_t) 0, (u16_t) 0,
            &udp_if, &os_if);
    ASSERT_FALSE(udpDriverUnderTest.setup(nullptr));
}

TEST(UdpDriverShould, FailSetupOnUdpNew) {

    EXPECT_CALL(udp_if, udpNew()).Times(1).WillOnce(Return(nullptr));
    EXPECT_CALL(os_if, OS_osSemaphoreCreate(_, _)).Times(1).WillOnce(Return((osSemaphoreId) 1));

    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, (u16_t) 0, (u16_t) 0,
            &udp_if, &os_if);
    ASSERT_FALSE(udpDriverUnderTest.setup(nullptr));
}

TEST(UdpDriverShould, FailSetupOnUdpBind) {
    struct udp_pcb udpPcb;

    EXPECT_CALL(udp_if, udpRemove(_)).Times(1);
    EXPECT_CALL(udp_if, udpBind(_, _, _)).Times(1).WillOnce(Return(ERR_USE));
    EXPECT_CALL(udp_if, udpNew()).Times(1).WillOnce(Return(&udpPcb));
    EXPECT_CALL(os_if, OS_osSemaphoreCreate(_, _)).Times(1).WillOnce(Return((osSemaphoreId) 1));

    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, (u16_t) 0, (u16_t) 0,
            &udp_if, &os_if);
    ASSERT_FALSE(udpDriverUnderTest.setup(nullptr));
}

TEST(UdpDriverRxPbufFilledShould, SucceedReceive) {
    uint8_t rxBuff[10] = {};

    //EXPECT_CALL(udp_if, pbufFree(_)).Times(1).WillOnce(Return((u8_t) 1));
    EXPECT_CALL(udp_if, pbufCopyPartial(_, _, _, _)).Times(1).WillOnce(Return((u16_t) 1));
    EXPECT_CALL(os_if, OS_osSemaphoreWait(_, _)).Times(1).WillOnce(Return(osOK));

    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, (u16_t) 0, (u16_t) 0,
            &udp_if, &os_if);
    ASSERT_TRUE(udpDriverUnderTest.receive(rxBuff, sizeof(rxBuff)));
}
