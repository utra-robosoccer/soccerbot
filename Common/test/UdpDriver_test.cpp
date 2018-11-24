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



namespace {

MockUdpInterface udp_if;
MockOsInterface os_if;
const ip_addr_t ZERO_IP_ADDR_T = {0x0};

bool operator==(const ip_addr_t& lhs, const ip_addr_t& rhs) {
    return lhs.addr == rhs.addr;
}

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

//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderSetupSuccess) {
//    mocks::MockUdpInterface mockUdpInterface;
//    mocks::MockOsInterface mockOsInterface;
//    EXPECT_CALL(mockUdpInterface, udpRecv(_, _, _)).Times(1);
//    EXPECT_CALL(mockUdpInterface, udpBind(_, _, _)).Times(1).WillOnce(
//            Return(ERR_OK));
//    EXPECT_CALL(mockUdpInterface, udpNew()).Times(1).WillOnce(
//            Return(NON_NULL_PTR_PCB));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, &mockOsInterface);
//    bool success = udpDriverUnderTest.setup();
//    ASSERT_TRUE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderSetupFailsOnNew) {
//    mocks::MockUdpInterface mockUdpInterface;
//    mocks::MockOsInterface mockOsInterface;
//    EXPECT_CALL(mockUdpInterface, udpNew()).Times(1).WillOnce(Return(nullptr));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, &mockOsInterface);
//    bool success = udpDriverUnderTest.setup();
//    ASSERT_FALSE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderSetupFailsOnBind) {
//    mocks::MockUdpInterface mockUdpInterface;
//    mocks::MockOsInterface mockOsInterface;
//    EXPECT_CALL(mockUdpInterface, udpRemove(_)).Times(1); // Assume removal happens without error
//    EXPECT_CALL(mockUdpInterface, udpBind(_, _, _)).Times(1).WillOnce(
//            Return(ERR_USE));
//    EXPECT_CALL(mockUdpInterface, udpNew()).Times(1).WillOnce(
//            Return(NON_NULL_PTR_PCB));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, &mockOsInterface);
//    bool success = udpDriverUnderTest.setup();
//    ASSERT_FALSE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderReceiveSuccess) {
//    mocks::MockUdpInterface mockUdpInterface;
//    mocks::MockOsInterface mockOsInterface;
//    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
//            Return((u8_t) 1));
//    EXPECT_CALL(mockOsInterface, OS_xSemaphoreTake(_, _)).Times(1).WillOnce(
//            Return(pdTRUE));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, &mockOsInterface);
//    bool success = udpDriverUnderTest.receive(nullptr);
//    ASSERT_TRUE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderReceiveFailsOnSemaphoreTake) {
//    mocks::MockUdpInterface mockUdpInterface;
//    mocks::MockOsInterface mockOsInterface;
//    EXPECT_CALL(mockOsInterface, OS_xSemaphoreTake(_, _)).Times(1).WillOnce(
//            Return(pdFALSE));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, &mockOsInterface);
//    bool success = udpDriverUnderTest.receive(nullptr);
//    ASSERT_FALSE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderReceiveFailsOnPbufFreeNoneFreed) {
//    mocks::MockUdpInterface mockUdpInterface;
//    mocks::MockOsInterface mockOsInterface;
//    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
//            Return((u8_t) 0));
//    EXPECT_CALL(mockOsInterface, OS_xSemaphoreTake(_, _)).Times(1).WillOnce(
//            Return(pdTRUE));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, &mockOsInterface);
//    bool success = udpDriverUnderTest.receive(nullptr);
//    ASSERT_FALSE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitSuccess) {
//    mocks::MockUdpInterface mockUdpInterface;
//    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
//            Return((u8_t) 1));
//    EXPECT_CALL(mockUdpInterface, udpDisconnect(_)).Times(1);
//    EXPECT_CALL(mockUdpInterface, udpSend(_, _)).Times(1).WillOnce(
//            Return(ERR_OK));
//    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
//            Return(ERR_OK));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, nullptr);
//    bool success = udpDriverUnderTest.transmit(nullptr);
//    ASSERT_TRUE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitFailsOnConnect) {
//    mocks::MockUdpInterface mockUdpInterface;
//    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
//            Return(ERR_VAL)); // use ERR_VAL; anything but ERR_OK works
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, nullptr);
//    bool success = udpDriverUnderTest.transmit(nullptr);
//    ASSERT_FALSE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitFailsOnSend) {
//    mocks::MockUdpInterface mockUdpInterface;
//    EXPECT_CALL(mockUdpInterface, udpSend(_, _)).Times(1).WillOnce(
//            Return(ERR_VAL));
//    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
//            Return(ERR_OK));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, nullptr);
//    bool success = udpDriverUnderTest.transmit(nullptr);
//    ASSERT_FALSE(success);
//}
//
//TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitFailsOnPbufFreeNoneFreed) {
//    mocks::MockUdpInterface mockUdpInterface;
//    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
//            Return((u8_t) 0));
//    EXPECT_CALL(mockUdpInterface, udpDisconnect(_)).Times(1);
//    EXPECT_CALL(mockUdpInterface, udpSend(_, _)).Times(1).WillOnce(
//            Return(ERR_OK));
//    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
//            Return(ERR_OK));
//
//    UdpDriver udpDriverUnderTest(ZERO_IP_ADDR_T, ZERO_IP_ADDR_T, ZERO_U16_T, ZERO_U16_T,
//            &mockUdpInterface, nullptr);
//    bool success = udpDriverUnderTest.transmit(nullptr);
//    ASSERT_FALSE(success);
//}

