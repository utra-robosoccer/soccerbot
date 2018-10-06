
#include <MockFreeRTOSInterface.h>
#include <MockUdpInterface.h>
#include <UdpDriver.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

// TODO: should move the consts to defines. Only works as const because they are literal type.
constexpr u16_t ZERO_U16_T = 0;
const ip_addr_t *TEST_IP_ADDR = (ip_addr_t *) 0xC0A80008;
const ip_addr_t *TEST_IP_ADDR_PC = (ip_addr_t *) 0xC0A80002;
constexpr u16_t TEST_PORT = 7;
constexpr u16_t TEST_PORT_PC = 6340;
const udp_interface::UdpInterface *NON_NULL_PTR_UDP_INTERFACE =
        (udp_interface::UdpInterface *) 0x71;
const FreeRTOS_Interface::FreeRTOSInterface *NON_NULL_PTR_FREERTOS_INTERFACE =
        (FreeRTOS_Interface::FreeRTOSInterface *) 0x72;
struct udp_pcb *NON_NULL_PTR_PCB = (struct udp_pcb *) 0x51;
struct pbuf_t *NON_NULL_PTR_PBUF = (struct pbuf_t *) 0x61;

using ::testing::Return;
using ::testing::_;

using namespace udp_driver;

TEST(UdpDriverTests, IpAddrDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getIpaddr(), nullptr);
}

TEST(UdpDriverTests, IpAddrPcDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getIpaddrPc(), nullptr);
}

TEST(UdpDriverTests, PortDefaultInitializesToZero) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getPort(), ZERO_U16_T);
}

TEST(UdpDriverTests, PortPcDefaultInitializesToZero) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getPortPc(), ZERO_U16_T);
}

TEST(UdpDriverTests, UdpInterfaceDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getUdpInterface(), nullptr);
}

TEST(UdpDriverTests, OsInterfaceDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getOsInterface(), nullptr);
}

TEST(UdpDriverTests, PcbDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getPcb(), nullptr);
}

TEST(UdpDriverTests, RxPbufDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getRxPbuf(), nullptr);
}

TEST(UdpDriverTests, TxPbufDefaultInitializesToNull) {
    udp_driver::UdpDriver udpDriverUnderTest;
    ASSERT_EQ(udpDriverUnderTest.getTxPbuf(), nullptr);
}

TEST(UdpDriverTests, RxPbufDefaultInitializesToNullThreaded) {
    MOCKS::MockFreeRTOSInterface mockOsInterface;
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T,
            ZERO_U16_T, nullptr, &mockOsInterface);
    ASSERT_EQ(udpDriverUnderTest.getRxPbufThreaded(), nullptr);
}

TEST(UdpDriverTests, TxPbufDefaultInitializesToNullThreaded) {
    MOCKS::MockFreeRTOSInterface mockOsInterface;
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T,
            ZERO_U16_T, nullptr, &mockOsInterface);
    ASSERT_EQ(udpDriverUnderTest.getTxPbufThreaded(), nullptr);
}

TEST(UdpDriverTests, CanInitializeIpaddrWithParameterizedConstructor) {
    udp_driver::UdpDriver udpDriverUnderTest(TEST_IP_ADDR, nullptr, ZERO_U16_T,
            ZERO_U16_T, nullptr, nullptr);
    ASSERT_EQ(TEST_IP_ADDR, udpDriverUnderTest.getIpaddr());
}

TEST(UdpDriverTests, CanInitializeIpaddrPcWithParameterizedConstructor) {
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, TEST_IP_ADDR_PC,
            ZERO_U16_T, ZERO_U16_T, nullptr, nullptr);
    ASSERT_EQ(TEST_IP_ADDR_PC, udpDriverUnderTest.getIpaddrPc());
}

TEST(UdpDriverTests, CanInitializePortWithParameterizedConstructor) {
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, nullptr, TEST_PORT,
            ZERO_U16_T, nullptr, nullptr);
    ASSERT_EQ(TEST_PORT, udpDriverUnderTest.getPort());
}

TEST(UdpDriverTests, CanInitializePortPcWithParameterizedConstructor) {
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T,
            TEST_PORT_PC, nullptr, nullptr);
    ASSERT_EQ(TEST_PORT_PC, udpDriverUnderTest.getPortPc());
}

TEST(UdpDriverTests, CanInitializeUdpInterfaceWithParameterizedConstructor) {
    MOCKS::MockUdpInterface mockUdpInterface;
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T,
            ZERO_U16_T, &mockUdpInterface, nullptr);
    ASSERT_EQ(udpDriverUnderTest.getUdpInterface(), &mockUdpInterface);
}

TEST(UdpDriverTests, CanInitializeOsInterfaceWithParameterizedConstructor) {
    MOCKS::MockFreeRTOSInterface mockOsInterface;
    udp_driver::UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T,
            ZERO_U16_T, nullptr, &mockOsInterface);
    ASSERT_EQ(udpDriverUnderTest.getOsInterface(), &mockOsInterface);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderSetupSuccess) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, udpRecv(_, _, _)).Times(1);
    EXPECT_CALL(mockUdpInterface, udpBind(_, _, _)).Times(1).WillOnce(
            Return(ERR_OK));
    EXPECT_CALL(mockUdpInterface, udpNew()).Times(1).WillOnce(
            Return(NON_NULL_PTR_PCB));

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.setup();
    ASSERT_TRUE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderSetupFailsOnNew) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, udpNew()).Times(1).WillOnce(Return(nullptr));

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.setup();
    ASSERT_FALSE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderSetupFailsOnBind) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, udpRemove(_)).Times(1); // Assume removal happens without error
    EXPECT_CALL(mockUdpInterface, udpBind(_, _, _)).Times(1).WillOnce(
            Return(ERR_USE));
    EXPECT_CALL(mockUdpInterface, udpNew()).Times(1).WillOnce(
            Return(NON_NULL_PTR_PCB));

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.setup();
    ASSERT_FALSE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderReceiveSuccess) {
    MOCKS::MockUdpInterface mockUdpInterface;
    MOCKS::MockFreeRTOSInterface mockOsInterface;
    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
            Return((u8_t) 1));
    EXPECT_CALL(mockOsInterface, OS_xSemaphoreTake(_, _)).Times(1).WillOnce(
            Return(pdTRUE));
    EXPECT_CALL(mockUdpInterface, ethernetifInput(_)).Times(1);

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, &mockOsInterface);
    bool success = udpDriverUnderTest.receive(nullptr);
    ASSERT_TRUE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderReceiveFailsOnSemaphoreTake) {
    MOCKS::MockUdpInterface mockUdpInterface;
    MOCKS::MockFreeRTOSInterface mockOsInterface;
    EXPECT_CALL(mockOsInterface, OS_xSemaphoreTake(_, _)).Times(1).WillOnce(
            Return(pdFALSE));
    EXPECT_CALL(mockUdpInterface, ethernetifInput(_)).Times(1);

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, &mockOsInterface);
    bool success = udpDriverUnderTest.receive(nullptr);
    ASSERT_FALSE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderReceiveFailsOnPbufFreeNoneFreed) {
    MOCKS::MockUdpInterface mockUdpInterface;
    MOCKS::MockFreeRTOSInterface mockOsInterface;
    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
            Return((u8_t) 0));
    EXPECT_CALL(mockOsInterface, OS_xSemaphoreTake(_, _)).Times(1).WillOnce(
            Return(pdTRUE));
    EXPECT_CALL(mockUdpInterface, ethernetifInput(_)).Times(1);

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, &mockOsInterface);
    bool success = udpDriverUnderTest.receive(nullptr);
    ASSERT_FALSE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitSuccess) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
            Return((u8_t) 1));
    EXPECT_CALL(mockUdpInterface, udpDisconnect(_)).Times(1);
    EXPECT_CALL(mockUdpInterface, udpSend(_, _)).Times(1).WillOnce(
            Return(ERR_OK));
    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
            Return(ERR_OK));

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.transmit(nullptr);
    ASSERT_TRUE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitFailsOnConnect) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
            Return(ERR_VAL)); // use ERR_VAL; anything but ERR_OK works

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.transmit(nullptr);
    ASSERT_FALSE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitFailsOnSend) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, udpSend(_, _)).Times(1).WillOnce(
            Return(ERR_VAL));
    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
            Return(ERR_OK));

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.transmit(nullptr);
    ASSERT_FALSE(success);
}

TEST(UdpDriverTests, FunctionCallsCorrectOrderTransmitFailsOnPbufFreeNoneFreed) {
    MOCKS::MockUdpInterface mockUdpInterface;
    EXPECT_CALL(mockUdpInterface, pbufFree(_)).Times(1).WillOnce(
            Return((u8_t) 0));
    EXPECT_CALL(mockUdpInterface, udpDisconnect(_)).Times(1);
    EXPECT_CALL(mockUdpInterface, udpSend(_, _)).Times(1).WillOnce(
            Return(ERR_OK));
    EXPECT_CALL(mockUdpInterface, udpConnect(_, _, _)).Times(1).WillOnce(
            Return(ERR_OK));

    UdpDriver udpDriverUnderTest(nullptr, nullptr, ZERO_U16_T, ZERO_U16_T,
            &mockUdpInterface, nullptr);
    bool success = udpDriverUnderTest.transmit(nullptr);
    ASSERT_FALSE(success);
}

// TODO: tests that vary the arguments passed in. test for non null arguments?
