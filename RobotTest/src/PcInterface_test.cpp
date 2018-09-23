/**
  *****************************************************************************
  * @file    PcInterface_test.cpp
  * @author  Robert Fairley
  * @brief   PcInterface testing/mocking data structures, tests and mocks.
  *
  * @defgroup pc_interface_tests
  * @brief    The pc_interface_tests module contains the structures required for running tests and mocking, and the tests themselves.
  * @{
  *****************************************************************************
  */

// TODO: add license terms from external projects e.g. googletest to our files too?
// NOTE: consider just doing using namespace pc_interface; ?

#include <cstdint>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <PcInterface.h>
#include <UdpInterface.h>

using ::testing::Return;
using ::testing::_;

// TODO: doxygen documentation for functions once complete

namespace {

class MockUdpInterface : public udp_interface::UdpInterface {
public:
	MOCK_METHOD0(udpNew, bool());
	MOCK_METHOD0(udpBind, bool());
	MOCK_METHOD0(udpRecv, bool());
	MOCK_METHOD0(udpRemove, bool());
	MOCK_METHOD0(ethernetifInput, bool());
	MOCK_METHOD0(udpConnect, bool());
	MOCK_METHOD0(udpSend, bool());
	MOCK_METHOD0(udpDisconnect, bool());
	MOCK_METHOD0(pbufFreeRx, bool());
	MOCK_METHOD0(pbufFreeTx, bool());
	MOCK_METHOD0(waitRecv, bool());
	MOCK_METHOD1(packetToBytes, bool(uint8_t*));
	MOCK_METHOD1(bytesToPacket, bool(const uint8_t*));
};

// MemberProtocolDefaultInitializesToUDP tests that the default
// constructor of PcInterface initializes the member protocol to UDP.
// This is to make sure protocol is initialized to *something*.
TEST(PcInterfaceTests, MemberProtocolDefaultInitializesToUDP) {
	pc_interface::PcInterface pcInterfaceTestObject;
	ASSERT_EQ(pc_interface::UDP, pcInterfaceTestObject.getProtocol());
}

TEST(PcInterfaceTests, MemberUdpInterfaceDefaultInitializesToNull) {
	pc_interface::PcInterface pcInterfaceTestObject;
	ASSERT_EQ(nullptr, pcInterfaceTestObject.getUdpInterface());
}

TEST(PcInterfaceTests, MemberUdpInterfaceParameterizedInitializesToNull) {
	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::UDP);
	ASSERT_EQ(nullptr, pcInterfaceTestObject.getUdpInterface());
}

TEST(PcInterfaceTests, MemberUdpInterfaceCanSetAndGet) {
	pc_interface::PcInterface pcInterfaceTestObject;
	MockUdpInterface udpMockInterface;
	pcInterfaceTestObject.setUdpInterface(&udpMockInterface);
	ASSERT_EQ(&udpMockInterface, pcInterfaceTestObject.getUdpInterface());
}

// MemberProtocolCanInitializeAndGet tests that a constructor can
// initialize and get back the protocol member of PcInterface with
// all of the values defined in type enum Protocol_e.
TEST(PcInterfaceTests, MemberProtocolCanInitializeAndGet) {
	for (int iProtocol = pc_interface::UDP; iProtocol <= pc_interface::USB_UART; iProtocol++) {
		pc_interface::Protocol_e protocol = (pc_interface::Protocol_e)iProtocol;
		pc_interface::PcInterface pcInterfaceTestObject(protocol);

		ASSERT_EQ(protocol, pcInterfaceTestObject.getProtocol());
	}
}

// MemberRxBufferDefaultInitializesToZero tests that the default
// constructor initializes the rxBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberRxBufferDefaultInitializesToZero) {
	pc_interface::PcInterface pcInterfaceTestObject;
	uint8_t rxArray [pc_interface::PC_INTERFACE_BUFFER_SIZE];
	bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
	ASSERT_TRUE(success);

	for (int iRxArray = 0; iRxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE; iRxArray++) {
		ASSERT_EQ(rxArray[iRxArray], 0);
	}
}

// MemberRxBufferParameterizedInitializesToZero tests that the
// parameterized constructor initializes the rxBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberRxBufferParameterizedInitalizesToZero) {
	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::UDP);
	uint8_t rxArray [pc_interface::PC_INTERFACE_BUFFER_SIZE];
	bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
	ASSERT_TRUE(success);

	for (int iRxArray = 0; iRxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE; iRxArray++) {
		ASSERT_EQ(rxArray[iRxArray], 0);
	}
}

// MemberTxBufferDefaultInitializesToZero tests that the default
// constructor initializes the txBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberTxBufferDefaultInitializesToZero) {
	pc_interface::PcInterface pcInterfaceTestObject;
	uint8_t txArray [pc_interface::PC_INTERFACE_BUFFER_SIZE];
	bool success = pc_interface::PcInterfaceTester::getTxBufferDebug(pcInterfaceTestObject, txArray);
	ASSERT_TRUE(success);

	for (int iTxArray = 0; iTxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
		ASSERT_EQ(txArray[iTxArray], 0);
	}
}

// MemberTxBufferParameterizedInitializesToZero tests that the
// parameterized constructor initializes the txBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberTxBufferParameterizedInitalizesToZero) {
	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::UDP);
	uint8_t txArray [pc_interface::PC_INTERFACE_BUFFER_SIZE];
	bool success = pc_interface::PcInterfaceTester::getTxBufferDebug(pcInterfaceTestObject, txArray);
	ASSERT_TRUE(success);

	for (int iTxArray = 0; iTxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
		ASSERT_EQ(txArray[iTxArray], 0);
	}
}

TEST(PcInterfaceTests, MockFunctionCallsUdpSetupSuccess) {
	MockUdpInterface udpMockInterface;
	EXPECT_CALL(udpMockInterface, udpRecv()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(udpMockInterface, udpBind()).Times(1).WillOnce(Return(true));
	EXPECT_CALL(udpMockInterface, udpNew()).Times(1).WillOnce(Return(true));

	pc_interface::PcInterface pcInterfaceTestObject;
	bool success = pcInterfaceTestObject.setUdpInterface(&udpMockInterface);
	ASSERT_TRUE(success);
	success = pcInterfaceTestObject.setup();
	ASSERT_TRUE(success);
}

TEST(PcInterfaceTests, MockFunctionCallsUdpSetupFailOnNew) {
	MockUdpInterface udpMockInterface;
	EXPECT_CALL(udpMockInterface, udpNew()).Times(1).WillOnce(Return(false));

	pc_interface::PcInterface pcInterfaceTestObject;
	bool success = pcInterfaceTestObject.setUdpInterface(&udpMockInterface);
	ASSERT_TRUE(success);
	success = pcInterfaceTestObject.setup();
	ASSERT_FALSE(success);
}

TEST(PcInterfaceTests, MockFunctionCallsUdpSetupFailOnBind) {
	MockUdpInterface udpMockInterface;
	EXPECT_CALL(udpMockInterface, udpRemove()).Times(1).WillOnce(Return(true)); // Assume successful remove call by returning true
	EXPECT_CALL(udpMockInterface, udpBind()).Times(1).WillOnce(Return(false));
	EXPECT_CALL(udpMockInterface, udpNew()).Times(1).WillOnce(Return(true));

	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::UDP);
	bool success = pcInterfaceTestObject.setUdpInterface(&udpMockInterface);
	ASSERT_TRUE(success);
	success = pcInterfaceTestObject.setup();
	ASSERT_FALSE(success);
}

TEST(PcInterfaceTests, MockFunctionCallsUdpReceive) {
	MockUdpInterface udpMockInterface;
	EXPECT_CALL(udpMockInterface, pbufFreeRx()).Times(1);
	EXPECT_CALL(udpMockInterface, packetToBytes(_)).Times(1);
	EXPECT_CALL(udpMockInterface, waitRecv()).Times(1);
	EXPECT_CALL(udpMockInterface, ethernetifInput()).Times(1);

	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::UDP);
	bool success = pcInterfaceTestObject.setUdpInterface(&udpMockInterface);
	ASSERT_TRUE(success);
	success = pcInterfaceTestObject.receive();
	ASSERT_TRUE(success);
}

TEST(PcInterfaceTests, MockFunctionCallsUdpTransmit) {
	MockUdpInterface udpMockInterface;
	EXPECT_CALL(udpMockInterface, pbufFreeTx()).Times(1);
	EXPECT_CALL(udpMockInterface, udpDisconnect()).Times(1);
	EXPECT_CALL(udpMockInterface, udpSend()).Times(1);
	EXPECT_CALL(udpMockInterface, udpConnect()).Times(1);
	EXPECT_CALL(udpMockInterface, bytesToPacket(_)).Times(1);

	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::UDP);
	bool success = pcInterfaceTestObject.setUdpInterface(&udpMockInterface);
	ASSERT_TRUE(success);
	success = pcInterfaceTestObject.transmit();
	ASSERT_TRUE(success);
}

TEST(PcInterfaceTests, MemberFunctionSetUdpInterfaceFailsOnNull) {
	pc_interface::PcInterface pcInterfaceTestObject;
	ASSERT_FALSE(pcInterfaceTestObject.setUdpInterface(nullptr));
}

TEST(PcInterfaceTests, MemberFunctionSetUdpInterfaceFailsOnNotUDP) {
	pc_interface::PcInterface pcInterfaceTestObject(pc_interface::USB_UART);
	MockUdpInterface udpMockInterface;
	ASSERT_FALSE(pcInterfaceTestObject.setUdpInterface(&udpMockInterface));
}

// TODO: tests for failing on everything - all possible combinations?
// TODO: add a test for setting TxBuffer and getting from RxBuffer. (use a test socket to set up the data)
// TODO: test coverage for LwipUdpInterface, set up fixtures to test passing
//       data between rx/txBuffers and pbufs

}

/**
 * @}
 */
/* end - pc_interface_tests */
