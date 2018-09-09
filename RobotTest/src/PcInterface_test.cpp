#include <PcInterface.h>
#include <UdpInterface.h>

// PcInterfaceTester contains testing functions that need access to private members
// but are not to be used under normal circumstances.
class PcInterfaceTester {
public:
	static bool setRxBufferDebug(PcInterface &pcInterfaceUnderTest, uint8_t *_rxArray);
	static bool getTxBufferDebug(PcInterface &pcInterfaceUnderTest, uint8_t *_txArray);
};

bool PcInterfaceTester::setRxBufferDebug(PcInterface &pcInterfaceUnderTest, uint8_t *_rxArray) {
	for (int iRxArray = 0; iRxArray < PC_INTERFACE_BUFFER_SIZE; iRxArray++) {
		pcInterfaceUnderTest.rxBuffer[iRxArray] = _rxArray[iRxArray];
	}
	return true;
}

bool PcInterfaceTester::getTxBufferDebug(PcInterface &pcInterfaceUnderTest, uint8_t *_txArray) {
	for (int iTxBuffer = 0; iTxBuffer < PC_INTERFACE_BUFFER_SIZE; iTxBuffer++) {
		_txArray[iTxBuffer] = pcInterfaceUnderTest.txBuffer[iTxBuffer];
	}
	return true;
}

///// GTEST/GMOCK /////
#include <gtest/gtest.h>
#include <gmock/gmock.h>

class MockUdpInterface : public UdpInterface {
public:
	MOCK_METHOD0(udpNew, void*());
	MOCK_METHOD3(udpBind, int8_t(void*, void*, uint16_t));
	MOCK_METHOD3(udpRecv, void(void*, void*, void*));
	MOCK_METHOD1(udpRemove, void(void*));
	MOCK_METHOD1(ethernetifInput, void(void*));
	MOCK_METHOD3(udpConnect, int8_t(void*, void*, uint16_t));
	MOCK_METHOD2(udpSend, int8_t(void*, void*));
	MOCK_METHOD1(udpDisconnect, void(void*));
	MOCK_METHOD1(pbufFree, int8_t(void*));
};

// MemberProtocolDefaultInitializesToUDP tests that the default
// constructor of PcInterface initializes the member protocol to UDP.
// This is to make sure protocol is initialized to *something*.
TEST(PcInterfaceTests, MemberProtocolDefaultInitializesToUDP) {
	PcInterface pcInterfaceTestObject;
	ASSERT_EQ(UDP, pcInterfaceTestObject.getProtocol());
}

// MemberProtocolCanInitializeAndGet tests that a constructor can
// initialize and get back the protocol member of PcInterface with
// all of the values defined in type enum Protocol_e.
TEST(PcInterfaceTests, MemberProtocolCanInitializeAndGet) {
	for (int iProtocol = UDP; iProtocol <= USB_UART; iProtocol++) {
		Protocol_e protocol = (Protocol_e)iProtocol;
		PcInterface pcInterfaceTestObject(protocol);

		ASSERT_EQ(protocol, pcInterfaceTestObject.getProtocol());
	}
}

// MemberRxBufferDefaultInitializesToZero tests that the default
// constructor initializes the rxBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberRxBufferDefaultInitializesToZero) {
	PcInterface pcInterfaceTestObject;
	uint8_t rxArray [PC_INTERFACE_BUFFER_SIZE];
	bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
	ASSERT_TRUE(success);

	for (int iRxArray = 0; iRxArray < PC_INTERFACE_BUFFER_SIZE; iRxArray++) {
		ASSERT_EQ(rxArray[iRxArray], 0);
	}
}

// MemberRxBufferParameterizedInitializesToZero tests that the
// parameterized constructor initializes the rxBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberRxBufferParameterizedInitalizesToZero) {
	PcInterface pcInterfaceTestObject(UDP);
	uint8_t rxArray [PC_INTERFACE_BUFFER_SIZE];
	bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
	ASSERT_TRUE(success);

	for (int iRxArray = 0; iRxArray < PC_INTERFACE_BUFFER_SIZE; iRxArray++) {
		ASSERT_EQ(rxArray[iRxArray], 0);
	}
}

// MemberTxBufferDefaultInitializesToZero tests that the default
// constructor initializes the txBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberTxBufferDefaultInitializesToZero) {
	PcInterface pcInterfaceTestObject;
	uint8_t txArray [PC_INTERFACE_BUFFER_SIZE];
	bool success = PcInterfaceTester::getTxBufferDebug(pcInterfaceTestObject, txArray);
	ASSERT_TRUE(success);

	for (int iTxArray = 0; iTxArray < PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
		ASSERT_EQ(txArray[iTxArray], 0);
	}
}

// MemberTxBufferParameterizedInitializesToZero tests that the
// parameterized constructor initializes the txBuffer entries to zeroes.
TEST(PcInterfaceTests, MemberTxBufferParameterizedInitalizesToZero) {
	PcInterface pcInterfaceTestObject(UDP);
	uint8_t txArray [PC_INTERFACE_BUFFER_SIZE];
	bool success = PcInterfaceTester::getTxBufferDebug(pcInterfaceTestObject, txArray);
	ASSERT_TRUE(success);

	for (int iTxArray = 0; iTxArray < PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
		ASSERT_EQ(txArray[iTxArray], 0);
	}
}

// TODO: add a test for setting TxBuffer and getting from RxBuffer. (use a test socket to set up the data)
