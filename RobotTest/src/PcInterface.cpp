#include <PcInterface.h>

PcInterface::PcInterface() {
	// No need to initialize any members here; this is done in PcInterface.h
}

PcInterface::PcInterface(Protocol_e _protocol) : protocol(_protocol) {
	// No need to initialize any members here; this is done in PcInterface.h
}

PcInterface::~PcInterface() {

}

bool PcInterface::setup() {
	// TODO: implement me
}

bool PcInterface::receive() {
	// TODO: implement me
}

bool PcInterface::transmit() {
	// TODO: implement me
}

// NOTE: Consider using STL to allow arrays of variable length < PC_INTERFACE_BUFFER_SIZE to be input.

// getRxBuffer deep copies all elements, out of rxBuffer to _rxArray.
// _rxArray must have length == PC_INTERFACE_BUFFER_SIZE.
bool PcInterface::getRxBuffer(uint8_t *_rxArray) const {
	// FIXME: return false if length of _rxArray - once <array> is used
	for (int iRxBuffer = 0; iRxBuffer < PC_INTERFACE_BUFFER_SIZE; iRxBuffer++) {
		_rxArray[iRxBuffer] = rxBuffer[iRxBuffer];
	}
	return true;
}

// setTxBuffer deep copies all elements, into txBuffer from _txArray.
// _txArray must have length == PC_INTERFACE_BUFFER_SIZE.
bool PcInterface::setTxBuffer(const uint8_t *_txArray) {
	// FIXME: return false if length of _txArray - once <array> is used
	for (int iTxArray = 0; iTxArray < PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
		txBuffer[iTxArray] = _txArray[iTxArray];
	}
	return true;
}

Protocol_e PcInterface::getProtocol() {
	return protocol;
}

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
