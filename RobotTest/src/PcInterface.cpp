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

// NOTE: Consider a setRxBuffer and getTxBuffer for debugging/testing purposes - friend functions?
// NOTE: Consider using STL to allow arrays of variable length < PC_INTERFACE_BUFFER_SIZE to be input.

// getRxBuffer deep copies all elements, out of rxBuffer to _rxArray.
// _rxArray must have length == PC_INTERFACE_BUFFER_SIZE.
bool PcInterface::getRxBuffer(uint8_t *_rxArray) const {
	// FIXME: assert length of _rxArray
	for (int iRxBuffer = 0; iRxBuffer < PC_INTERFACE_BUFFER_SIZE; iRxBuffer++) {
		_rxArray[iRxBuffer] = rxBuffer[iRxBuffer];
	}
	return true;
}

// setTxBuffer deep copies all elements, into txBuffer from _txArray.
// _txArray must have length == PC_INTERFACE_BUFFER_SIZE.
bool PcInterface::setTxBuffer(const uint8_t *_txArray) {
	// FIXME: assert length of _txArray
	for (int iTxArray = 0; iTxArray < PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
		txBuffer[iTxArray] = _txArray[iTxArray];
	}
	return true;
}

Protocol_e PcInterface::getProtocol() {
	return protocol;
}

///// GMOCK /////
#include <gtest/gtest.h>

// MemberProtocolDefaultInitializesToUDP tests that the default
// constructor of PcInterface initializes the member protocol to UDP.
// This is to make sure protocol is initialized to *something*.
TEST(PcInterfaceTests, MemberProtocolDefaultInitializesToUDP) {
	PcInterface pcInterfaceTestObject;
	ASSERT_EQ(UDP, pcInterfaceTestObject.getProtocol());
}

// CanInitializeAndGetMemberProtocol tests that a constructor can
// initialize and get back the protocol member of PcInterface with
// all of the values defined in type enum Protocol_e.
TEST(PcInterfaceTests, CanInitializeAndGetMemberProtocol) {
	for (int iProtocol = UDP; iProtocol <= USB_UART; iProtocol++) {
		Protocol_e protocol = (Protocol_e)iProtocol;
		PcInterface pcInterfaceTestObject(protocol);

		ASSERT_EQ(protocol, pcInterfaceTestObject.getProtocol());
	}
}

// RxBufferMemberDefaultInitializesToZero tests that the default
// constructor initializes the rxArray entries to zeroes.
TEST(PcInterfaceTests, RxBufferMemberDefaultInitializesToZero) {
	PcInterface pcInterfaceTestObject;
	uint8_t rxArray [PC_INTERFACE_BUFFER_SIZE];
	bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
	ASSERT_TRUE(success);

	for (int iRxArray = 0; iRxArray < PC_INTERFACE_BUFFER_SIZE; iRxArray++) {
		ASSERT_EQ(rxArray[iRxArray], 0);
	}
}

// TODO: add a test for setting TxBuffer.
// TODO: test for parameterized constructor initializing to zero.
