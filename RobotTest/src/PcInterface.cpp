#include <PcInterface.h>
#include <UdpInterface.h>

// LwipUdpInterface is the real i.e. hardware-facing implementation of the
// abstract UdpInterface class. This will call out to LwIP and any
// lower-level networking functions to implement UDP functionality.
// For now, it is never used outside of PcInterface, so the class
// definition and implementation live in PcInterface.cpp.
// Functions have no parameters, and instead pass the state information
// they were initialized with (members of this class). This means the PcInterface,
// UdpInterface, and mocking classes do not have to care about how the
// networking functions are implemented under the hood - only the methods
// called.
class LwipUdpInterface : public UdpInterface {
public:
	LwipUdpInterface();
	// TODO: parameterized constructor.
	~LwipUdpInterface();
	bool udpNew();
	bool udpBind();
	bool udpRecv();
	bool udpRemove();
	bool ethernetifInput();
	bool udpConnect();
	bool udpSend();
	bool udpDisconnect();
	bool pbufFree();
private:
	// FIXME: These will be of the right types for the networking stack.
	const int *pcb = nullptr;
	const int *ipaddr = nullptr;
	const int port = 0;
	const int *recvCallback = nullptr;
	const int *recvArgs = nullptr;
	const int *netif = nullptr;
};

LwipUdpInterface::LwipUdpInterface() {

}

LwipUdpInterface::~LwipUdpInterface() {

}

bool LwipUdpInterface::udpNew() {
	return true;
}

bool LwipUdpInterface::udpBind() {
	return true;
}

bool LwipUdpInterface::udpRecv() {
	return true;
}

bool LwipUdpInterface::udpRemove() {
	return true;
}

bool LwipUdpInterface::ethernetifInput() {
	return true;
}

bool LwipUdpInterface::udpConnect() {
	return true;
}

bool LwipUdpInterface::udpSend() {
	return true;
}

bool LwipUdpInterface::udpDisconnect() {
	return true;
}

bool LwipUdpInterface::pbufFree() {
	return true;
}

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
