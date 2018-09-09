#include <PcInterface.h>
#include <UdpInterface.h>

// LwipUdpInterface is the real i.e. hardware-facing implementation of the
// abstract UdpInterface class. This will call out to LwIP and any
// lower-level networking functions to implement UDP functionality.
// For now, it is never used outside of PcInterface, so the class
// definition and implementation live in PcInterface.cpp.
class LwipUdpInterface : public UdpInterface {
	LwipUdpInterface();
	~LwipUdpInterface();
	void* udpNew();
	int8_t udpBind(void *pcb, void *ipaddr, uint16_t port);
	void udpRecv(void *pcb, void *recv, void *recvArgs);
	void udpRemove(void *pcb);
	void ethernetifInput(void *netif);
	int8_t udpConnect(void *pcb, void *ipaddr, uint16_t port);
	int8_t udpSend(void *pcb, void *pbuf);
	void udpDisconnect(void *pcb);
	int8_t pbufFree(void *pbuf);
};

LwipUdpInterface::LwipUdpInterface() {

}

LwipUdpInterface::~LwipUdpInterface() {

}

void* LwipUdpInterface::udpNew() {
	return nullptr;
}

int8_t LwipUdpInterface::udpBind(void *pcb, void *ipaddr, uint16_t port) {
	return 0;
}

void LwipUdpInterface::udpRecv(void *pcb, void *recv, void *recvArgs) {

}

void LwipUdpInterface::udpRemove(void *pcb) {

}

void LwipUdpInterface::ethernetifInput(void *netif) {

}

int8_t LwipUdpInterface::udpConnect(void *pcb, void *ipaddr, uint16_t port) {
	return 0;
}

int8_t LwipUdpInterface::udpSend(void *pcb, void *pbuf) {
	return 0;
}

void LwipUdpInterface::udpDisconnect(void *pcb) {

}

int8_t LwipUdpInterface::pbufFree(void *pbuf) {
	return 0;
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
