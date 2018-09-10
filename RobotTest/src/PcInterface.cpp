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
// Another approach rather than having stateful members is to pass a
// void* to every interface function, and pass data as needed into there.
class LwipUdpInterface : public UdpInterface {
public:
	LwipUdpInterface();
	// TODO: parameterized constructor.
	~LwipUdpInterface();

	// Abstract UdpInterface class overrides.
	bool udpNew();
	bool udpBind();
	bool udpRecv();
	bool udpRemove();
	bool ethernetifInput();
	bool udpConnect();
	bool udpSend();
	bool udpDisconnect();
	bool pbufFreeRx();
	bool pbufFreeTx();
	bool waitRecv();
	bool packetToBytes(uint8_t *_byteArray);
	bool bytesToPacket(const uint8_t *_byteArray);

	// Extra helper functions
	int* getRecvCallbackArg() const;
	int* getRecvCallbackPcb() const;
	int* getRecvCallbackPbuf() const;
	int* getRecvCallbackAddr() const;
	int getRecvCallbackPort() const;

	// FIXME: use correct networking stack types in callback function.
	friend void recvCallback(void *arg, void *pcb, void *p,
			const void *addr, int port);
private:
	// FIXME: These need to be of the right types for the networking stack.
	const int *pcb = nullptr;
	const int *ipaddr = nullptr;
	const int *ipaddrPc = nullptr;
	const int port = 0;
	const int portPc = 0;
	const int *netif = nullptr;
	const int *recvSemaphore = nullptr;

	// Set by recvCallback.
	static int *recvCallbackArg = nullptr;
	static int *recvCallbackPcb = nullptr;
	static int *recvCallbackPbuf = nullptr;
	static int *recvCallbackAddr = nullptr;
	static int recvCallbackPort = 0;

	int *txPbuf = nullptr;
	// TODO: synchronize access to pbufs
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

bool LwipUdpInterface::pbufFreeRx() {
	return true;
}

bool LwipUdpInterface::pbufFreeTx() {
	return true;
}

bool LwipUdpInterface::waitRecv() {
	return true;
}

bool LwipUdpInterface::packetToBytes(uint8_t *_byteArray) {
	return true;
}

bool LwipUdpInterface::bytesToPacket(const uint8_t *_byteArray) {
	return true;
}

int* LwipUdpInterface::getRecvCallbackArg() const {
	return recvCallbackArg;
}

int* LwipUdpInterface::getRecvCallbackPcb() const {
	return recvCallbackPcb;
}

int* LwipUdpInterface::getRecvCallbackPbuf() const {
	return recvCallbackPbuf;
}

int* LwipUdpInterface::getRecvCallbackAddr() const {
	return recvCallbackAddr;
}

int LwipUdpInterface::getRecvCallbackPort() const {
	return recvCallbackPort;
}

namespace {
	void recvCallback(void *arg, void *pcb, void *p,
		    const void *addr, int port) {
		// TODO: synchronize access to these members
		LwipUdpInterface::recvCallbackArg = arg;
		LwipUdpInterface::recvCallbackPcb = pcb;
		LwipUdpInterface::recvCallbackPbuf = p;
		LwipUdpInterface::recvCallbackAddr = addr;
		LwipUdpInterface::recvCallbackPort = port;
		// TODO: release recvSemaphore
	}
}

PcInterface::PcInterface() {
	// No need to initialize any members here; this is done in PcInterface.h
}

PcInterface::PcInterface(Protocol_e _protocol) : protocol(_protocol) {
	// No need to initialize any members here; this is done in PcInterface.h
}

PcInterface::~PcInterface() {

}

// NOTE: this is a good place where err_t (which is set to integer values
// for each error) might be handy, to tell what kind of error happened.
bool PcInterface::setup() {
	bool success = false;
	switch(protocol) {
	case UDP:
		if (!getUdpInterface()) {
			return false;
		}
		success = udpInterface->udpNew();
		if (success) {
			success = udpInterface->udpBind();
			if (success) {
				udpInterface->udpRecv();
			}
			else {
				udpInterface->udpRemove();
			}
		}
		break;
	default:
		break;
	}
	return success;
}

// NOTE: Consider an input() function that calls only ethernetifInput, which runs
// in its own thread.

// Purpose: to make the HW calls and convert packets into array of bytes.
bool PcInterface::receive() {
	switch(protocol) {
	case UDP:
		if (!getUdpInterface()) {
			return false;
		}
		udpInterface->ethernetifInput(); // Extract packets from network interface
		udpInterface->waitRecv(); // Wait for callback to write data members (including packets) to UdpInterface
		udpInterface->packetToBytes(rxBuffer); // Copy contents of the packets into rxBuffer
		udpInterface->pbufFreeRx();
		return true;
	default:
		return false;
	}
}

// Purpose: to covert array of bytes into packets and make the HW calls.
bool PcInterface::transmit() {
	switch(protocol) {
	case UDP:
		if (!getUdpInterface()) {
			return false;
		}
		udpInterface->bytesToPacket(txBuffer);
		udpInterface->udpConnect();
		udpInterface->udpSend();
		udpInterface->udpDisconnect();
		udpInterface->pbufFreeTx();
		// Should not return.
		return true;
	default:
		return false;
	}
}

// NOTE: Consider using <array> to allow arrays of variable length < PC_INTERFACE_BUFFER_SIZE to be input.

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

bool PcInterface::setUdpInterface(UdpInterface *_udpInterface) {
	if (!_udpInterface) {
		return false;
	}
	udpInterface = _udpInterface;
	return true;
}

UdpInterface* PcInterface::getUdpInterface() const {
	return udpInterface;
}
