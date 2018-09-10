#include <PcInterface.h>

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
	if (getProtocol() != UDP) {
		return false;
	}
	if (!_udpInterface) {
		return false;
	}
	udpInterface = _udpInterface;
	return true;
}

UdpInterface* PcInterface::getUdpInterface() const {
	return udpInterface;
}
