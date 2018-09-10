#include <LwipUdpInterface.h>

// TODO: implement these functions.
// Most will simply be a 1-line call out to lwIP library,
// passing in data members defined in the LwipUdpInterface class
// as parameters.

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

void LwipUdpInterface::setRecvCallbackPbuf(void* _recvCallbackPbuf) {
	recvCallbackPbuf = _recvCallbackPbuf;
}

namespace {
	void recvCallback(void *arg, void *pcb, void *p,
			const void *addr, int port) {
		// TODO: synchronize access to these members (stop them getting clobbered before read by PcInterface)
		LwipUdpInterface *lwipUdpInterfaceCaller = (LwipUdpInterface*)arg;
		lwipUdpInterfaceCaller->setRecvCallbackPbuf(p);
		// TODO: release recvSemaphore
	}
}
