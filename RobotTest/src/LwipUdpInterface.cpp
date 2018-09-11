/**
  *****************************************************************************
  * @file    LwipUdpInterface.cpp
  * @author  Robert Fairley
  * @brief   Implements Lwip/UDP interface-facing functions, and means to access network stack data.
  *
  * @defgroup lwip_udp_interface
  * @brief    The lwip_udp_interface module provides an abstraction layer to the LwIP network stack, so that network parameters and actions are hidden from other components, and means to access only necessary data is provided.
  * @{
  *****************************************************************************
  */


#include <LwipUdpInterface.h>

// TODO: implement these functions.
// Most will simply be a 1-line call out to lwIP library,
// passing in data members defined in the LwipUdpInterface class
// as parameters.

// TODO: doxygen documentation for functions once complete

// FIXME: use correct networking stack types in callback function.
void recvCallback(void *arg, void *pcb, void *p,
		const void *addr, int port);

lwip_udp_interface::LwipUdpInterface::LwipUdpInterface() {

}

lwip_udp_interface::LwipUdpInterface::~LwipUdpInterface() {

}

bool lwip_udp_interface::LwipUdpInterface::udpNew() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::udpBind() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::udpRecv() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::udpRemove() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::ethernetifInput() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::udpConnect() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::udpSend() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::udpDisconnect() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::pbufFreeRx() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::pbufFreeTx() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::waitRecv() {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::packetToBytes(uint8_t *_byteArray) {
	return true;
}

bool lwip_udp_interface::LwipUdpInterface::bytesToPacket(const uint8_t *_byteArray) {
	return true;
}

void lwip_udp_interface::LwipUdpInterface::setRecvCallbackPbuf(void* _recvCallbackPbuf) {
	recvCallbackPbuf = _recvCallbackPbuf;
}

// Note: not surrounding this in namespace in order to
// make available to member functions above.

void recvCallback(void *arg, void *pcb, void *p,
		const void *addr, int port) {
	// TODO: synchronize access to these members (stop them getting clobbered before read by PcInterface)
	lwip_udp_interface::LwipUdpInterface *lwipUdpInterfaceCaller = (lwip_udp_interface::LwipUdpInterface*)arg;
	lwipUdpInterfaceCaller->setRecvCallbackPbuf(p);
	// TODO: release recvSemaphore
}

/**
 * @}
 */
/* end - lwip_udp_interface */
