#ifndef UDP_INTERFACE_H
#define UDP_INTERFACE_H

#include <cstdint>

// Interface the application with framework-dependent UDP networking code.
// Extend this interface as the need arises with virtual functions
// and generic parameters. Then, implement the UDP code
// in a child class of this interface.
// Necessary in order to make the application classes mockable
// (gmock hooks onto this).
class UdpInterface {
public:
	virtual ~UdpInterface() {}
	virtual void* udpNew() = 0;
	virtual int8_t udpBind(void *pcb, void *ipaddr, uint16_t port) = 0;
	virtual void udpRecv(void *pcb, void *recv, void *recvArgs) = 0;
	virtual void udpRemove(void *pcb) = 0;
	virtual void ethernetifInput(void *netif) = 0;
	virtual int8_t udpConnect(void *pcb, void *ipaddr, uint16_t port) = 0;
	virtual int8_t udpSend(void *pcb, void *pbuf) = 0;
	virtual void udpDisconnect(void *pcb) = 0;
	virtual int8_t pbufFree(void *pbuf) = 0;
};

#endif
