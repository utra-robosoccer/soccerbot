#ifndef UDP_INTERFACE_H
#define UDP_INTERFACE_H

#include <cstdint>

// Interface the application with framework-dependent UDP networking code.
// Extend this interface as the need arises with virtual functions
// and generic parameters. Then, implement the UDP code
// in a child class of this interface.
// Necessary in order to make the application classes mockable
// (gmock hooks onto this).
// Return types are bool to report success/failure with true/false
// respectively, but could be replaced by some generic error type
// err_t.
class UdpInterface {
public:
	virtual ~UdpInterface() {}
	virtual bool udpNew() = 0;
	virtual bool udpBind() = 0;
	virtual bool udpRecv() = 0;
	virtual bool udpRemove() = 0;
	virtual bool ethernetifInput() = 0;
	virtual bool udpConnect() = 0;
	virtual bool udpSend() = 0;
	virtual bool udpDisconnect() = 0;
	virtual bool pbufFree() = 0;
};

#endif
