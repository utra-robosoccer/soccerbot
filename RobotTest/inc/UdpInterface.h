/**
  *****************************************************************************
  * @file    UdpInterface.h
  * @author  Robert Fairley
  * @brief   Defines an abstract interface of UDP networking functions, to be implemented and extended as needed, e.g. by hardware-facing classes, test frameworks.
  *
  * @defgroup Header
  * @{
  *****************************************************************************
  */

// NOTE: defgroup used above since there is no .cpp file associated with this class.

#ifndef __UDP_INTERFACE_H__
#define __UDP_INTERFACE_H__

#include <cstdint>

namespace udp_interface {

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
	virtual bool pbufFreeRx() = 0;
	virtual bool pbufFreeTx() = 0;
	virtual bool waitRecv() = 0;
	virtual bool packetToBytes(uint8_t *_byteArray) = 0;
	virtual bool bytesToPacket(const uint8_t *_byteArray) = 0;
};

} // end namespace udp_interface

/**
 * @}
 */
/* end - Header */

#endif /* __UDP_INTERFACE_H__ */
