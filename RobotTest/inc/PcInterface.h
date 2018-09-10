#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <cstdint>

#include <UdpInterface.h>

#define PC_INTERFACE_BUFFER_SIZE 1024
#define PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC 5656

enum Protocol_e {
	UDP = PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC,
	USB_UART
};

class PcInterface {
public:
	PcInterface();
	PcInterface(Protocol_e _protocol);
	~PcInterface();
	bool setup();
	bool receive();
	bool transmit();
	bool getRxBuffer(uint8_t *_rxArray) const;
	bool setTxBuffer(const uint8_t *_txArray);
	Protocol_e getProtocol();
	bool setUdpInterface(UdpInterface *_udpInterface);
	UdpInterface* getUdpInterface() const;

	friend class PcInterfaceTester;
private:
	const Protocol_e protocol = UDP; // NOTE: possibly support multiple protocols at same time? would make this array in that case. right now only designed for one protocol though.
	uint8_t rxBuffer [PC_INTERFACE_BUFFER_SIZE] = {}; // TODO: synchronize access to this
	uint8_t txBuffer [PC_INTERFACE_BUFFER_SIZE] = {}; // TODO: synchronize access to this
	UdpInterface *udpInterface = nullptr;
	// Other interfaces may be added here.
	// TODO: implement mutexes before integrating with FreeRTOS
	// rxBuffer mutex
	// txBuffer mutex
};

#endif
