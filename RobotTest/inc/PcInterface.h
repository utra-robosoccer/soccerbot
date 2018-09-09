#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <cstdint>

#define PC_INTERFACE_BUFFER_SIZE 1024
#define PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC 5656

enum Protocol_e {
	UDP = PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC,
	USB_UART
};

class PcInterface {
public:
	PcInterface();
	~PcInterface();
	PcInterface(Protocol_e _protocol);

	bool setup();
	bool receive();
	bool transmit();
	bool getRxBuffer(uint8_t *_rxArray) const;
	bool setTxBuffer(const uint8_t *_txArray);
	Protocol_e getProtocol();

	friend class PcInterfaceTester;
private:
	const Protocol_e protocol = UDP;
	uint8_t rxBuffer [PC_INTERFACE_BUFFER_SIZE] = {};
	uint8_t txBuffer [PC_INTERFACE_BUFFER_SIZE] = {};
	// TODO: implement mutexes before integrating with FreeRTOS
	// rxBuffer mutex
	// txBuffer mutex
};

// PcInterfaceTester contains testing functions that need access to private members
// but are not to be used under normal circumstances.
class PcInterfaceTester {
public:
	static bool setRxBufferDebug(PcInterface &pcInterfaceUnderTest, uint8_t *_rxArray);
	static bool getTxBufferDebug(PcInterface &pcInterfaceUnderTest, uint8_t *_txArray);
};

#endif
