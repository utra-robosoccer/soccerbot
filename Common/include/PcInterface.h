/**
 *****************************************************************************
 * @file    PcInterface.h
 * @author  Robert Fairley
 * @brief   Defines and implements a interface through which the PC is accessed, independent of the underlying hardware communication protocol.
 *
 * @defgroup Header
 * @ingroup  pc_interface
 * @{
 *****************************************************************************
 */

#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H

#include <cstdint>
#include <UdpDriver.h>
#include <UartDriver.h>
#include <FreeRTOSInterface.h>

namespace pc_interface {

constexpr TickType_t SEMAPHORE_WAIT_NUM_TICKS = 10;
constexpr int PC_INTERFACE_BUFFER_SIZE = 1024;
constexpr int PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC = 5656;

enum class PcProtocol {
    UDP = PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC, USB_UART
};

class PcInterface {
public:
    PcInterface();
    PcInterface(PcProtocol protocolIn);
    ~PcInterface();

    bool setup();
    bool receive();
    bool transmit();

    bool setRxBuffer(const uint8_t *rxArrayIn);
    bool setTxBuffer(const uint8_t *txArrayIn);
    bool setUdpDriver(udp_driver::UdpDriver *udpDriverIn);
    bool setUartDriver(UART::UartDriver *uartDriverIn);
    bool setOsInterface(FreeRTOS_Interface::FreeRTOSInterface *osInterfaceIn);

    bool getRxBuffer(uint8_t *rxArrayOut) const;
    bool getTxBuffer(uint8_t *txArrayOut) const;
    PcProtocol getProtocol() const;
    udp_driver::UdpDriver* getUdpDriver() const;
    UART::UartDriver* getUartDriver() const;
    FreeRTOS_Interface::FreeRTOSInterface* getOsInterface() const;

private:
    const PcProtocol protocol = PcProtocol::UDP;

    udp_driver::UdpDriver *udpDriver = nullptr;
    UART::UartDriver *uartDriver = nullptr;
    FreeRTOS_Interface::FreeRTOSInterface *osInterface = nullptr;

    uint8_t rxBuffer[PC_INTERFACE_BUFFER_SIZE] = { };
    uint8_t txBuffer[PC_INTERFACE_BUFFER_SIZE] = { };

    // To initialize within constructor
    mutable SemaphoreHandle_t rxMutex;
    mutable SemaphoreHandle_t txMutex;

    // This will be initialized as the eventHandler thread in our system
    osThreadId readerThread;
};

} // end namespace pc_interface

// Some way of threaded tests?
// TODO: doxygen documentation for functions once complete
// TODO: consider one function setDriver(driverptr, drivertype), so interface stays constant as
// drivers are added (no more functions need to be declared)
// NOTE: possibly support multiple protocols at same time? would make this array in that case. right now only designed for one protocol though.
// TODO: consider using DMA for copies to/from rx/txBuffer, for larger PC_INTERFACE_BUFFERS, to allow other threads to run
// TODO: may be better to have counter for space used, so not copying entire buffer length each time
// Consider removing setUdpDriver and have it set in constructor
// TODO: should detect if input array is too small for what the buffer has to give, and pass the error up (return false)
// add another parameter for array size ^
// consider separate threads for comm "servers" which PcInterface binds to
// TODO: maybe UdpInterface class can provide macros in its header like ARGS_THIS_FUNC so no need to retype in child class
// should destructors be doing anything

/**
 * @}
 */
/* end - Header */

#endif /* PC_INTERFACE */
