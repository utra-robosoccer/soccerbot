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
#include <OsInterface.h>

namespace pc_interface {

constexpr TickType_t SEMAPHORE_WAIT_NUM_TICKS = 10;
constexpr size_t PC_INTERFACE_BUFFER_SIZE = 1024;
constexpr int PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC = 5656;

enum class PcProtocol {
    UDP = PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC,
    UART
};

class PcInterface {
public:
    PcInterface();
    PcInterface(PcProtocol protocolIn, udp_driver::UdpDriver *udpDriverIn, uart::UartDriver *uartDriverIn, os::OsInterface *osInterfaceIn);
    ~PcInterface();

    bool setup();
    bool receive(const size_t numBytes);
    bool transmit(const size_t numBytes);

    bool setRxBuffer(const uint8_t *rxArrayIn, const size_t numBytes);
    bool setTxBuffer(const uint8_t *txArrayIn, const size_t numBytes);

    bool getRxBuffer(uint8_t *rxArrayOut, const size_t numBytes) const;
    bool getTxBuffer(uint8_t *txArrayOut, const size_t numBytes) const;
    PcProtocol getProtocol() const;
    udp_driver::UdpDriver* getUdpDriver() const;
    uart::UartDriver* getUartDriver() const;
    os::OsInterface* getOsInterface() const;

private:
    const PcProtocol protocol = PcProtocol::UDP;

    udp_driver::UdpDriver *udpDriver = nullptr;
    uart::UartDriver *uartDriver = nullptr;
    os::OsInterface *osInterface = nullptr;

    uint8_t rxBuffer[PC_INTERFACE_BUFFER_SIZE] = { };
    uint8_t txBuffer[PC_INTERFACE_BUFFER_SIZE] = { };

    // To initialize within constructor
    mutable osMutexId rxMutex;
    mutable osStaticMutexDef_t rxMutexControlBlock;
    mutable osMutexId txMutex;
    mutable osStaticMutexDef_t txMutexControlBlock;
};

} // end namespace pc_interface

// Some way of threaded tests?
// TODO: doxygen documentation for functions once complete
// TODO: consider one function setDriver(driverptr, drivertype), so interface stays constant as
// drivers are added (no more functions need to be declared)
// NOTE: possibly support multiple protocols at same time? would make this array in that case. right now only designed for one protocol though.
// consider separate threads for comm "servers" which PcInterface binds to
// TODO: maybe UdpInterface class can provide macros in its header like ARGS_THIS_FUNC so no need to retype in child class
// should destructors be doing anything

/**
 * @}
 */
/* end - Header */

#endif /* PC_INTERFACE */
