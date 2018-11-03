/**
  *****************************************************************************
  * @file    PcInterface.h
  * @author  Robert Fairley
  * @brief   Interface through which the PC is accessed, independent of the underlying hardware communication protocol.
  *
  * @defgroup Header
  * @ingroup  pc_interface
  * @{
  *****************************************************************************
  */




#ifndef PC_INTERFACE_H
#define PC_INTERFACE_H
#if defined(STM32F767xx)
#define PC_INTERFACE_USE_LWIP
#endif




/********************************* Includes **********************************/
#include <stdint.h>
#include "OsInterface.h"
#include "UartDriver.h"
#if defined(PC_INTERFACE_USE_LWIP)
#include "UdpDriver.h"
#endif




/******************************** PcInterface ********************************/
namespace pc_interface {

// Constants
// ----------------------------------------------------------------------------
constexpr TickType_t SEMAPHORE_WAIT_NUM_TICKS = 10;
constexpr size_t PC_INTERFACE_BUFFER_SIZE = 1024;
constexpr int PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC = 5656;

// Types & enums
// ----------------------------------------------------------------------------
enum class PcProtocol {
    UART = PC_INTERFACE_PROTOCOL_ENUM_START_MAGIC,
    UDP
};

// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class PcInterface Manages communication drivers and interfaces determined at
 *        compile time, by making the necessary driver calls to receive/transmit
 *        data. Rx and Tx byte arrays are provided to access the data received/
 *        to be sent, respectively. Conversions between the byte arrays and packets
 *        for the selected communication protocol are handled by PcInterface.
 */
class PcInterface {
public:
    PcInterface();
#if defined(PC_INTERFACE_USE_LWIP)
    PcInterface(PcProtocol protocolIn, udp_driver::UdpDriver *udpDriverIn, uart::UartDriver *uartDriverIn, os::OsInterface *osInterfaceIn);
#else
    PcInterface(PcProtocol protocolIn, uart::UartDriver *uartDriverIn, os::OsInterface *osInterfaceIn);
#endif
    ~PcInterface();

    bool setup();
    bool receive(const size_t numBytes);
    bool transmit(const size_t numBytes);

    bool setRxBuffer(const uint8_t *rxArrayIn, const size_t numBytes);
    bool setTxBuffer(const uint8_t *txArrayIn, const size_t numBytes);

    bool getRxBuffer(uint8_t *rxArrayOut, const size_t numBytes) const;
    bool getTxBuffer(uint8_t *txArrayOut, const size_t numBytes) const;
    const PcProtocol getProtocol() const;
#if defined(PC_INTERFACE_USE_LWIP)
    const udp_driver::UdpDriver* getUdpDriver() const;
#endif
    const uart::UartDriver* getUartDriver() const;
    const os::OsInterface* getOsInterface() const;

private:
    const PcProtocol protocol = PcProtocol::UART;
#if defined(PC_INTERFACE_USE_LWIP)
    const udp_driver::UdpDriver *udpDriver = nullptr;
#endif
    const uart::UartDriver *uartDriver = nullptr;
    const os::OsInterface *osInterface = nullptr;

    uint8_t rxBuffer[PC_INTERFACE_BUFFER_SIZE] = { };
    uint8_t txBuffer[PC_INTERFACE_BUFFER_SIZE] = { };

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
