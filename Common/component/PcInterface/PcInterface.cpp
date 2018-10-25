/**
  *****************************************************************************
  * @file    PcInterface.cpp
  * @author  Robert Fairley
  * @brief   Implementation for PcInterface.
  *
  * @defgroup pc_interface
  * @brief    Abstract layer to communicate with a PC through a chosen communication protocol.
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "PcInterface.h"




namespace pc_interface {

/********************************* PcInterface *******************************/
// Public
// ----------------------------------------------------------------------------
PcInterface::PcInterface() {
}

#if defined(PC_INTERFACE_USE_LWIP)
PcInterface::PcInterface(PcProtocol protocolIn, udp_driver::UdpDriver *udpDriverIn,
        uart::UartDriver *uartDriverIn, os::OsInterface *osInterfaceIn) :
        protocol(protocolIn), udpDriver(udpDriverIn), uartDriver(uartDriverIn),
        osInterface(osInterfaceIn) {
}
#else
PcInterface::PcInterface(PcProtocol protocolIn,
        uart::UartDriver *uartDriverIn, os::OsInterface *osInterfaceIn) :
        protocol(protocolIn), uartDriver(uartDriverIn),
        osInterface(osInterfaceIn) {
}
#endif

PcInterface::~PcInterface() {

}

bool PcInterface::setup() {
    bool success = false;

    osMutexStaticDef(PcInterfaceRx, &rxMutexControlBlock);
    rxMutex = osInterface->OS_osMutexCreate(osMutex(PcInterfaceRx));

    osMutexStaticDef(PcInterfaceTx, &txMutexControlBlock);
    txMutex = osInterface->OS_osMutexCreate(osMutex(PcInterfaceTx));

    switch (protocol) {
#if defined(PC_INTERFACE_USE_LWIP)
    case PcProtocol::UDP:
        // TODO: this may be good place for CONST_API_CALL define to wrap the const_cast
        success = const_cast<udp_driver::UdpDriver*>(udpDriver)->setup();
        break;
#endif
    case PcProtocol::UART:
        success = const_cast<uart::UartDriver*>(uartDriver)->setup();
        break;
    default:
        break;
    }
    return success;
}

bool PcInterface::receive(const size_t numBytes) {
    bool success = false;
    switch (protocol) {
#if defined(PC_INTERFACE_USE_LWIP)
    case PcProtocol::UDP:
        success = const_cast<udp_driver::UdpDriver*>(udpDriver)->receive(rxBuffer);
        break;
#endif
    case PcProtocol::UART:
        success = uartDriver->receive(rxBuffer, numBytes);
        break;
    default:
        return false;
    }
    return success;
}

bool PcInterface::transmit(const size_t numBytes) {
    bool success = false;
    switch (protocol) {
#if defined(PC_INTERFACE_USE_LWIP)
    case PcProtocol::UDP:
        success = const_cast<udp_driver::UdpDriver*>(udpDriver)->transmit(txBuffer);
        break;
#endif
    case PcProtocol::UART:
        success = uartDriver->transmit(txBuffer, numBytes);
        break;
    default:
        return false;
    }
    return success;
}

bool PcInterface::setRxBuffer(const uint8_t *rxArrayIn, const size_t numBytes) {
    if (sizeof(rxBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(rxMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (size_t iRxArray = 0; iRxArray < numBytes;
            iRxArray++) {
        rxBuffer[iRxArray] = rxArrayIn[iRxArray];
    }
    osInterface->OS_osMutexRelease(rxMutex);
    return true;
}

bool PcInterface::setTxBuffer(const uint8_t *txArrayIn, const size_t numBytes) {
    if (sizeof(txBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(txMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (size_t iTxArray = 0; iTxArray < numBytes; iTxArray++) {
        txBuffer[iTxArray] = txArrayIn[iTxArray];
    }
    osInterface->OS_osMutexRelease(txMutex);
    return true;
}

const PcProtocol PcInterface::getProtocol() const {
    return protocol;
}

#if defined(PC_INTERFACE_USE_LWIP)
const udp_driver::UdpDriver* PcInterface::getUdpDriver() const {
    return udpDriver;
}
#endif

const uart::UartDriver* PcInterface::getUartDriver() const {
    return uartDriver;
}

const os::OsInterface* PcInterface::getOsInterface() const {
    return osInterface;
}

bool PcInterface::getRxBuffer(uint8_t *rxArrayOut, const size_t numBytes) const {
    if (sizeof(rxBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(rxMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (size_t iRxBuffer = 0; iRxBuffer < numBytes; iRxBuffer++) {
        rxArrayOut[iRxBuffer] = rxBuffer[iRxBuffer];
    }
    osInterface->OS_osMutexRelease(rxMutex);
    return true;
}

bool PcInterface::getTxBuffer(uint8_t *txArrayOut, const size_t numBytes) const {
    if (sizeof(txBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(txMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (size_t iTxBuffer = 0; iTxBuffer < numBytes;
            iTxBuffer++) {
        txArrayOut[iTxBuffer] = txBuffer[iTxBuffer];
    }
    osInterface->OS_osMutexRelease(txMutex);
    return true;
}

} // end namespace pc_interface

/**
 * @}
 */
/* end - pc_interface */
