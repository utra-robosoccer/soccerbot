/**
  *****************************************************************************
  * @file    PcInterface.cpp
  * @author  Robert Fairley
  * @brief   Implementations for classes and structures used within pc_interface.
  *
  * @defgroup pc_interface
  * @brief    Provides an abstract layer to communicate with the PC, and handles all communication-related data and actions appropriate for the selected interface.
  * @{
  *****************************************************************************
  */

#include <PcInterface.h>

namespace pc_interface {

PcInterface::PcInterface() {
}

PcInterface::PcInterface(PcProtocol protocolIn, udp_driver::UdpDriver *udpDriverIn,
        uart::UartDriver *uartDriverIn, os::OsInterface *osInterfaceIn) :
        protocol(protocolIn), udpDriver(udpDriverIn), uartDriver(uartDriverIn),
        osInterface(osInterfaceIn) {
}

PcInterface::~PcInterface() {

}

bool PcInterface::setup() {
    bool success = false;

    osMutexStaticDef(PcInterfaceRx, &rxMutexControlBlock);
    rxMutex = osInterface->OS_osMutexCreate(osMutex(PcInterfaceRx));

    osMutexStaticDef(PcInterfaceTx, &txMutexControlBlock);
    txMutex = osInterface->OS_osMutexCreate(osMutex(PcInterfaceTx));

    switch (protocol) {
    case PcProtocol::UDP:
        success = udpDriver->setup();
        break;
    case PcProtocol::UART:
        success = uartDriver->setup();
        break;
    default:
        break;
    }
    return success;
}

bool PcInterface::receive(const size_t numBytes) {
    bool success = false;
    switch (protocol) {
    case PcProtocol::UDP:
        success = udpDriver->receive(rxBuffer);
        break;
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
    case PcProtocol::UDP:
        success = udpDriver->transmit(txBuffer);
        break;
    case PcProtocol::UART:
        success = uartDriver->transmit(txBuffer, numBytes);
        break;
    default:
        return false;
    }
    return success;
}

bool PcInterface::getRxBuffer(uint8_t *rxArrayOut, const size_t numBytes) const {
    if (sizeof(rxBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(rxMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iRxBuffer = 0; iRxBuffer < numBytes; iRxBuffer++) {
        rxArrayOut[iRxBuffer] = rxBuffer[iRxBuffer];
    }
    osInterface->OS_osMutexRelease(rxMutex);
    return true;
}

bool PcInterface::setTxBuffer(const uint8_t *txArrayIn, const size_t numBytes) {
    if (sizeof(txBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(txMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iTxArray = 0; iTxArray < numBytes; iTxArray++) {
        txBuffer[iTxArray] = txArrayIn[iTxArray];
    }
    osInterface->OS_osMutexRelease(txMutex);
    return true;
}

PcProtocol PcInterface::getProtocol() const {
    return protocol;
}

udp_driver::UdpDriver* PcInterface::getUdpDriver() const {
    return udpDriver;
}

uart::UartDriver* PcInterface::getUartDriver() const {
    return uartDriver;
}

os::OsInterface* PcInterface::getOsInterface() const {
    return osInterface;
}

bool PcInterface::setRxBuffer(const uint8_t *rxArrayIn, const size_t numBytes) {
    if (sizeof(rxBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(rxMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iRxArray = 0; iRxArray < numBytes;
            iRxArray++) {
        rxBuffer[iRxArray] = rxArrayIn[iRxArray];
    }
    osInterface->OS_osMutexRelease(rxMutex);
    return true;
}

bool PcInterface::getTxBuffer(uint8_t *txArrayOut, const size_t numBytes) const {
    if (sizeof(txBuffer) >= numBytes) {
        return false;
    }
    osInterface->OS_osMutexWait(txMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iTxBuffer = 0; iTxBuffer < numBytes;
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
