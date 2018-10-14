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

PcInterface::PcInterface(PcProtocol protocolIn) :
        protocol(protocolIn) {

}

PcInterface::~PcInterface() {

}

bool PcInterface::setup() {
    bool success = false;

    osMutexStaticDef(PcInterfaceRx, &rxMutexControlBlock);
    rxMutex = osInterface->OS_osMutexCreate(osMutex(PcInterfaceRx));

    osMutexStaticDef(PcInterfaceTx, &txMutexControlBlock);
    rxMutex = osInterface->OS_osMutexCreate(osMutex(PcInterfaceTx));

    switch (protocol) {
    case PcProtocol::UDP:

        success = udpDriver->setup();

        break;
    default:
        break;
    }
    return success;
}

bool PcInterface::receive() {
    bool success = false;
    switch (protocol) {
    case PcProtocol::UDP:
        success = udpDriver->receive(rxBuffer);
        break;
    default:
        return false;
    }
    return success;
}

bool PcInterface::transmit() {
    bool success = false;
    switch (protocol) {
    case PcProtocol::UDP:
        success = udpDriver->transmit(txBuffer);
        break;
    default:
        return false;
    }
    return success;
}

bool PcInterface::getRxBuffer(uint8_t *rxArrayOut) const {
    osMutexWait(rxMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iRxBuffer = 0; iRxBuffer < PC_INTERFACE_BUFFER_SIZE; iRxBuffer++) {
        rxArrayOut[iRxBuffer] = rxBuffer[iRxBuffer];
    }
    osMutexRelease(rxMutex);
    return true;
}

bool PcInterface::setTxBuffer(const uint8_t *txArrayIn) {
    osMutexWait(txMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iTxArray = 0; iTxArray < PC_INTERFACE_BUFFER_SIZE; iTxArray++) {
        txBuffer[iTxArray] = txArrayIn[iTxArray];
    }
    osMutexRelease(txMutex);
    return true;
}

PcProtocol PcInterface::getProtocol() const {
    return protocol;
}

bool PcInterface::setUdpDriver(udp_driver::UdpDriver *udpDriverIn) {
    if (getProtocol() != PcProtocol::UDP) {
        return false;
    }
    if (!udpDriverIn) {
        return false;
    }
    udpDriver = udpDriverIn;
    return true;
}

udp_driver::UdpDriver* PcInterface::getUdpDriver() const {
    return udpDriver;
}

bool PcInterface::setUartDriver(uart::UartDriver *uartDriverIn) {
    if (getProtocol() != PcProtocol::USB_UART) {
        return false;
    }
    if (!uartDriverIn) {
        return false;
    }
    uartDriver = uartDriverIn;
    return true;
}

uart::UartDriver* PcInterface::getUartDriver() const {
    return uartDriver;
}

bool PcInterface::setOsInterface(
        os::OsInterface *osInterfaceIn) {
    if (!osInterfaceIn) {
        return false;
    }
    osInterface = osInterfaceIn;
    return true;
}

os::OsInterface* PcInterface::getOsInterface() const {
    return osInterface;
}

bool PcInterface::setRxBuffer(const uint8_t *rxArrayIn) {
    osMutexWait(rxMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iRxArray = 0; iRxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE;
            iRxArray++) {
        rxBuffer[iRxArray] = rxArrayIn[iRxArray];
    }
    osMutexRelease(rxMutex);
    return true;
}

bool PcInterface::getTxBuffer(uint8_t *txArrayOut) const {
    osMutexWait(txMutex, SEMAPHORE_WAIT_NUM_TICKS);
    for (int iTxBuffer = 0; iTxBuffer < pc_interface::PC_INTERFACE_BUFFER_SIZE;
            iTxBuffer++) {
        txArrayOut[iTxBuffer] = txBuffer[iTxBuffer];
    }
    osMutexRelease(txMutex);
    return true;
}

} // end namespace pc_interface

/**
 * @}
 */
/* end - pc_interface */
