/**
  *****************************************************************************
  * @file    udp_driver.cpp
  * @author  Robert Fairley
  * @brief   Interface for UDP connection driver.
  *
  * @defgroup udp_driver
  * @brief    Manages the usage of a UDP connection, attaching to a lwIP UDP PCB and ethernet interface.
  * @{
  *****************************************************************************
  */

// May rename to LwipUdpDriver as it is specific to lwIP.

// TODO: use const_cast

// TOOD: check defgroups are correct in cpp and h
#include <UdpDriver.h>

namespace udp_driver {

void recvCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
        const ip_addr_t *addr, u16_t port);

UdpDriver::UdpDriver() {

}

UdpDriver::UdpDriver(const ip_addr_t ipaddrIn, const ip_addr_t ipaddrPcIn,
        const u16_t portIn, const u16_t portPcIn,
        const udp_interface::UdpInterface *udpInterfaceIn,
        const os::OsInterface *osInterfaceIn) :
        ipaddr(ipaddrIn), ipaddrPc(ipaddrPcIn), port(portIn), portPc(portPcIn), udpInterface(
                udpInterfaceIn), osInterface(osInterfaceIn) {

}

UdpDriver::~UdpDriver() {

}

bool UdpDriver::setup() {
    bool success = false;

    osMutexStaticDef(UdpDriverRxPbuf, &rxSemaphoreControlBlock);
    rxSemaphore = osInterface->OS_osMutexCreate(osMutex(UdpDriverRxPbuf));

    osMutexStaticDef(UdpDriverTxPbuf, &txSemaphoreControlBlock);
    txSemaphore = osInterface->OS_osMutexCreate(osMutex(UdpDriverTxPbuf));

    osSemaphoreStaticDef(UdpDriverRecv, &recvSemaphoreControlBlock);
    recvSemaphore = osInterface->OS_osSemaphoreCreate(osSemaphore(UdpDriverRecv), 1);

    // XXX: hacky way to initialize the semaphore with a zero count.
    //osSemaphoreWait(recvSemaphore, osWaitForever);
    //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    if (!getUdpInterface()) {
        return false;
    }
    pcb = udpInterface->udpNew();

    if (!pcb) {
        return false;
    }
    success = (udpInterface->udpBind(const_cast<struct udp_pcb *>(pcb), &ipaddr, port) == ERR_OK);
    if (success) {
        udpInterface->udpRecv(const_cast<struct udp_pcb *>(pcb), recvCallback, this);
    } else {
        udpInterface->udpRemove(const_cast<struct udp_pcb *>(pcb));
    }
    return success;
}

// ethernetif_input needs to be called before this
// Should have more error checking where void is returned where possible
bool UdpDriver::receive(uint8_t *rxArrayOut) {
    if (!getUdpInterface()) {
        return false;
    }

    osInterface->OS_osSemaphoreWait(recvSemaphore, osWaitForever); // Wait for callback to write data members (including packets) to UdpInterface

    if (!packetToBytes(rxArrayOut)) { // Copy contents of the packets into rxBuffer
        return false;
    }
    if (udpInterface->pbufFree(getRxPbufThreaded()) == (u8_t) 0) { // Bad if no pbufs were freed here - where did it go?
        return false;
    }
    return true;
}

// Should have more error checking where void is returned where possible
bool UdpDriver::transmit(const uint8_t *txArrayIn) {
    if (!getUdpInterface()) {
        return false;
    }
    struct pbuf *allocPbuf = udpInterface->pbufAlloc(PBUF_TRANSPORT, 80, PBUF_RAM);
    if (!allocPbuf) {
    	return false;
    }
    if (!setTxPbuf(allocPbuf)) { // TODO: Can probably alloc this in setup. Should have proper command size here
    	return false;
    }
    if (!bytesToPacket(txArrayIn)) {
        return false;
    }
    if (udpInterface->udpConnect(const_cast<struct udp_pcb *>(pcb), &ipaddrPc, portPc) != ERR_OK) {
        return false;
    }
    if (udpInterface->udpSend(const_cast<struct udp_pcb *>(pcb), getTxPbufThreaded()) != ERR_OK) {
        return false;
    }
    udpInterface->udpDisconnect(const_cast<struct udp_pcb *>(pcb));
    if (udpInterface->pbufFree(getTxPbufThreaded()) == (u8_t) 0) {
        return false;
    }
    return true;
}

const udp_interface::UdpInterface* UdpDriver::getUdpInterface() const {
    return udpInterface;
}

const os::OsInterface* UdpDriver::getOsInterface() const {
    return osInterface;
}

bool UdpDriver::setRxPbuf(struct pbuf *rxPbufIn) {
    osInterface->OS_osMutexWait(rxSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    rxPbuf = rxPbufIn;
    osInterface->OS_osMutexRelease(rxSemaphore);
    return true;
}

bool UdpDriver::setTxPbuf(struct pbuf *txPbufIn) {
    osInterface->OS_osMutexWait(txSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    txPbuf = txPbufIn;
    osInterface->OS_osMutexRelease(txSemaphore);
    return true;
}

struct pbuf* UdpDriver::getRxPbufThreaded() const {
    osInterface->OS_osMutexWait(rxSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    struct pbuf *p = rxPbuf;
    osInterface->OS_osMutexRelease(rxSemaphore);
    return p;
}

struct pbuf* UdpDriver::getTxPbufThreaded() const {
    osInterface->OS_osMutexWait(txSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    struct pbuf *p = txPbuf;
    osInterface->OS_osMutexRelease(txSemaphore);
    return p;
}

struct pbuf* UdpDriver::getRxPbuf() const {
    struct pbuf *p = rxPbuf;
    return p;
}

struct pbuf* UdpDriver::getTxPbuf() const {
    struct pbuf *p = txPbuf;
    return p;
}

bool UdpDriver::giveRecvSemaphore() {
    osInterface->OS_osSemaphoreRelease(recvSemaphore);
    return true;
}

bool UdpDriver::setPcb(struct udp_pcb *pcbIn) {
    pcb = pcbIn;
    return true;
}

bool UdpDriver::packetToBytes(uint8_t *byteArrayOut) const {
	return (udpInterface->pbufCopyPartial(getRxPbufThreaded(), byteArrayOut, getRxPbufThreaded()->len, 0) > (u16_t) 0); // Not expecting nothing to be copied
}

bool UdpDriver::bytesToPacket(const uint8_t *byteArrayIn) {
	return (udpInterface->pbufTake(getTxPbufThreaded(), byteArrayIn, 80) == ERR_OK); // TODO: should be array size
}

const ip_addr_t UdpDriver::getIpaddr() const {
    return ipaddr;
}

const ip_addr_t UdpDriver::getIpaddrPc() const {
    return ipaddrPc;
}

const u16_t UdpDriver::getPort() const {
    return port;
}

const u16_t UdpDriver::getPortPc() const {
    return portPc;
}

const struct udp_pcb* UdpDriver::getPcb() const {
    return pcb;
}

void recvCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
        const ip_addr_t *addr, u16_t port) {
    udp_driver::UdpDriver *caller = (udp_driver::UdpDriver*) arg;
    caller->setRxPbuf(p);
    caller->giveRecvSemaphore();
}

} // end namespace udp_driver

/**
 * @}
 */
/* end - module name */
