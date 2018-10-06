/**
 *****************************************************************************
 * @file    template_cpp.cpp
 * @author  TODO -- your name here
 * @brief   TODO -- brief description of file
 *
 * @defgroup TODO -- module name
 * @brief    TODO -- description of module
 * @{
 *****************************************************************************
 */

// TOOD: check defgroups are correct in cpp and h
#include <UdpDriver.h>

namespace udp_driver {

void recvCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
        const ip_addr_t *addr, u16_t port);

UdpDriver::UdpDriver() {

}

UdpDriver::UdpDriver(const ip_addr_t *ipaddrIn, const ip_addr_t *ipaddrPcIn,
        const u16_t portIn, const u16_t portPcIn,
        const udp_interface::UdpInterface *udpInterfaceIn,
        const FreeRTOS_Interface::FreeRTOSInterface *osInterfaceIn) :
        ipaddr(ipaddrIn), ipaddrPc(ipaddrPcIn), port(portIn), portPc(portPcIn), udpInterface(
                udpInterfaceIn), osInterface(osInterfaceIn) {
    // TODO: call semaphore create functions
}

UdpDriver::~UdpDriver() {
    // TODO: call semaphore destroy functions
}

bool UdpDriver::setup() {
    bool success = false;
    if (!getUdpInterface()) {
        return false;
    }
    pcb = udpInterface->udpNew();
    if (!pcb) {
        return false;
    }
    success = (udpInterface->udpBind(pcb, ipaddr, port) == ERR_OK);
    if (success) {
        udpInterface->udpRecv(pcb, recvCallback, this);
    } else {
        udpInterface->udpRemove(pcb);
    }
    return success;
}

// Should have more error checking where void is returned where possible
bool UdpDriver::receive(uint8_t *rxArrayOut) {
    if (!getUdpInterface()) {
        return false;
    }

    udpInterface->ethernetifInput(gnetif); // Extract packets from network interface

    BaseType_t osError = osInterface->OS_xSemaphoreTake(recvSemaphore,
            SEMAPHORE_WAIT_NUM_TICKS); // Wait for callback to write data members (including packets) to UdpInterface
    if (osError != pdTRUE) {
        return false;
    }
    if (!packetToBytes(rxArrayOut)) { // Copy contents of the packets into rxBuffer
        return false;
    }
    if (udpInterface->pbufFree(getRxPbuf()) == (u8_t) 0) { // Bad if no pbufs were freed here - where did it go?
        return false;
    }
    return true;
}

// Should have more error checking where void is returned where possible
bool UdpDriver::transmit(const uint8_t *txArrayIn) {
    if (!getUdpInterface()) {
        return false;
    }
    if (!bytesToPacket(txArrayIn)) {
        return false;
    }
    if (udpInterface->udpConnect(pcb, ipaddrPc, portPc) != ERR_OK) {
        return false;
    }
    if (udpInterface->udpSend(pcb, getTxPbuf()) != ERR_OK) {
        return false;
    }
    udpInterface->udpDisconnect(pcb);
    if (udpInterface->pbufFree(getTxPbuf()) == (u8_t) 0) {
        return false;
    }
    return true;
}

const udp_interface::UdpInterface* UdpDriver::getUdpInterface() const {
    return udpInterface;
}

const FreeRTOS_Interface::FreeRTOSInterface* UdpDriver::getOsInterface() const {
    return osInterface;
}

bool UdpDriver::setRxPbuf(struct pbuf *rxPbufIn) {
    osInterface->OS_xSemaphoreTake(rxSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    rxPbuf = rxPbufIn;
    osInterface->OS_xSemaphoreGive(rxSemaphore);
    return true;
}

bool UdpDriver::setTxPbuf(struct pbuf *txPbufIn) {
    osInterface->OS_xSemaphoreTake(txSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    rxPbuf = txPbufIn;
    osInterface->OS_xSemaphoreGive(txSemaphore);
    return true;
}

struct pbuf* UdpDriver::getRxPbufThreaded() const {
    osInterface->OS_xSemaphoreTake(rxSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    struct pbuf *p = rxPbuf;
    osInterface->OS_xSemaphoreGive(rxSemaphore);
    return p;
}

struct pbuf* UdpDriver::getTxPbufThreaded() const {
    osInterface->OS_xSemaphoreTake(txSemaphore, SEMAPHORE_WAIT_NUM_TICKS);
    struct pbuf *p = txPbuf;
    osInterface->OS_xSemaphoreGive(txSemaphore);
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
    osInterface->OS_xSemaphoreGive(recvSemaphore);
    return true;
}

bool UdpDriver::setNetif(struct netif *gnetifIn) {
    gnetif = gnetifIn;
    return true;
}

bool UdpDriver::setPcb(struct udp_pcb *pcbIn) {
    pcb = pcbIn;
    return true;
}

// TODO: implement when doing smoke test
bool UdpDriver::packetToBytes(uint8_t *byteArrayOut) const {
    return true;
}

// TODO: implement when doing smoke test
bool UdpDriver::bytesToPacket(const uint8_t *byteArrayIn) {
    return true;
}

const ip_addr_t* UdpDriver::getIpaddr() const {
    return ipaddr;
}

const ip_addr_t* UdpDriver::getIpaddrPc() const {
    return ipaddrPc;
}

u16_t UdpDriver::getPort() const {
    return port;
}

u16_t UdpDriver::getPortPc() const {
    return portPc;
}

struct udp_pcb* UdpDriver::getPcb() const {
    return pcb;
}

struct netif *UdpDriver::getNetif() const {
    return gnetif;
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
