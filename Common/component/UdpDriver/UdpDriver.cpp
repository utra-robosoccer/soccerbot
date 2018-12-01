/**
  *****************************************************************************
  * @file    UdpDriver.cpp
  * @author  Robert Fairley
  * @brief   Implementation of UdpDriver using UDP functions of the lwIP Raw API.
  *
  * @defgroup udp_driver
  * @{
  *****************************************************************************
  */

/* TODO: namespacing+doxygen groups to be redone, and UdpInterface to be renamed to LwipRawUdpInterface. */
/* TODO: licensing terms for projects we are making use of e.g. googletest? */


// TODO: test coverage for UdpDriver receive and transmit failure/success paths, set up fixtures to test passing
//       data between rx/txBuffers and pbufs.
// TODO: further refactor setup, receive, transmit
// TODO: check against template and add doxygen comments
// TODO: add in correct semaphores

#include "UdpDriver.h"

namespace {

void recvCallback(void *arg,
                  struct udp_pcb *pcb,
                  struct pbuf *packet,
                  const ip_addr_t *addr,
                  u16_t port)
{
    udp_driver::UdpDriver *caller = (udp_driver::UdpDriver*) arg;
    caller->setRxPbuf(packet);
    caller->signalReceiveCplt();
}

} // end anonymous namespace

namespace udp_driver {

UdpDriver::UdpDriver() {
}

UdpDriver::UdpDriver(const ip_addr_t ipaddrIn,
                     const ip_addr_t ipaddrPcIn,
                     const u16_t portIn,
                     const u16_t portPcIn,
                     const udp_interface::UdpInterface *udpInterfaceIn,
                     const os::OsInterface *osInterfaceIn
                     ) :
                         ipaddr(ipaddrIn),
                         ipaddrPc(ipaddrPcIn),
                         port(portIn),
                         portPc(portPcIn),
                         udpInterface(udpInterfaceIn),
                         osInterface(osInterfaceIn)
{

}

UdpDriver::~UdpDriver() {
    /* TODO: think about a macro for all the common "interface" calls. */
    if (getPcb()) {
        getUdpInterface()->udpRemove(const_cast<struct udp_pcb *>(getPcb()));
        pcb = nullptr;
    }
    if (getRxPbuf()) {
        getUdpInterface()->pbufFree(getRxPbuf());
        rxPbuf = nullptr;
    }
    if (getTxPbuf()) {
        getUdpInterface()->pbufFree(getTxPbuf());
        txPbuf = nullptr;
    }
}

bool UdpDriver::setup() {
    bool success = false;

    osSemaphoreStaticDef(UdpDriverRecv, &recvSemaphoreControlBlock);
    if ((recvSemaphore = getOsInterface()->OS_osSemaphoreCreate(osSemaphore(UdpDriverRecv), 1)) == NULL) {
        return false;
    }

    if (!getUdpInterface() || !getOsInterface()) {
        return false;
    }

    pcb = getUdpInterface()->udpNew();

    if (!getPcb()) {
        return false;
    }

    success = (getUdpInterface()->udpBind(const_cast<struct udp_pcb *>(getPcb()), &ipaddr, port) == ERR_OK);

    if (success) {
        getUdpInterface()->udpRecv(const_cast<struct udp_pcb *>(getPcb()), recvCallback, this);
    } else {
        getUdpInterface()->udpRemove(const_cast<struct udp_pcb *>(getPcb()));
        pcb = nullptr;
    }

    return success;
}

bool UdpDriver::receive(uint8_t *rxArrayOut, const size_t numBytes) {
    bool success = false;

    if (!getUdpInterface()) {
        goto out;
    }

    waitReceiveCplt();

    if (!packetToBytes(rxArrayOut, numBytes)) {
        goto out;
    }

    success = true;

  out:
    if (getRxPbuf()) {
        /* If zero pbufs were freed here, where did the allocated one for Rx go? Consider this an error. */
        if ((success = getUdpInterface()->pbufFree(getRxPbuf()) == (u8_t) 0)) {
            rxPbuf = nullptr;
        }
    }

    return success;
}

/** Transmit numBytes from txArrayIn, by allocating a new pbuf and sending
 *  the pbuf to the PC. Cleans up the pbuf internally. */
bool UdpDriver::transmit(const uint8_t *txArrayIn, const size_t numBytes) {
    bool success = false;

    if (!getUdpInterface()) {
        return false;
    }

    /* TODO: see if this can be allocated once at setup, if numBytes is known and unchanging. */
    struct pbuf *allocPbuf = getUdpInterface()->pbufAlloc(PBUF_TRANSPORT, numBytes, PBUF_RAM);
    if (!allocPbuf) {
        goto out;
    }

    setTxPbuf(allocPbuf);

    if (!bytesToPacket(txArrayIn, numBytes)) {
        goto out;
    }
    if (getUdpInterface()->udpConnect(const_cast<struct udp_pcb *>(getPcb()), &ipaddrPc, getPortPc()) != ERR_OK) {
        goto out;
    }
    if (getUdpInterface()->udpSend(const_cast<struct udp_pcb *>(getPcb()), getTxPbuf()) != ERR_OK) {
        goto out;
    }

    getUdpInterface()->udpDisconnect(const_cast<struct udp_pcb *>(getPcb()));

    success = true;

  out:
    if (getTxPbuf()) {
        /* Error if zero pbufs freed for the one just allocated for Tx. */
        if ((success = getUdpInterface()->pbufFree(getTxPbuf()) == (u8_t) 0)) {
            txPbuf = nullptr;
        }
    }

    return success;
}

const udp_interface::UdpInterface* UdpDriver::getUdpInterface() const {
    return udpInterface;
}

const os::OsInterface* UdpDriver::getOsInterface() const {
    return osInterface;
}

void UdpDriver::setRxPbuf(struct pbuf *rxPbufIn) {
    rxPbuf = rxPbufIn;
}

void UdpDriver::setTxPbuf(struct pbuf *txPbufIn) {
    txPbuf = txPbufIn;
}

struct pbuf* UdpDriver::getRxPbuf() const {
    return rxPbuf;
}

struct pbuf* UdpDriver::getTxPbuf() const {
    return txPbuf;
}

void UdpDriver::signalReceiveCplt() {
    getOsInterface()->OS_osSemaphoreRelease(recvSemaphore); // XXX
}

void UdpDriver::waitReceiveCplt() {
    while (getOsInterface()->OS_osSemaphoreWait(recvSemaphore, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
}

/* Copy the received packet to byteArrayOut, to a maximum of numBytes. */
bool UdpDriver::packetToBytes(uint8_t *byteArrayOut, const size_t numBytes) const {
    if (!byteArrayOut) {
        return false;
    }

    /* Success only if a nonzero number of bytes was copied. */
	return (getUdpInterface()->pbufCopyPartial(getRxPbuf(), byteArrayOut, numBytes, 0) > (u16_t) 0);
}

/* Copy byteArrayIn of size numBytes to the transmit packet. */
bool UdpDriver::bytesToPacket(const uint8_t *byteArrayIn, const size_t numBytes) {
    if (!byteArrayIn) {
        return false;
    }

	return (getUdpInterface()->pbufTake(getTxPbuf(), byteArrayIn, numBytes) == ERR_OK);
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

} // end namespace udp_driver

/**
 * @}
 */
/* end - udp_driver */
