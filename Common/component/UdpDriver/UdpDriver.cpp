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
/* TODO: consider refactoring setup, receive, transmit if making the class support >1 UDP connection. */

// TODO: test coverage for UdpDriver receive and transmit failure/success paths, set up fixtures to test passing
//       data between rx/txBuffers and pbufs.
// TODO: check against template and add doxygen comments

#include "UdpDriver.h"

namespace {

static void defaultRecvCallback(void *arg,
                                struct udp_pcb *pcb,
                                struct pbuf *pPbuf,
                                const ip_addr_t *addr,
                                u16_t port)
{
    udp_driver::UdpDriver *caller = (udp_driver::UdpDriver*) arg;
    caller->setRecvPbuf(pPbuf);
    caller->signalReceiveCplt();
}

static bool transmitImpl(udp_driver::UdpDriver* caller, struct pbuf * pPbuf) {
    ip_addr_t addr = caller->getIpaddrPc();

    if (caller->getUdpInterface()->udpConnect(const_cast<struct udp_pcb *>(caller->getPcb()),
            &addr, caller->getPortPc()) != ERR_OK) {
        return false;
    }
    if (caller->getUdpInterface()->udpSend(const_cast<struct udp_pcb *>(caller->getPcb()), pPbuf) != ERR_OK) {
        return false;
    }
    caller->getUdpInterface()->udpDisconnect(const_cast<struct udp_pcb *>(caller->getPcb()));

    return true;
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

}

bool UdpDriver::initialize() {
    osSemaphoreStaticDef(UdpDriverRecv, &recvSemaphoreControlBlock);
    if ((recvSemaphore = getOsInterface()->OS_osSemaphoreCreate(osSemaphore(UdpDriverRecv), 1)) == NULL) {
        return false;
    }

    osMutexStaticDef(UdpDriverRecvPbuf, &recvPbufMutexControlBlock);
    if ((recvPbufMutex = getOsInterface()->OS_osMutexCreate(osMutex(UdpDriverRecvPbuf))) == NULL) {
        return false;
    }
    return true;
}

bool UdpDriver::setupReceive(udp_recv_fn recvCallback) {
    bool success = false;

    if (!getUdpInterface() || !getOsInterface()) {
        return false;
    }

    pcb = getUdpInterface()->udpNew();

    if (!getPcb()) {
        return false;
    }

    success = (getUdpInterface()->udpBind(const_cast<struct udp_pcb *>(getPcb()), &ipaddr, port) == ERR_OK);

    if (success) {
        udp_recv_fn callback = (recvCallback == nullptr) ? defaultRecvCallback : recvCallback;
        getUdpInterface()->udpRecv(const_cast<struct udp_pcb *>(getPcb()), callback, this);
    } else {
        getUdpInterface()->udpRemove(const_cast<struct udp_pcb *>(getPcb()));
        pcb = nullptr;
    }

    return success;
}

void UdpDriver::unSetupReceive() {
    if (getPcb()) {
        getUdpInterface()->udpRemove(const_cast<struct udp_pcb *>(getPcb()));
        pcb = nullptr;
    }
    if (getRecvPbuf()) {
        forgetRecvPbuf();
    }
}

bool UdpDriver::receive(uint8_t *rxArrayOut, const size_t numBytes) {
    // TODO: also set recvPbuf to rxArrayOut, and free recvPbuf
    bool success = false;
    struct pbuf *recvPbuf = nullptr;

    if (!getUdpInterface() || !getOsInterface()) {
        goto out;
    }

    waitReceiveCplt();

    recvPbuf = getRecvPbuf();

    if (!packetToBytes(rxArrayOut, numBytes, recvPbuf)) {
        goto out;
    }

    success = true;

  out:

    if (recvPbuf) {
        forgetRecvPbuf();
    }

    return success;
}

/** Transmit numBytes from txArrayIn, by allocating a new pbuf and sending
 *  the pbuf to the PC. Cleans up the pbuf internally. */
bool UdpDriver::transmit(const uint8_t *txArrayIn, const size_t numBytes) {
    bool success = false;

    if (!getUdpInterface() || !getOsInterface()) {
        return false;
    }

    /* TODO: see if this can be allocated once at setup, if numBytes is known and unchanging. */
    struct pbuf *allocPbuf = getUdpInterface()->pbufAlloc(PBUF_TRANSPORT, numBytes, PBUF_RAM);
    if (!allocPbuf) {
        goto out;
    }

    if (!bytesToPacket(txArrayIn, numBytes, allocPbuf)) {
        goto out;
    }

    if (!transmitImpl(this, allocPbuf)) {
        goto out;
    }

    success = true;

  out:
    if (allocPbuf) {
        /* Error if zero pbufs freed for the one just allocated for Tx. */
        if ((success = getUdpInterface()->pbufFree(allocPbuf) > (u8_t) 0)) {
            allocPbuf = nullptr;
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

void UdpDriver::signalReceiveCplt() {
    getOsInterface()->OS_osSemaphoreRelease(recvSemaphore); // XXX
}

void UdpDriver::waitReceiveCplt() {
    while (getOsInterface()->OS_osSemaphoreWait(recvSemaphore, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
}

/* Copy the received packet to byteArrayOut, to a maximum of numBytes. */
bool UdpDriver::packetToBytes(uint8_t *byteArrayOut, const size_t numBytes, struct pbuf *pPbuf) const {
    if (!byteArrayOut || !pPbuf) {
        return false;
    }

    /* Success only if a nonzero number of bytes was copied. */
	return (getUdpInterface()->pbufCopyPartial(pPbuf, byteArrayOut, numBytes, 0) > (u16_t) 0);
}

/* Copy byteArrayIn of size numBytes to the transmit packet. */
bool UdpDriver::bytesToPacket(const uint8_t *byteArrayIn, const size_t numBytes, struct pbuf *pPbuf) {
    if (!byteArrayIn || !pPbuf) {
        return false;
    }

	return (getUdpInterface()->pbufTake(pPbuf, byteArrayIn, numBytes) == ERR_OK);
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

struct udp_pcb* UdpDriver::getPcb() const {
    return pcb;
}

struct pbuf* UdpDriver::getRecvPbuf() const {
    while (getOsInterface()->OS_osMutexWait(recvPbufMutex, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
    struct pbuf *pPbuf = recvPbuf;
    getOsInterface()->OS_osMutexRelease(recvPbufMutex);
    return pPbuf;
}

void UdpDriver::setRecvPbuf(struct pbuf *pPbuf) {
    while (getOsInterface()->OS_osMutexWait(recvPbufMutex, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
    recvPbuf = pPbuf;
    getOsInterface()->OS_osMutexRelease(recvPbufMutex);
}

void UdpDriver::forgetRecvPbuf() {
    while (getOsInterface()->OS_osMutexWait(recvPbufMutex, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
    getUdpInterface()->pbufFree(recvPbuf);
    setRecvPbuf(nullptr);
    getOsInterface()->OS_osMutexRelease(recvPbufMutex);
}

} // end namespace udp_driver

/**
 * @}
 */
/* end - udp_driver */
