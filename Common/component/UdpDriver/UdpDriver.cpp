/**
  *****************************************************************************
  * @file    UdpDriver.cpp
  * @author  Robert Fairley
  *
  * @defgroup udp_driver
  * @brief    Implementation of UdpDriver using UDP functions of the lwIP Raw API.
  * @{
  *****************************************************************************
  */

/* TODO: namespacing+doxygen groups to be redone, and UdpInterface to be renamed to LwipRawUdpInterface. */
/* TODO: licensing terms for projects we are making use of e.g. googletest? */




/********************************* Includes **********************************/
#include "UdpDriver.h"


using lwip::UdpInterface;
using cmsis::OsInterface;


/******************************** File-local *********************************/
namespace {

// Functions
// ----------------------------------------------------------------------------

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
    const ip_addr_t addr = caller->getIpaddrPc();
    bool success = false;

    if (caller->getUdpInterface()->udpConnect(const_cast<struct udp_pcb *>(caller->getPcb()),
            &addr, caller->getPortPc()) != ERR_OK) {
        goto out;
    }
    if (caller->getUdpInterface()->udpSend(const_cast<struct udp_pcb *>(caller->getPcb()), pPbuf) != ERR_OK) {
        goto disconnect;
    }

    success = true;

  disconnect:
    caller->getUdpInterface()->udpDisconnect(const_cast<struct udp_pcb *>(caller->getPcb()));

  out:
    return success;
}

} // end anonymous namespace

namespace udp_driver {

/************************** UdpDriver ***************************/
// Public
// ----------------------------------------------------------------------------

UdpDriver::UdpDriver() {
}

UdpDriver::UdpDriver(const ip_addr_t ipaddrIn,
                     const ip_addr_t ipaddrPcIn,
                     const u16_t portIn,
                     const u16_t portPcIn,
                     const UdpInterface *udpInterfaceIn,
                     const OsInterface *osInterfaceIn
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

/**
 * @brief initialize UdpDriver and do critical checks.
 * @return false if a check fails, true if initialization completes successfully.
 */
bool UdpDriver::initialize() {
    if (!getUdpInterface() || !getOsInterface()) {
        return false;
    }

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

/**
 * @brief set up UdpDriver to receive UdpPackets, calling recvCallback when a packet is received.
 * @param recvCallback callback function to call when lwIP detects a packet has been received.
 * @return true if setup completed successfully, false otherwise.
 */
bool UdpDriver::setupReceive(udp_recv_fn recvCallback) {
    bool success = false;

    pcb = getUdpInterface()->udpNew();

    if (!getPcb()) {
        return false;
    }

    success = (getUdpInterface()->udpBind(const_cast<struct udp_pcb *>(getPcb()), &ipaddr, port) == ERR_OK);

    if (success) {
        udp_recv_fn callback = (recvCallback == nullptr) ? defaultRecvCallback : recvCallback;
        getUdpInterface()->udpRecv(const_cast<struct udp_pcb *>(getPcb()), callback, this);
    } else {
        forgetPcb();
    }

    return success;
}

/**
 * @brief Wait for a packet to be available to read, and read the packet into rxArrayOut. Cleans up the packet read from lwIP.
 * @param rxArrayOut array to read packet data payload into.
 * @param numBytes the maximum number of bytes to read from the packet (should not be greater than the length of rxArrayout).
 * @return true if read packet successfully, false if failed to read the packet into rxArrayOut.
 */
bool UdpDriver::receive(uint8_t *rxArrayOut, const size_t numBytes) {
    bool success = false;
    struct pbuf *recvPbuf = nullptr;

    waitReceiveCplt();

    recvPbuf = getRecvPbuf();

    if (packetToBytes(rxArrayOut, numBytes, recvPbuf) <= (u16_t) 0) {
        goto out;
    }

    success = true;

  out:

    if (recvPbuf) {
        forgetRecvPbuf();
    }

    return success;
}

/**
 * @brief transmit data from txArrayIn through UDP. Allocates and cleans up a new packet internally.
 * @param txArrayIn array holding data to transmit.
 * @param number of bytes to transmit.
 * @return true if the data was transmitted successfully, false otherwise.
 */
bool UdpDriver::transmit(const uint8_t *txArrayIn, const size_t numBytes) {
    bool success = false;

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

/**
 * @brief Copy the received packet to byteArrayOut, to a maximum of numBytes.
 * @param byteArrayOut the array to copy the bytes from the packet to.
 * @param numBytes the maximum number of bytes to copy (should not be greater than the size of byteArrayOut).
 * @param pPbuf pointer to the packet (pbuf) to copy the bytes from.
 * @return the number of bytes copied to byteArrayOut.
 */
u16_t UdpDriver::packetToBytes(uint8_t *byteArrayOut, const size_t numBytes, struct pbuf *pPbuf) const {
    if (!byteArrayOut || !pPbuf) {
        return (u16_t) 0;
    }

	return getUdpInterface()->pbufCopyPartial(pPbuf, byteArrayOut, numBytes, 0);
}

/**
 * @brief Copy numBytes from byteArrayIn to a packet.
 * @param byteArrayOut the array to copy the bytes to the packet from.
 * @param numBytes the number of bytes to copy from byteArrayIn.
 * @param pPbuf pointer to the packet (pbuf) to copy the bytes to.
 * @return true if copied numBytes succesfully, false otherwise.
 */
bool UdpDriver::bytesToPacket(const uint8_t *byteArrayIn, const size_t numBytes, struct pbuf *pPbuf) const {
    if (!byteArrayIn || !pPbuf) {
        return false;
    }

	return (getUdpInterface()->pbufTake(pPbuf, byteArrayIn, numBytes) == ERR_OK);
}

void UdpDriver::signalReceiveCplt() {
    getOsInterface()->OS_osSemaphoreRelease(recvSemaphore);
}

void UdpDriver::waitReceiveCplt() {
    while (getOsInterface()->OS_osSemaphoreWait(recvSemaphore, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
}

void UdpDriver::setRecvPbuf(struct pbuf *pPbuf) {
    while (getOsInterface()->OS_osMutexWait(recvPbufMutex, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
    recvPbuf = pPbuf;
    getOsInterface()->OS_osMutexRelease(recvPbufMutex);
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

const UdpInterface* UdpDriver::getUdpInterface() const {
    return udpInterface;
}

const OsInterface* UdpDriver::getOsInterface() const {
    return osInterface;
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

void UdpDriver::forgetRecvPbuf() {
    while (getOsInterface()->OS_osMutexWait(recvPbufMutex, SEMAPHORE_WAIT_NUM_MS) != osOK) {
        ;
    }
    getUdpInterface()->pbufFree(recvPbuf);
    setRecvPbuf(nullptr);
    getOsInterface()->OS_osMutexRelease(recvPbufMutex);
}

void UdpDriver::forgetPcb() {
    getUdpInterface()->udpRemove(const_cast<struct udp_pcb *>(getPcb()));
    pcb = nullptr;
}

} // end namespace udp_driver

/**
 * @}
 */
/* end - udp_driver */
