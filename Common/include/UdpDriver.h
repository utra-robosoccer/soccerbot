/**
  *****************************************************************************
  * @file    UdpDriver.h
  * @author  Robert Fairley
  * @brief   Interface of UDP driver.
  *
  * @defgroup Header
  * @ingroup  udp_driver
  * @{
  *****************************************************************************
  */

#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

#include <UdpInterface.h>
#include <OsInterface.h>

// TODO: redo namespacing

namespace udp_driver {

constexpr TickType_t SEMAPHORE_WAIT_NUM_TICKS = 10;

class UdpDriver {
public:
    UdpDriver();
    UdpDriver(const ip_addr_t ipaddrIn, const ip_addr_t ipaddrPcIn,
            const u16_t portIn, const u16_t portPcIn,
            const udp_interface::UdpInterface *udpInterfaceIn,
            const os::OsInterface *osInterfaceIn);
    ~UdpDriver();

    bool setup();
    bool receive(uint8_t *rxArrayOut);
    bool transmit(const uint8_t *txArrayIn);
    bool packetToBytes(uint8_t *byteArrayOut) const;
    bool bytesToPacket(const uint8_t *byteArrayIn);

    bool giveRecvSemaphore();

    bool setPcb(struct udp_pcb *pcbIn);
    bool setRxPbuf(struct pbuf *rxPbufIn);
    bool setTxPbuf(struct pbuf *txPbufIn);

    const udp_interface::UdpInterface* getUdpInterface() const;
    const os::OsInterface* getOsInterface() const;
    struct pbuf* getRxPbuf() const;
    struct pbuf* getTxPbuf() const;
    struct pbuf* getRxPbufThreaded() const;
    struct pbuf* getTxPbufThreaded() const;
    const ip_addr_t getIpaddr() const;
    const ip_addr_t getIpaddrPc() const;
    const u16_t getPort() const;
    const u16_t getPortPc() const;
    const struct udp_pcb* getPcb() const;

private:
    const ip_addr_t ipaddr = {0x0};
    const ip_addr_t ipaddrPc = {0x0};
    const u16_t port = 0;
    const u16_t portPc = 0;

    const udp_interface::UdpInterface *udpInterface = nullptr;
    const os::OsInterface *osInterface = nullptr;

    const struct udp_pcb *pcb = nullptr;
    struct pbuf *rxPbuf = nullptr;
    struct pbuf *txPbuf = nullptr;

    // mutable as they do not represent external state of class - can be modified in const functions
    // Initialized within constructor
    mutable osMutexId rxSemaphore;
    mutable osStaticMutexDef_t rxSemaphoreControlBlock;
    mutable osMutexId txSemaphore;
    mutable osStaticMutexDef_t txSemaphoreControlBlock;
    mutable osSemaphoreId recvSemaphore;
    mutable osStaticSemaphoreDef_t recvSemaphoreControlBlock;
};

} // end namespace udp_driver

/**
 * @}
 */
/* end - Header */

#endif /* UDP_DRIVER_H */
