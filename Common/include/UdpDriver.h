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
#include <FreeRTOSInterface.h>

namespace udp_driver {

constexpr TickType_t SEMAPHORE_WAIT_NUM_TICKS = 10;

class UdpDriver {
public:
    UdpDriver();
    UdpDriver(const ip_addr_t *ipaddrIn, const ip_addr_t *ipaddrPcIn,
            const u16_t portIn, const u16_t portPcIn,
            const udp_interface::UdpInterface *udpInterfaceIn,
            const FreeRTOS_Interface::FreeRTOSInterface *osInterfaceIn);
    ~UdpDriver();

    bool setup();
    bool receive(uint8_t *rxArrayOut);
    bool transmit(const uint8_t *txArrayIn);
    bool packetToBytes(uint8_t *byteArrayOut) const;
    bool bytesToPacket(const uint8_t *byteArrayIn);

    // TODO: probably should encapsulate give/take for other semaphores
    bool giveRecvSemaphore();

    bool setPcb(struct udp_pcb *pcbIn);
    bool setNetif(struct netif *gnetifIn);
    bool setRxPbuf(struct pbuf *rxPbufIn);
    bool setTxPbuf(struct pbuf *txPbufIn);

    // Read-only unless set function provided
    const udp_interface::UdpInterface* getUdpInterface() const;
    const FreeRTOS_Interface::FreeRTOSInterface* getOsInterface() const;
    struct pbuf* getRxPbuf() const;
    struct pbuf* getTxPbuf() const;
    struct pbuf* getRxPbufThreaded() const;
    struct pbuf* getTxPbufThreaded() const;
    const ip_addr_t* getIpaddr() const;
    const ip_addr_t* getIpaddrPc() const;
    u16_t getPort() const;
    u16_t getPortPc() const;
    struct udp_pcb* getPcb() const;
    struct netif* getNetif() const;

private:
    const ip_addr_t *ipaddr = nullptr;
    const ip_addr_t *ipaddrPc = nullptr;
    const u16_t port = 0;
    const u16_t portPc = 0;

    // No set function provided
    const udp_interface::UdpInterface *udpInterface = nullptr;
    const FreeRTOS_Interface::FreeRTOSInterface *osInterface = nullptr;

    // References to non-const structures
    struct udp_pcb *pcb = nullptr;
    struct netif *gnetif = nullptr;
    struct pbuf *rxPbuf = nullptr;
    struct pbuf *txPbuf = nullptr;

    // mutable as they do not represent external state of class - can be modified in const functions
    // Initialized within constructor
    mutable SemaphoreHandle_t rxSemaphore;
    mutable SemaphoreHandle_t txSemaphore;
    mutable SemaphoreHandle_t recvSemaphore;
};

} // end namespace udp_driver

/**
 * @}
 */
/* end - Header */

#endif /* UDP_DRIVER_H */
