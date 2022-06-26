/**
  *****************************************************************************
  * @file
  * @author  Robert Fairley
  * @brief   Abstraction over UDP library providing common driver functions.
  *
  * @defgroup HeaderUdpDriver
  * @ingroup  lwip
  * @{
  *****************************************************************************
  */




#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H




/********************************* Includes **********************************/
#include "UdpRawInterface.h"
#include "OsInterface.h"

using cmsis::OsInterface;
using lwip::UdpRawInterface;

namespace udp {

// Constants
// ----------------------------------------------------------------------------
constexpr TickType_t SEMAPHORE_WAIT_NUM_MS = 10;
constexpr TickType_t SEMAPHORE_WAIT_NUM_TICKS = pdMS_TO_TICKS(SEMAPHORE_WAIT_NUM_MS);

// Classes and structs
// ----------------------------------------------------------------------------
class UdpDriver {
public:
    UdpDriver();
    UdpDriver(const ip_addr_t m_ip_addr_src,
              const ip_addr_t m_ip_addr_dest,
              const u16_t m_port_src,
              const u16_t m_port_dest,
              const UdpRawInterface *m_udp_if,
              const OsInterface *m_os_if);
    ~UdpDriver();

    /* User-facing - typically call directly. */
    bool initialize();
    bool setupReceive(udp_recv_fn recv_callback);
    bool receive(uint8_t *rx_array_out, const size_t num_bytes);
    bool transmit(const uint8_t *tx_array_in, const size_t num_bytes);

    /* Utility - public but typically no need to call directly. */
    bool bytesToPacket(
            const uint8_t *byte_array_in,
            const size_t num_bytes,
            struct pbuf *p_pbuf
    ) const;
    u16_t packetToBytes(
            uint8_t *byte_array_out,
            const size_t num_bytes,
            struct pbuf *p_pbuf
    ) const;
    void signalReceiveCplt();
    void waitReceiveCplt();
    void forgetPcb();
    void forgetRecvPbuf();

    /* Accessors - public but typically no need to call directly. */
    void setRecvPbuf(struct pbuf *p_pbuf);
    const ip_addr_t getIpAddrSrc() const;
    const ip_addr_t getIpAddrDest() const;
    const u16_t getPortSrc() const;
    const u16_t getPortDest() const;
    const UdpRawInterface* getUdpIf() const;
    const OsInterface* getOsIf() const;
    struct udp_pcb* getPcb() const;
    struct pbuf* getRecvPbuf() const;

private:
    /* UdpDriver configuration. */
    const ip_addr_t m_ip_addr_src = {0x0};
    const ip_addr_t m_ip_addr_dest = {0x0};
    const u16_t m_port_src = 0;
    const u16_t m_port_dest = 0;

    /* External interfaces. */
    const UdpRawInterface *m_udp_if = nullptr;
    const OsInterface *m_os_if = nullptr;

    /* Data modified internally by the Raw API. */
    /* TODO: decide whether NULL or nullptr, since lwIP API will use NULL when setting these. */
    struct udp_pcb *m_pcb = nullptr;
    struct pbuf *m_recv_pbuf = nullptr;

    /* Synchronization. */
    mutable osSemaphoreId m_recv_semaphore; /* TODO: replace with binary semaphore-style task notification. */
    mutable osStaticSemaphoreDef_t m_recv_semaphore_control_block;
    mutable osMutexId m_recv_pbuf_mutex;
    mutable osStaticMutexDef_t m_recv_pbuf_mutex_control_block;
};

} // end namespace lwip

/**
 * @}
 */
/* end - HeaderUdpDriver */

#endif /* UDP_DRIVER_H */
