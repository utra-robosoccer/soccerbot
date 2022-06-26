/**
  *****************************************************************************
  * @file
  * @author  Robert Fairley
  *
  * @defgroup udp
  * @brief    Implementation of UdpDriver.
  * @{
  *****************************************************************************
  */

/* TODO: licensing terms for projects we are making use of e.g. googletest? */




/********************************* Includes **********************************/
#include "UdpDriver.h"

using lwip::UdpRawInterface;
using cmsis::OsInterface;
using udp::UdpDriver;


/******************************** File-local *********************************/
namespace {

// Functions
// ----------------------------------------------------------------------------
static void defaultRecvCallback(void *arg,
                                struct udp_pcb *pcb,
                                struct pbuf *p_pbuf,
                                const ip_addr_t *ip_addr,
                                u16_t port)
{
    UdpDriver *self = static_cast<UdpDriver*>(arg);
    self->setRecvPbuf(p_pbuf);
    self->signalReceiveCplt();
}

static bool transmitImpl(UdpDriver* self, struct pbuf * p_pbuf) {
    const ip_addr_t addr = self->getIpAddrDest();
    bool success = false;

    if (self->getUdpIf()->udpConnect(
            const_cast<struct udp_pcb *>(self->getPcb()),
            &addr,
            self->getPortDest()
            ) != ERR_OK)
    {
        goto out;
    }
    if (self->getUdpIf()->udpSend(
            const_cast<struct udp_pcb *>(self->getPcb()),
            p_pbuf
            ) != ERR_OK)
    {
        goto disconnect;
    }

    success = true;

  disconnect:
    self->getUdpIf()->udpDisconnect(
            const_cast<struct udp_pcb *>(self->getPcb())
    );

  out:
    return success;
}

} // end anonymous namespace

namespace udp {

// Public
// ----------------------------------------------------------------------------
UdpDriver::UdpDriver()
{

}

UdpDriver::UdpDriver(
    const ip_addr_t m_ip_addr_src,
    const ip_addr_t m_ip_addr_dest,
    const u16_t m_port_src,
    const u16_t m_port_dest,
    const UdpRawInterface *m_udp_if,
    const OsInterface *m_os_if
) :
    m_ip_addr_src(m_ip_addr_src),
    m_ip_addr_dest(m_ip_addr_dest),
    m_port_src(m_port_src),
    m_port_dest(m_port_dest),
    m_udp_if(m_udp_if),
    m_os_if(m_os_if)
{

}

UdpDriver::~UdpDriver()
{

}

/**
 * @brief initialize UdpDriver and do critical checks.
 * @return false if a check fails, true if initialization completes successfully.
 */
bool UdpDriver::initialize()
{
    if (!getUdpIf() || !getOsIf())
    {
        return false;
    }

    osSemaphoreStaticDef(UdpDriverRecv, &m_recv_semaphore_control_block);
    if ((m_recv_semaphore = getOsIf()->OS_osSemaphoreCreate(
            osSemaphore(UdpDriverRecv),
            1
            )) == NULL)
    {
        return false;
    }

    osMutexStaticDef(UdpDriverRecvPbuf, &m_recv_pbuf_mutex_control_block);
    if ((m_recv_pbuf_mutex = getOsIf()->OS_osMutexCreate(
            osMutex(UdpDriverRecvPbuf)
            )) == NULL)
    {
        return false;
    }

    return true;
}

/**
 * @brief set up UdpDriver to receive UDP packets, calling recvCallback when a packet is received.
 * @param recv_callback callback function to call when lwIP detects a packet has been received.
 * @return true if setup completed successfully, false otherwise.
 */
bool UdpDriver::setupReceive(udp_recv_fn recv_callback)
{
    bool success = false;

    m_pcb = getUdpIf()->udpNew();

    if (!getPcb())
    {
        return false;
    }

    success = (getUdpIf()->udpBind(
            const_cast<struct udp_pcb *>(getPcb()),
            &m_ip_addr_src,
            getPortSrc()
    ) == ERR_OK);

    if (success)
    {
        udp_recv_fn callback = (recv_callback == nullptr)
                ? defaultRecvCallback : recv_callback;
        getUdpIf()->udpRecv(
                const_cast<struct udp_pcb *>(getPcb()),
                callback,
                this
        );
    }
    else
    {
        forgetPcb();
    }

    return success;
}

/**
 * @brief Wait for a packet to be available to read, and read the packet into rxArrayOut. Cleans up the packet read from lwIP.
 * @param rx_array_out array to read packet data payload into.
 * @param num_bytes the maximum number of bytes to read from the packet (should not be greater than the length of rxArrayout).
 * @return true if read packet successfully, false if failed to read the packet into rxArrayOut.
 */
bool UdpDriver::receive(uint8_t *rx_array_out, const size_t num_bytes)
{
    bool success = false;
    struct pbuf *recv_pbuf = nullptr;

    waitReceiveCplt();

    recv_pbuf = getRecvPbuf();

    if (packetToBytes(rx_array_out, num_bytes, recv_pbuf) <= (u16_t) 0)
    {
        goto out;
    }

    success = true;

  out:

    if (recv_pbuf)
    {
        forgetRecvPbuf();
    }

    return success;
}

/**
 * @brief transmit data from txArrayIn through UDP. Allocates and cleans up a new temporary packet internally.
 * @param tx_array_in array holding data to transmit.
 * @param num_bytes number of bytes to transmit.
 * @return true if the data was transmitted successfully, false otherwise.
 */
bool UdpDriver::transmit(const uint8_t *tx_array_in, const size_t num_bytes)
{
    bool success = false;

    /* TODO: see if this can be allocated once at setup, if numBytes is known and unchanging. */
    struct pbuf *temp_pbuf = getUdpIf()->pbufAlloc(
            PBUF_TRANSPORT,
            num_bytes,
            PBUF_RAM
    );
    if (!temp_pbuf)
    {
        goto out;
    }

    if (!bytesToPacket(tx_array_in, num_bytes, temp_pbuf))
    {
        goto out;
    }

    if (!transmitImpl(this, temp_pbuf))
    {
        goto out;
    }

    success = true;

  out:
    if (temp_pbuf)
    {
        /* Error if zero pbufs freed for the one just allocated for Tx. */
        if ((success = getUdpIf()->pbufFree(temp_pbuf) > (u8_t) 0))
        {
            temp_pbuf = nullptr;
        }
    }

    return success;
}

/**
 * @brief Copy the received packet to byteArrayOut, to a maximum of numBytes.
 * @param byte_array_out the array to copy the bytes from the packet to.
 * @param num_bytes the maximum number of bytes to copy (should not be greater than the size of byteArrayOut).
 * @param p_pbuf pointer to the packet (pbuf) to copy the bytes from.
 * @return the number of bytes copied to byte_array_out.
 */
u16_t UdpDriver::packetToBytes(
        uint8_t *byte_array_out,
        const size_t num_bytes,
        struct pbuf *p_pbuf
        ) const
{
    if (!byte_array_out || !p_pbuf)
    {
        return (u16_t) 0;
    }

	return getUdpIf()->pbufCopyPartial(
	        p_pbuf,
	        byte_array_out,
	        num_bytes,
	        0
	);
}

/**
 * @brief Copy num_bytes from byte_array_in to a packet.
 * @param byte_array_in the array to copy the bytes to the packet from.
 * @param num_bytes the number of bytes to copy from byte_array_in.
 * @param p_pbuf pointer to the packet (pbuf) to copy the bytes to.
 * @return true if copied numBytes succesfully, false otherwise.
 */
bool UdpDriver::bytesToPacket(
        const uint8_t *byte_array_in,
        const size_t num_bytes,
        struct pbuf *p_pbuf) const
{
    if (!byte_array_in || !p_pbuf)
    {
        return false;
    }

	return (getUdpIf()->pbufTake(p_pbuf, byte_array_in, num_bytes)
	        == ERR_OK);
}

void UdpDriver::signalReceiveCplt()
{
    getOsIf()->OS_osSemaphoreRelease(m_recv_semaphore);
}

void UdpDriver::waitReceiveCplt()
{
    while (getOsIf()->OS_osSemaphoreWait(
            m_recv_semaphore,
            SEMAPHORE_WAIT_NUM_MS
            ) != osOK)
    {
        ;
    }
}

void UdpDriver::setRecvPbuf(struct pbuf *p_pbuf)
{
    while (getOsIf()->OS_osMutexWait(
            m_recv_pbuf_mutex,
            SEMAPHORE_WAIT_NUM_MS
            ) != osOK)
    {
        ;
    }
    m_recv_pbuf = p_pbuf;
    getOsIf()->OS_osMutexRelease(m_recv_pbuf_mutex);
}

const ip_addr_t UdpDriver::getIpAddrSrc() const
{
    return m_ip_addr_src;
}

const ip_addr_t UdpDriver::getIpAddrDest() const
{
    return m_ip_addr_dest;
}

const u16_t UdpDriver::getPortSrc() const
{
    return m_port_src;
}

const u16_t UdpDriver::getPortDest() const
{
    return m_port_dest;
}

const UdpRawInterface* UdpDriver::getUdpIf() const
{
    return m_udp_if;
}

const OsInterface* UdpDriver::getOsIf() const
{
    return m_os_if;
}

struct udp_pcb* UdpDriver::getPcb() const
{
    return m_pcb;
}

struct pbuf* UdpDriver::getRecvPbuf() const
{
    while (getOsIf()->OS_osMutexWait(
            m_recv_pbuf_mutex,
            SEMAPHORE_WAIT_NUM_MS
            ) != osOK)
    {
        ;
    }
    struct pbuf *p_pbuf = m_recv_pbuf;
    getOsIf()->OS_osMutexRelease(m_recv_pbuf_mutex);
    return p_pbuf;
}

void UdpDriver::forgetRecvPbuf()
{
    while (getOsIf()->OS_osMutexWait(
            m_recv_pbuf_mutex,
            SEMAPHORE_WAIT_NUM_MS
            ) != osOK)
    {
        ;
    }
    getUdpIf()->pbufFree(m_recv_pbuf);
    setRecvPbuf(nullptr);
    getOsIf()->OS_osMutexRelease(m_recv_pbuf_mutex);
}

void UdpDriver::forgetPcb()
{
    getUdpIf()->udpRemove(const_cast<struct udp_pcb *>(getPcb()));
    m_pcb = nullptr;
}

} // end namespace udp

/**
 * @}
 */
/* end - udp */
