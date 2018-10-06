/**
 *****************************************************************************
 * @file    LwipUdpInterface.h
 * @author  Robert Fairley
 * @brief   Defines the interface through which UDP and required Lwip network stack data/functions will be accessed.
 *
 * @defgroup Header
 * @ingroup  lwip_udp_interface
 * @{
 *****************************************************************************
 */

#ifndef LWIP_UDP_INTERFACE_H
#define LWIP_UDP_INTERFACE_H

#include <cstdint>
#include <UdpInterface.h>

namespace lwip_udp_interface {

class LwipUdpInterface: public udp_interface::UdpInterface {
public:
    struct udp_pcb *udpNew() const override final;
    err_t udpBind(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
            u16_t port) const override final;
    void udpRecv(struct udp_pcb *pcb, udp_recv_fn recv, void *recv_arg) const
            override final;
    void udpRemove(struct udp_pcb *pcb) const override final;
    void ethernetifInput(void const * argument) const override final;
    err_t udpConnect(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
            u16_t port) const override final;
    err_t udpSend(struct udp_pcb *pcb, struct pbuf *p) const override final;
    void udpDisconnect(struct udp_pcb *pcb) const override final;
    u8_t pbufFree(struct pbuf *p) const override final;
};

} // end namespace lwip_udp_interface

/**
 * @}
 */
/* end - Header */

#endif /* LWIP_UDP_INTERFACE_H */
