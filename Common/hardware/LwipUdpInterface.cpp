/**
  *****************************************************************************
  * @file    LwipUdpInterface.cpp
  * @author  Robert Fairley
  * @brief   Implements Lwip/UDP interface-facing functions, and means to access network stack data.
  *
  * @defgroup lwip_udp_interface
  * @brief    Provides an abstraction layer to the LwIP network stack, so that network parameters and actions are hidden from other components, and means to access only necessary data is provided.
  * @{
  *****************************************************************************
  */

#include <LwipUdpInterface.h>

namespace lwip_udp_interface {

struct udp_pcb* LwipUdpInterface::udpNew() const {
    struct udp_pcb *pcb = udp_new();
    return pcb;
}

err_t LwipUdpInterface::udpBind(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
        u16_t port) const {
    return udp_bind(pcb, IP_ADDR_ANY, port);
}

// TODO: should implement error checking here
void LwipUdpInterface::udpRecv(struct udp_pcb *pcb, udp_recv_fn recv,
        void *recv_arg) const {
    udp_recv(pcb, recv, recv_arg);
}

// TODO: should implement error checking here
void LwipUdpInterface::udpRemove(struct udp_pcb *pcb) const {
    udp_remove(pcb);
}

// TODO: should implement error checking here
void LwipUdpInterface::ethernetifInput(void const * argument) const {
    ethernetif_input(argument);
}

err_t LwipUdpInterface::udpConnect(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
        u16_t port) const {
    return udp_connect(pcb, ipaddr, port);
}

// TODO: should implement error checking here
err_t LwipUdpInterface::udpSend(struct udp_pcb *pcb, struct pbuf *p) const {
    return udp_send(pcb, p);
}

// TODO: should implement error checking here
void LwipUdpInterface::udpDisconnect(struct udp_pcb *pcb) const {
    udp_disconnect(pcb);
}

// TODO: should implement error checking here
u8_t LwipUdpInterface::pbufFree(struct pbuf *p) const {
    return pbuf_free(p);
}

} // end namespace lwip_udp_interface

/**
 * @}
 */
/* end - lwip_udp_interface */
