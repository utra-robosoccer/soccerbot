/**
  *****************************************************************************
  * @file    LwipUdpInterface.cpp
  * @author  Robert Fairley
  * @brief   Implements LwipUdpInterface, making direct calls to UDP functions of the lwIP Raw API.
  *
  * @defgroup lwip_udp_interface
  * @{
  *****************************************************************************
  */

#include <LwipUdpInterface.h>

namespace lwip_udp_interface {

struct udp_pcb* LwipUdpInterface::udpNew() const {
    return udp_new();
}

err_t LwipUdpInterface::udpBind(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
        u16_t port) const {
    return udp_bind(pcb, ipaddr, port);
}

void LwipUdpInterface::udpRecv(struct udp_pcb *pcb, udp_recv_fn recv,
        void *recv_arg) const {
    udp_recv(pcb, recv, recv_arg);
}

void LwipUdpInterface::udpRemove(struct udp_pcb *pcb) const {
    udp_remove(pcb);
}

err_t LwipUdpInterface::udpConnect(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
        u16_t port) const {
    return udp_connect(pcb, ipaddr, port);
}

err_t LwipUdpInterface::udpSend(struct udp_pcb *pcb, struct pbuf *p) const {
    return udp_send(pcb, p);
}

void LwipUdpInterface::udpDisconnect(struct udp_pcb *pcb) const {
    udp_disconnect(pcb);
}

u8_t LwipUdpInterface::pbufFree(struct pbuf *p) const {
    return pbuf_free(p);
}

struct pbuf* LwipUdpInterface::pbufAlloc(pbuf_layer layer, u16_t length, pbuf_type type) const {
    return pbuf_alloc(layer, length, type);
}

u16_t LwipUdpInterface::pbufCopyPartial(const struct pbuf *buf, void *dataptr, u16_t len, u16_t offset) const {
    return pbuf_copy_partial(buf, dataptr, len, offset);
}

err_t LwipUdpInterface::pbufTake(struct pbuf *buf, const void *dataptr, u16_t len) const {
    return pbuf_take(buf, dataptr, len);
}

} // end namespace lwip_udp_interface

/**
 * @}
 */
/* end - lwip_udp_interface */
