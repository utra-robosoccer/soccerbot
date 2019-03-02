/**
  *****************************************************************************
  * @author  Robert Fairley
  * @brief   Implements UdpRawInterfaceImpl.
  *
  * @defgroup lwip
  * @{
  *****************************************************************************
  */

#include "UdpRawInterfaceImpl.h"

namespace lwip {

struct udp_pcb* UdpRawInterfaceImpl::udpNew() const {
    return udp_new();
}

err_t UdpRawInterfaceImpl::udpBind(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
        u16_t port) const {
    return udp_bind(pcb, ipaddr, port);
}

void UdpRawInterfaceImpl::udpRecv(struct udp_pcb *pcb, udp_recv_fn recv,
        void *recv_arg) const {
    udp_recv(pcb, recv, recv_arg);
}

void UdpRawInterfaceImpl::udpRemove(struct udp_pcb *pcb) const {
    udp_remove(pcb);
}

err_t UdpRawInterfaceImpl::udpConnect(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
        u16_t port) const {
    return udp_connect(pcb, ipaddr, port);
}

err_t UdpRawInterfaceImpl::udpSend(struct udp_pcb *pcb, struct pbuf *p) const {
    return udp_send(pcb, p);
}

void UdpRawInterfaceImpl::udpDisconnect(struct udp_pcb *pcb) const {
    udp_disconnect(pcb);
}

u8_t UdpRawInterfaceImpl::pbufFree(struct pbuf *p) const {
    return pbuf_free(p);
}

struct pbuf* UdpRawInterfaceImpl::pbufAlloc(pbuf_layer layer, u16_t length, pbuf_type type) const {
    return pbuf_alloc(layer, length, type);
}

u16_t UdpRawInterfaceImpl::pbufCopyPartial(const struct pbuf *buf, void *dataptr, u16_t len, u16_t offset) const {
    return pbuf_copy_partial(buf, dataptr, len, offset);
}

err_t UdpRawInterfaceImpl::pbufTake(struct pbuf *buf, const void *dataptr, u16_t len) const {
    return pbuf_take(buf, dataptr, len);
}

} // end namespace lwip

/**
 * @}
 */
/* end - lwip */
