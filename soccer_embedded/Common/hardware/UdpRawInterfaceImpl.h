/**
  *****************************************************************************
  * @file
  * @author  Robert Fairley
  * @brief   Extend UdpInterface to make the direct calls to the lwIP Raw API.
  *
  * @defgroup Header
  * @ingroup  lwip
  * @{
  *****************************************************************************
  */

#ifndef UDP_RAW_INTERFACE_IMPL_H
#define UDP_RAW_INTERFACE_IMPL_H

#include <stdint.h>
#include "UdpRawInterface.h"

namespace lwip {

class UdpRawInterfaceImpl: public lwip::UdpRawInterface {
public:
    struct udp_pcb *udpNew() const override final;
    err_t udpBind(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
            u16_t port) const override final;
    void udpRecv(struct udp_pcb *pcb, udp_recv_fn recv, void *recv_arg) const
            override final;
    void udpRemove(struct udp_pcb *pcb) const override final;
    err_t udpConnect(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
            u16_t port) const override final;
    err_t udpSend(struct udp_pcb *pcb, struct pbuf *p) const override final;
    void udpDisconnect(struct udp_pcb *pcb) const override final;

    /* TODO: split off pbuf-related functions to their own mockable interface. */
    u8_t pbufFree(struct pbuf *p) const override final;
    struct pbuf* pbufAlloc(pbuf_layer layer, u16_t length, pbuf_type type) const override final;
    u16_t pbufCopyPartial(const struct pbuf *buf, void *dataptr, u16_t len, u16_t offset) const override final;
    err_t pbufTake(struct pbuf *buf, const void *dataptr, u16_t len) const override final;
};

} // end namespace lwip

/**
 * @}
 */
/* end - Header */

#endif /* UDP_RAW_INTERFACE_IMPL_H */
