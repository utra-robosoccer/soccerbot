/**
  *****************************************************************************
  * @file
  * @author  Robert Fairley
  * @brief   Wrapper interface for UDP function calls from the lwIP Raw API.
  *
  * @defgroup HeaderUdpRawInterface
  * @defgroup lwip
  * @{
  *****************************************************************************
  */

#ifndef UDP_RAW_INTERFACE_H
#define UDP_RAW_INTERFACE_H

#include <stdint.h>

#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/arch.h"

namespace lwip {

class UdpRawInterface {
public:
    virtual ~UdpRawInterface() {}
    virtual struct udp_pcb *udpNew() const = 0;
    virtual err_t udpBind(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
            u16_t port) const = 0;
    virtual void udpRecv(struct udp_pcb *pcb, udp_recv_fn recv,
            void *recv_arg) const = 0;
    virtual void udpRemove(struct udp_pcb *pcb) const = 0;
    virtual err_t udpConnect(struct udp_pcb *pcb, const ip_addr_t *ipaddr,
            u16_t port) const = 0;
    virtual err_t udpSend(struct udp_pcb *pcb, struct pbuf *p) const = 0;
    virtual void udpDisconnect(struct udp_pcb *pcb) const = 0;
    virtual u8_t pbufFree(struct pbuf *p) const = 0;
    virtual struct pbuf* pbufAlloc(pbuf_layer layer, u16_t length, pbuf_type type) const = 0;
    virtual u16_t pbufCopyPartial(const struct pbuf *buf, void *dataptr, u16_t len, u16_t offset) const = 0;
    virtual err_t pbufTake(struct pbuf *buf, const void *dataptr, u16_t len) const = 0;
};

} // end namespace lwip

/**
 * @}
 */
/* end - HeaderUdpRawInterface */

#endif /* UDP_INTERFACE_H */
