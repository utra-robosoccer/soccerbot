/**
  *****************************************************************************
  * @file    MockUdpInterface.h
  * @author  Robert Fairley
  * @brief   Mock class for UdpInterface using gmock.
  *
  * @defgroup Header
  * @ingroup  udp_interface
  * @{
  *****************************************************************************
  */

#ifndef MOCK_UDP_INTERFACE_H
#define MOCK_UDP_INTERFACE_H

#include "UdpInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace lwip {
namespace gmock {

class MockUdpInterface: public UdpInterface {
public:
    MOCK_CONST_METHOD0(udpNew, struct udp_pcb*());
    MOCK_CONST_METHOD3(udpBind, err_t(struct udp_pcb*, const ip_addr_t*, u16_t));
    MOCK_CONST_METHOD3(udpRecv, void(struct udp_pcb*, udp_recv_fn, void*));
    MOCK_CONST_METHOD1(udpRemove, void(struct udp_pcb*));
    MOCK_CONST_METHOD3(udpConnect, err_t(struct udp_pcb*, const ip_addr_t*, u16_t));
    MOCK_CONST_METHOD2(udpSend, err_t(struct udp_pcb*, struct pbuf*));
    MOCK_CONST_METHOD1(udpDisconnect, void(struct udp_pcb*));
    MOCK_CONST_METHOD1(pbufFree, u8_t(struct pbuf*));
    MOCK_CONST_METHOD3(pbufAlloc, struct pbuf*(pbuf_layer, u16_t, pbuf_type));
    MOCK_CONST_METHOD4(pbufCopyPartial, u16_t(const struct pbuf *, void *, u16_t, u16_t));
    MOCK_CONST_METHOD3(pbufTake, err_t(struct pbuf *, const void *, u16_t));
};

} // end namespace gmock
} // end namespace lwip

#endif
