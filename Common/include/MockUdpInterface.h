#ifndef MOCK_UDP_INTERFACE_H
#define MOCK_UDP_INTERFACE_H

#include "UdpInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace MOCKS {

class MockUdpInterface: public udp_interface::UdpInterface {
public:
    MOCK_CONST_METHOD0(udpNew, struct udp_pcb*());
    MOCK_CONST_METHOD3(udpBind, err_t(struct udp_pcb*, const ip_addr_t*, u16_t));
    MOCK_CONST_METHOD3(udpRecv, void(struct udp_pcb*, udp_recv_fn, void*));
    MOCK_CONST_METHOD1(udpRemove, void(struct udp_pcb*));
    MOCK_CONST_METHOD1(ethernetifInput, void(void const*));
    MOCK_CONST_METHOD3(udpConnect, err_t(struct udp_pcb*, const ip_addr_t*, u16_t));
    MOCK_CONST_METHOD2(udpSend, err_t(struct udp_pcb*, struct pbuf*));
    MOCK_CONST_METHOD1(udpDisconnect, void(struct udp_pcb*));
    MOCK_CONST_METHOD1(pbufFree, u8_t(struct pbuf*));
};

}

#endif
