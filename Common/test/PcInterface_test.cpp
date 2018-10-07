/**
*****************************************************************************
* @file    PcInterface_test.cpp
* @author  Robert Fairley
* @brief   Unit tests for PcInterface.
*
* @defgroup pc_interface_test
* @brief    Unit tests and mocking for the PcInterface class.
* @{
*****************************************************************************
*/

// TODO: add license terms from external projects e.g. googletest to our files too?
// NOTE: consider just doing using namespace pc_interface; ?
#include <cstdint>

#include "PcInterface.h"

// TODO: for PcInterface to work on F4, need to have if defined(board) to include this or not
#include "MockUdpInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#define NON_NULL_PTR 0x1

using ::testing::Return;
using ::testing::_;

// TODO: doxygen documentation for functions once complete

namespace {

// MemberProtocolDefaultInitializesToUDP tests that the default
// constructor of PcInterface initializes the member protocol to UDP.
// This is to make sure protocol is initialized to *something*.
TEST(PcInterfaceTests, MemberProtocolDefaultInitializesToUDP) {
    pc_interface::PcInterface pcInterfaceTestObject;
    ASSERT_EQ(pc_interface::PcProtocol::UDP,
            pcInterfaceTestObject.getProtocol());
}

TEST(PcInterfaceTests, MemberUdpDriverDefaultInitializesToNull) {
    pc_interface::PcInterface pcInterfaceTestObject;
    ASSERT_EQ(nullptr, pcInterfaceTestObject.getUdpDriver());
}

TEST(PcInterfaceTests, MemberUdpDriverParameterizedInitializesToNull) {
    pc_interface::PcInterface pcInterfaceTestObject(
            pc_interface::PcProtocol::UDP);
    ASSERT_EQ(nullptr, pcInterfaceTestObject.getUdpDriver());
}

TEST(PcInterfaceTests, MemberUdpDriverCanSetAndGet) {
    pc_interface::PcInterface pcInterfaceTestObject;
    udp_driver::UdpDriver udpDriverTestObject;
    ASSERT_TRUE(pcInterfaceTestObject.setUdpDriver(&udpDriverTestObject));
    ASSERT_EQ(&udpDriverTestObject, pcInterfaceTestObject.getUdpDriver());
}

// MemberProtocolCanInitializeAndGet tests that a constructor can
// initialize and get back the protocol member of PcInterface with
// all of the values defined in type enum Protocol_e.
TEST(PcInterfaceTests, MemberProtocolCanInitializeAndGet) {
    for (int iProtocol = static_cast<int>(pc_interface::PcProtocol::UDP);
            iProtocol <= static_cast<int>(pc_interface::PcProtocol::USB_UART);
            iProtocol++) {
        pc_interface::PcProtocol protocol = (pc_interface::PcProtocol) iProtocol;
        pc_interface::PcInterface pcInterfaceTestObject(protocol);

        ASSERT_EQ(protocol, pcInterfaceTestObject.getProtocol());
    }
}

// FIXME: an osInterface needs to be set in pcInterfaceTestObject in all of these.
// For now, commenting it out as the way it is set will probably change to
// doing it in the parameterized constructor rather than by member function.

//// MemberRxBufferDefaultInitializesToZero tests that the default
//// constructor initializes the rxBuffer entries to zeroes.
//TEST(PcInterfaceTests, MemberRxBufferDefaultInitializesToZero) {
//    pc_interface::PcInterface pcInterfaceTestObject;
//    uint8_t rxArray[pc_interface::PC_INTERFACE_BUFFER_SIZE];
//    bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
//    ASSERT_TRUE(success);
//
//    for (int iRxArray = 0; iRxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE;
//            iRxArray++) {
//        ASSERT_EQ(rxArray[iRxArray], 0);
//    }
//}
//
//// MemberRxBufferParameterizedInitializesToZero tests that the
//// parameterized constructor initializes the rxBuffer entries to zeroes.
//TEST(PcInterfaceTests, MemberRxBufferParameterizedInitalizesToZero) {
//    pc_interface::PcInterface pcInterfaceTestObject(
//            pc_interface::PcProtocol::UDP);
//    uint8_t rxArray[pc_interface::PC_INTERFACE_BUFFER_SIZE];
//    bool success = pcInterfaceTestObject.getRxBuffer(rxArray);
//    ASSERT_TRUE(success);
//
//    for (int iRxArray = 0; iRxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE;
//            iRxArray++) {
//        ASSERT_EQ(rxArray[iRxArray], 0);
//    }
//}
//
//// MemberTxBufferDefaultInitializesToZero tests that the default
//// constructor initializes the txBuffer entries to zeroes.
//TEST(PcInterfaceTests, MemberTxBufferDefaultInitializesToZero) {
//    pc_interface::PcInterface pcInterfaceTestObject;
//    uint8_t txArray[pc_interface::PC_INTERFACE_BUFFER_SIZE];
//    bool success = pcInterfaceTestObject.getTxBuffer(txArray);
//    ASSERT_TRUE(success);
//
//    for (int iTxArray = 0; iTxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE;
//            iTxArray++) {
//        ASSERT_EQ(txArray[iTxArray], 0);
//    }
//}
//
//// MemberTxBufferParameterizedInitializesToZero tests that the
//// parameterized constructor initializes the txBuffer entries to zeroes.
//TEST(PcInterfaceTests, MemberTxBufferParameterizedInitalizesToZero) {
//    pc_interface::PcInterface pcInterfaceTestObject(
//            pc_interface::PcProtocol::UDP);
//    uint8_t txArray[pc_interface::PC_INTERFACE_BUFFER_SIZE];
//    bool success = pcInterfaceTestObject.getTxBuffer(txArray);
//    ASSERT_TRUE(success);
//
//    for (int iTxArray = 0; iTxArray < pc_interface::PC_INTERFACE_BUFFER_SIZE;
//            iTxArray++) {
//        ASSERT_EQ(txArray[iTxArray], 0);
//    }
//}

// TODO: tests for failing on every mock function call - all possible combinations?
// TODO: add a test for setting TxBuffer and getting from RxBuffer. (use a test socket to set up the data)
// TODO: test coverage for LwipUdpInterface, set up fixtures to test passing
//       data between rx/txBuffers and pbufs
// TODO: more test coverage after smoke test when the interface (.h) of PcInterface is more settled

}

/**
 * @}
 */
/* end - pc_interface_test */
