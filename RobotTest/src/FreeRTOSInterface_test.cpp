
/**
  *****************************************************************************
  * @file    FreeRTOSInterface_test.cpp
  * @author  Izaak Niksan
  * @brief   Source file for FreeRTOS testing and mocking
  *
  * @defgroup FreeRTOS_Interface_test
  * @brief    The pc_interface_tests module contains the structures required for running tests and mocking, and the tests themselves.
  * @{
  *****************************************************************************
  */


namespace {

class MockUdpInterface : public udp_interface::UdpInterface {
public:
    MOCK_METHOD0(udpNew, bool());
    MOCK_METHOD0(udpBind, bool());
    MOCK_METHOD0(udpRecv, bool());
    MOCK_METHOD0(udpRemove, bool());
    MOCK_METHOD0(ethernetifInput, bool());
    MOCK_METHOD0(udpConnect, bool());
    MOCK_METHOD0(udpSend, bool());
    MOCK_METHOD0(udpDisconnect, bool());
    MOCK_METHOD0(pbufFreeRx, bool());
    MOCK_METHOD0(pbufFreeTx, bool());
    MOCK_METHOD0(waitRecv, bool());
    MOCK_METHOD1(packetToBytes, bool(uint8_t*));
    MOCK_METHOD1(bytesToPacket, bool(const uint8_t*));
};

}

/**
 * @}
 */
/* end - FreeRTOS_Interface_test*/
