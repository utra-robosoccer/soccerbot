/**
  *****************************************************************************
  * @file    DaisyChain_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup DaisyChain_Test
  * @ingroup  DaisyChain
  * @brief    Unit test driver for DaisyChain
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "DaisyChain.h"

#include "MockUartInterface.h"
#include "MockOsInterface.h"
#include "MockGpioInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::DoAll;
using ::testing::Return;
using ::testing::_;

using uart::UartDriver;
using mocks::MockOsInterface;
using mocks::MockUartInterface;
using mocks::MockGpioInterface;

using dynamixel::DaisyChainParams;
using dynamixel::DaisyChain;




/******************************** File-local *********************************/
namespace{
// Variables
// ----------------------------------------------------------------------------
MockUartInterface uart;
MockOsInterface os;
MockGpioInterface gpio;

UART_HandleTypeDef UARTx = {0};
UartDriver UARTxDriver(&os, &uart, &UARTx);
GPIO_TypeDef dataDirPort;

DaisyChainParams p;




// Classes & structs
// ----------------------------------------------------------------------------
class DaisyChainShould : public ::testing::Test {
protected:
    void SetUp() override {
        p.uartDriver = &UARTxDriver;
        p.gpioif = &gpio;
        p.dataDirPort = &dataDirPort;
        p.dataDirPinNum = 1;
    }
};




// Functions
// ----------------------------------------------------------------------------
TEST_F(DaisyChainShould, Construct){
    DaisyChain chain(p);
}

// We cannot really test much more than this for transmit without totally
// whiteboxing the test, which isn't good as we may change the implementation
// soon to buffer the requests then package them as a sync write command
TEST_F(DaisyChainShould, CallSetGpioFunctionUponTransmissionRequest){
    DaisyChain chain(p);

    char msg[] = "hey!";

    EXPECT_CALL(gpio, writePin);
    auto status = chain.requestTransmission((uint8_t*)msg, sizeof(msg));
    EXPECT_TRUE(status);
}

// We cannot really test much more than this for transmit without totally
// whiteboxing the test, which doesn't tell us anything useful in this case
TEST_F(DaisyChainShould, CallSetGpioFunctionUponReceptionRequest){
    DaisyChain chain(p);

    uint8_t buf[10];

    EXPECT_CALL(gpio, writePin);
    auto status = chain.requestReception(buf, 5);
    EXPECT_TRUE(status);
}



} // end anonymous namespace




/**
 * @}
 */
/* end - DaisyChain_Test */
