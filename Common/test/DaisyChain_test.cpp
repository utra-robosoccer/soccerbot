/**
  *****************************************************************************
  * @file    DaisyChain_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup Test
  * @ingroup  DaisyChain
  * @brief    Unit test driver for TODO
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

TEST_F(DaisyChainShould, SetCallGpioFunctionUponTransmissionRequest){
    DaisyChain chain(p);

    char msg[] = "hey!";

    EXPECT_CALL(gpio, writePin);
    chain.requestTransmission((uint8_t*)msg, sizeof(msg));
}

TEST_F(DaisyChainShould, SetCallGpioFunctionUponReceptionRequest){
    DaisyChain chain(p);

    uint8_t buf[10];

    EXPECT_CALL(gpio, writePin);
    chain.requestReception(buf, 5);
}



} // end anonymous namespace




/**
 * @}
 */
/* end - module name */
