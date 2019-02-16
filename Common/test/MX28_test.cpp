/**
  *****************************************************************************
  * @file    MX28_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup MX28_Test
  * @ingroup  Dynamixel
  * @brief    MX28 test driver
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "Dynamixel/MX28.h"

#include "MockUartInterface.h"
#include "MockOsInterface.h"
#include "MockGpioInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::_;

using uart::UartDriver;

using mocks::MockOsInterface;
using mocks::MockUartInterface;
using mocks::MockGpioInterface;

using dynamixel::DaisyChainParams;
using dynamixel::DaisyChain;
using dynamixel::ResolutionDivider;
using dynamixel::MX28;




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
class MX28Test : public ::testing::Test {
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
TEST_F(MX28Test, CanBeCreated){
    DaisyChain chain(p);
    MX28 m(1, &chain);
}

TEST_F(MX28Test, setBaudRateBoundsCheckPasses){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    m.setBaudRate(3500000);
    m.setBaudRate(dynamixel::MX28_DEFAULT_BAUD_RATE);
    m.setBaudRate(9600);

    ASSERT_FALSE(m.setBaudRate(9600 - 1));
    ASSERT_FALSE(m.setBaudRate(3500000 + 1));
}

TEST_F(MX28Test, setGoalVelocityBoundsCheckPasses){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    m.setGoalVelocity(dynamixel::MX28_MAX_VELOCITY);
    m.setGoalVelocity(dynamixel::MIN_VELOCITY);

    ASSERT_FALSE(m.setGoalVelocity(dynamixel::MX28_MAX_VELOCITY + 1));
    ASSERT_FALSE(m.setGoalVelocity(dynamixel::MIN_VELOCITY - 1));
}

TEST_F(MX28Test, CanSetGoalAcceleration){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    m.setGoalAcceleration(10.0);
}

TEST_F(MX28Test, CanSetDGain){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    m.setDGain(128);
}

TEST_F(MX28Test, CanSetIGain){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    m.setIGain(128);
}

TEST_F(MX28Test, CanSetPGain){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    m.setPGain(128);
}

TEST_F(MX28Test, CanGetVelocity){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    float currentVelocity = 0;
    m.getVelocity(currentVelocity);
}

} // end anonymous namespace




/**
 * @}
 */
/* end - MX28_Test */
