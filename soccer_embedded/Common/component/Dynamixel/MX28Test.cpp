/**
  *****************************************************************************
  * @file
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

#include "UartDriver/UartDriver.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "GpioInterfaceMock.h"
#include "OsInterfaceMock.h"
#include "UartInterfaceMock.h"

using ::testing::_;
using ::testing::NiceMock;

using uart::UartDriver;

using cmsis::gmock::OsInterfaceMock;
using hal::gmock::UartInterfaceMock;
using hal::gmock::GpioInterfaceMock;

using dynamixel::DaisyChainParams;
using dynamixel::DaisyChain;
using dynamixel::ResolutionDivider;
using dynamixel::MX28;




/******************************** File-local *********************************/
namespace{
// Variables
// ----------------------------------------------------------------------------
UART_HandleTypeDef UARTx = {0};
GPIO_TypeDef dataDirPort;





// Classes & structs
// ----------------------------------------------------------------------------
class MX28Test : public ::testing::Test {
protected:
    void SetUp() override {
        UARTxDriver = new UartDriver(&os, &uart, &UARTx);
        p.uartDriver = UARTxDriver;
        p.gpioif = &gpio;
        p.dataDirPort = &dataDirPort;
        p.dataDirPinNum = 1;
    }

    void TearDown() override {
        delete UARTxDriver;
    }

    UartDriver *UARTxDriver = nullptr;
    NiceMock<UartInterfaceMock> uart;
    NiceMock<OsInterfaceMock> os;
    NiceMock<GpioInterfaceMock> gpio;
    DaisyChainParams p;
};




// Functions
// ----------------------------------------------------------------------------
TEST_F(MX28Test, setBaudRateBoundsCheckPasses){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    ASSERT_TRUE(m.setBaudRate(3500000));
    ASSERT_TRUE(m.setBaudRate(dynamixel::MX28_DEFAULT_BAUD_RATE));
    ASSERT_TRUE(m.setBaudRate(9600));

    ASSERT_FALSE(m.setBaudRate(9600 - 1));
    ASSERT_FALSE(m.setBaudRate(3500000 + 1));
}

TEST_F(MX28Test, setGoalVelocityBoundsCheckPasses){
    DaisyChain chain(p);
    MX28 m(1, &chain);

    ASSERT_TRUE(m.setGoalVelocity(dynamixel::MX28_MAX_VELOCITY));
    ASSERT_TRUE(m.setGoalVelocity(dynamixel::MIN_VELOCITY));

    ASSERT_FALSE(m.setGoalVelocity(dynamixel::MX28_MAX_VELOCITY + 1));
    ASSERT_FALSE(m.setGoalVelocity(dynamixel::MIN_VELOCITY - 1));
}

} // end anonymous namespace




/**
 * @}
 */
/* end - MX28_Test */
