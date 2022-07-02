/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  *
  * @defgroup AX12A_Test
  * @ingroup  Dynamixel
  * @brief    AX12A test driver
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "Dynamixel/AX12A.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "GpioInterfaceMock.h"
#include "OsInterfaceMock.h"
#include "UartDriver/UartDriver.h"
#include "UartInterfaceMock.h"

using ::testing::_;
using ::testing::NiceMock;

using cmsis::gmock::OsInterfaceMock;
using hal::gmock::UartInterfaceMock;
using hal::gmock::GpioInterfaceMock;

using dynamixel::DaisyChainParams;
using dynamixel::DaisyChain;
using dynamixel::ResolutionDivider;
using dynamixel::AX12A;
using uart::UartDriver;




/******************************** File-local *********************************/
namespace{
// Variables
// ----------------------------------------------------------------------------
UART_HandleTypeDef UARTx = {0};
GPIO_TypeDef dataDirPort;



// Classes & structs
// ----------------------------------------------------------------------------
class AX12ATest : public ::testing::Test {
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
TEST_F(AX12ATest, setBaudRateBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    ASSERT_TRUE(m.setBaudRate(dynamixel::AX12A_DEFAULT_BAUD_RATE));
    ASSERT_TRUE(m.setBaudRate(7844));

    ASSERT_FALSE(m.setBaudRate(1000000 + 1));
    ASSERT_FALSE(m.setBaudRate(7844 - 1));
}

TEST_F(AX12ATest, setGoalVelocityBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    ASSERT_TRUE(m.setGoalVelocity(dynamixel::AX12A_MAX_VELOCITY));
    ASSERT_TRUE(m.setGoalVelocity(dynamixel::MIN_VELOCITY));

    ASSERT_FALSE(m.setGoalVelocity(dynamixel::AX12A_MAX_VELOCITY + 1));
    ASSERT_FALSE(m.setGoalVelocity(dynamixel::MIN_VELOCITY - 1));
}

TEST_F(AX12ATest, setCwComplianceSlopeBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    ASSERT_TRUE(m.setCwComplianceSlope(dynamixel::AX12A_DEFAULT_CW_COMPLIANCE_SLOPE));

    ASSERT_FALSE(m.setCwComplianceSlope(8));
}

TEST_F(AX12ATest, setCcwComplianceSlopeBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    ASSERT_TRUE(m.setCcwComplianceSlope(dynamixel::AX12A_DEFAULT_CW_COMPLIANCE_SLOPE));

    ASSERT_FALSE(m.setCcwComplianceSlope(8));
}

TEST_F(AX12ATest, setComplianceSlopeBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    ASSERT_TRUE(m.setComplianceSlope(5));

    ASSERT_FALSE(m.setComplianceSlope(8));
}

} // end anonymous namespace




/**
 * @}
 */
/* end - AX12A_Test */
