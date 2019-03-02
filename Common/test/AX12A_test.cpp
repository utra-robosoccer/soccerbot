/**
  *****************************************************************************
  * @file    AX12A_test.cpp
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

#include "MockUartInterface.h"
#include "MockOsInterface.h"
#include "MockGpioInterface.h"
#include "UartDriver/UartDriver.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::_;

using cmsis::gmock::MockOsInterface;
using hal::gmock::MockUartInterface;
using hal::gmock::MockGpioInterface;

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
    MockUartInterface uart;
    MockOsInterface os;
    MockGpioInterface gpio;
    DaisyChainParams p;
};




// Functions
// ----------------------------------------------------------------------------
TEST_F(AX12ATest, CanBeCreated){
    DaisyChain chain(p);
    AX12A m(1, &chain);
}

TEST_F(AX12ATest, setBaudRateBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setBaudRate(dynamixel::AX12A_DEFAULT_BAUD_RATE);
    m.setBaudRate(7844);

    ASSERT_FALSE(m.setBaudRate(1000000 + 1));
    ASSERT_FALSE(m.setBaudRate(7844 - 1));
}

TEST_F(AX12ATest, setGoalVelocityBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setGoalVelocity(dynamixel::AX12A_MAX_VELOCITY);
    m.setGoalVelocity(dynamixel::MIN_VELOCITY);

    ASSERT_FALSE(m.setGoalVelocity(dynamixel::AX12A_MAX_VELOCITY + 1));
    ASSERT_FALSE(m.setGoalVelocity(dynamixel::MIN_VELOCITY - 1));
}

TEST_F(AX12ATest, CanSetCwComplianceMargin){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    uint8_t cwMargin = 1; // 0.29 degrees
    m.setCwComplianceMargin(cwMargin);
    m.setCwComplianceMargin(dynamixel::AX12A_DEFAULT_CW_COMPLIANCE_MARGIN);
}

TEST_F(AX12ATest, CanSetCcwComplianceMargin){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    uint8_t cwMargin = 8; // 2.32 degrees
    m.setCcwComplianceMargin(cwMargin);
    m.setCcwComplianceMargin(dynamixel::AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN);
}

TEST_F(AX12ATest, setCwComplianceSlopeBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setCwComplianceSlope(dynamixel::AX12A_DEFAULT_CW_COMPLIANCE_SLOPE);

    ASSERT_FALSE(m.setCwComplianceSlope(8));
}

TEST_F(AX12ATest, setCcwComplianceSlopeBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setCcwComplianceSlope(dynamixel::AX12A_DEFAULT_CW_COMPLIANCE_SLOPE);

    ASSERT_FALSE(m.setCcwComplianceSlope(8));
}

TEST_F(AX12ATest, setComplianceSlopeBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setComplianceSlope(5);

    ASSERT_FALSE(m.setComplianceSlope(8));
}

TEST_F(AX12ATest, CanSetComplianceMargin){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    uint8_t margin = 4; // 1.16 degrees
    m.setComplianceMargin(margin);
}

TEST_F(AX12ATest, CanGetVelocity){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    float currentVelocity = 0;
    m.getVelocity(currentVelocity);
}

} // end anonymous namespace




/**
 * @}
 */
/* end - AX12A_Test */
