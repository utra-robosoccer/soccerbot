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
#include "AX12A.h"

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
using dynamixel::AX12A;




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
class AX12ATest : public ::testing::Test {
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
TEST_F(AX12ATest, CanBeCreated){
    DaisyChain chain(p);
    AX12A m(1, &chain);
}

TEST_F(AX12ATest, setBaudRateBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setBaudRate(1000000);
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

TEST_F(AX12ATest, CanGetVelocity){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    float currentVelocity = 0;
    m.getVelocity(currentVelocity);
}

TEST_F(AX12ATest, setCwComplianceMarginBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    uint8_t cwMargin = 1; // 0.25 degrees
    m.setCwComplianceMargin(cwMargin);
    m.setCwComplianceMargin(dynamixel::AX12A_DEFAULT_CW_COMPLIANCE_MARGIN);

    ASSERT_FALSE(m.setComplianceMargin(0));
}

TEST_F(AX12ATest, setCcwComplianceMarginBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    uint8_t cwMargin = 8; // 2 degrees
    m.setCcwComplianceMargin(cwMargin);
    m.setCcwComplianceMargin(dynamixel::AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN);

    ASSERT_FALSE(m.setComplianceMargin(0));
}

TEST_F(AX12ATest, CanSetCwComplianceSlope){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setCwComplianceSlope();
}

TEST_F(AX12ATest, CanSetCcwComplianceSlope){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setCcwComplianceSlope();
}

TEST_F(AX12ATest, CanSetComplianceSlope){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    m.setComplianceSlope();
}

TEST_F(AX12ATest, setComplianceMarginBoundsCheckPasses){
    DaisyChain chain(p);
    AX12A m(1, &chain);

    uint8_t margin = 4; // 1 degree
    m.setComplianceMargin(margin);

    ASSERT_FALSE(m.setComplianceMargin(0));
}

} // end anonymous namespace




/**
 * @}
 */
/* end - AX12A_Test */
