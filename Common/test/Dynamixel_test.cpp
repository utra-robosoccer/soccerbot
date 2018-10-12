/**
  *****************************************************************************
  * @file    Dynamixel_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup Test
  * @ingroup  Dynamixel
  * @brief    Dynamixel test driver
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "MockMotor.h"

#include "MockUartInterface.h"
#include "MockOsInterface.h"
#include "MockGpioInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::_;

using uart::UartDriver;

using mocks::MockMotor;
using mocks::MockOsInterface;
using mocks::MockUartInterface;
using mocks::MockGpioInterface;

using dynamixel::Motor;
using dynamixel::DaisyChainParams;
using dynamixel::DaisyChain;
using dynamixel::ResolutionDivider;
using dynamixel::VoltageLimit;
using dynamixel::StatusReturnLevel;
using dynamixel::AlarmType;
using dynamixel::AlarmCondition;




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
class DynamixelTest : public ::testing::Test {
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
// TODO(tyler) refactor these tests to be bounds checks where possible. If we
// can do a bounds check, we can obviously call the function
TEST_F(DynamixelTest, CanBeCreated){
    DaisyChain chain(p);
    MockMotor m1(1, &chain, ResolutionDivider::AX12A);
    MockMotor m2(1, &chain, ResolutionDivider::MX28);
}

TEST_F(DynamixelTest, CanResetMotor){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.reset();
}

TEST_F(DynamixelTest, CanSetId){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setId(2);
}

TEST_F(DynamixelTest, CanSetReturnDelayTime){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setReturnDelayTime(150);
}

TEST_F(DynamixelTest, CanSetCWAngleLimit){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setCWAngleLimit(0);
}

TEST_F(DynamixelTest, CanSetCCWAngleLimit){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setCCWAngleLimit(300);
}

TEST_F(DynamixelTest, CanSetVoltageLimits){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setVoltageLimit(VoltageLimit::HIGHEST, 14.0);
    m.setVoltageLimit(VoltageLimit::LOWEST, 6.0);

    // TODO: move to separate unit test for bounds checks
    ASSERT_FALSE(m.setVoltageLimit(VoltageLimit::HIGHEST, 9000.0));
    ASSERT_FALSE(m.setVoltageLimit(VoltageLimit::LOWEST, 0));
}

TEST_F(DynamixelTest, CanSetMaxTorque){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setMaxTorque(100.0);

    // TODO: move to separate unit test for bounds checks
    ASSERT_FALSE(m.setMaxTorque(-1.0));
    ASSERT_FALSE(m.setMaxTorque(101.0));
}

TEST_F(DynamixelTest, CanSetStatusReturnLevel){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setStatusReturnLevel(StatusReturnLevel::PING_ONLY);
    m.setStatusReturnLevel(StatusReturnLevel::READS_ONLY);
    m.setStatusReturnLevel(StatusReturnLevel::ALL_COMMANDS);

    // Bounds check should cause this to return false
    // TODO: move to separate unit test for bounds checks
    ASSERT_FALSE(m.setStatusReturnLevel(StatusReturnLevel::NUM_LEVELS));
}

TEST_F(DynamixelTest, CanSetAlarm){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setAlarm(AlarmType::LED, AlarmCondition::INPUT_VOLTAGE_ERR);
    m.setAlarm(AlarmType::SHUTDOWN, AlarmCondition::OVERHEATING_ERR);

    // Bounds checks should cause these to return false
    // TODO: move to separate unit test for bounds checks
    ASSERT_FALSE(m.setAlarm(AlarmType::LED, AlarmCondition::NUM_CONDITIONS));
    ASSERT_FALSE(m.setAlarm(AlarmType::NUM_TYPES, AlarmCondition::OVERHEATING_ERR));
}

TEST_F(DynamixelTest, CanEnableTorque){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.enableTorque(true);
    m.enableTorque(false);
}

TEST_F(DynamixelTest, CanEnableLed){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.enableLed(true);
    m.enableLed(false);
}

TEST_F(DynamixelTest, CanSetGoalPosition){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setGoalPosition(150.0);
}

TEST_F(DynamixelTest, CanSetGoalTorque){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setGoalTorque(100.0);
}

TEST_F(DynamixelTest, CanLockEEPROM){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.lockEEPROM();
}

TEST_F(DynamixelTest, CanSetPunch){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.setPunch(10.0);
}

TEST_F(DynamixelTest, CanGetGoalPosition){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    float angle = 0;
    m.getPosition(angle);
}

TEST_F(DynamixelTest, CanGetLoad){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    float load = 0;
    m.getLoad(load);
}

TEST_F(DynamixelTest, CanGetVoltage){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    float voltage = 0;
    m.getVoltage(voltage);
}

TEST_F(DynamixelTest, CanGetTemperature){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    uint8_t temp = 0;
    m.getTemperature(temp);
}

TEST_F(DynamixelTest, CanCheckIfIsJointMode){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    bool isJointMode = false;
    m.isJointMode(isJointMode);
}

TEST_F(DynamixelTest, CanPing){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    uint8_t id;
    m.ping(id);
}

TEST_F(DynamixelTest, CanCheckIfIsMoving){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    bool isMoving = false;
    m.isMoving(isMoving);
}

TEST_F(DynamixelTest, CanEnterWheelMode){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.enterWheelMode();
}

TEST_F(DynamixelTest, CanEnterJointMode){
    DaisyChain chain(p);
    MockMotor m(1, &chain, ResolutionDivider::AX12A);

    m.enterJointMode();
}

} // end anonymous namespace




/**
 * @}
 */
/* end - module name */
