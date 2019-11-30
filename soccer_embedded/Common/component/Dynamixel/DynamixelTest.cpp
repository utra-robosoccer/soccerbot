/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  *
  * @defgroup Motor_Test
  * @ingroup  Dynamixel
  * @brief    Motor test driver
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "GpioInterfaceMock.h"
#include "MotorMock.h"
#include "OsInterfaceMock.h"
#include "UartInterfaceMock.h"

using ::testing::_;
using ::testing::Args;
using ::testing::ElementsAreArray;
using ::testing::SetArrayArgument;
using ::testing::Return;
using ::testing::NiceMock;

using uart::UartDriver;

using dynamixel::gmock::MotorMock;
using cmsis::gmock::OsInterfaceMock;
using hal::gmock::UartInterfaceMock;
using hal::gmock::GpioInterfaceMock;

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
UART_HandleTypeDef UARTx = {0};
GPIO_TypeDef dataDirPort;




// Classes & structs
// ----------------------------------------------------------------------------
class MotorTest : public ::testing::Test {
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
TEST_F(MotorTest, IdReturnsIdSetAtInitialization){
    DaisyChain chain(p);

    NiceMock<MotorMock> m0(1, &chain, ResolutionDivider::AX12A);
    ASSERT_EQ(m0.id(), 1);

    NiceMock<MotorMock> m1(2, &chain, ResolutionDivider::AX12A);
    ASSERT_EQ(m1.id(), 2);

    NiceMock<MotorMock> m2(18, &chain, ResolutionDivider::AX12A);
    ASSERT_EQ(m2.id(), 18);

    NiceMock<MotorMock> m3(42, &chain, ResolutionDivider::AX12A);
    ASSERT_NE(m3.id(), 0);
}

TEST_F(MotorTest, setIdBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setId(10));

    ASSERT_FALSE(m.setId(0xFD));
    ASSERT_FALSE(m.setId(0xFF));
}

TEST_F(MotorTest, setReturnDelayTimeBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setReturnDelayTime(150));

    ASSERT_FALSE(m.setReturnDelayTime(0));
    ASSERT_FALSE(m.setReturnDelayTime(510));
}

TEST_F(MotorTest, setCWAngleLimitBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setCwAngleLimit(dynamixel::MIN_ANGLE));

    ASSERT_FALSE(m.setCwAngleLimit(dynamixel::MIN_ANGLE - 1));
    ASSERT_FALSE(m.setCwAngleLimit(dynamixel::MAX_ANGLE + 1));
}

TEST_F(MotorTest, setCCWAngleLimitBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setCcwAngleLimit(dynamixel::MAX_ANGLE));

    ASSERT_FALSE(m.setCcwAngleLimit(dynamixel::MIN_ANGLE - 1));
    ASSERT_FALSE(m.setCcwAngleLimit(dynamixel::MAX_ANGLE + 1));
}

TEST_F(MotorTest, setVoltageLimitsBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setVoltageLimit(VoltageLimit::HIGHEST, dynamixel::MAX_VOLTAGE));
    ASSERT_TRUE(m.setVoltageLimit(VoltageLimit::LOWEST, dynamixel::MIN_VOLTAGE));

    ASSERT_FALSE(
        m.setVoltageLimit(VoltageLimit::HIGHEST, dynamixel::MAX_VOLTAGE + 1)
    );

    ASSERT_FALSE(
        m.setVoltageLimit(VoltageLimit::LOWEST, dynamixel::MIN_VOLTAGE - 1)
    );
}

TEST_F(MotorTest, setMaxTorqueBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setMaxTorque(100.0));

    ASSERT_FALSE(m.setMaxTorque(-1.0));
    ASSERT_FALSE(m.setMaxTorque(101.0));
}

TEST_F(MotorTest, setStatusReturnLevelBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_FALSE(m.setStatusReturnLevel(StatusReturnLevel::NUM_LEVELS));
}

TEST_F(MotorTest, setAlarmBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setAlarm(AlarmType::LED, AlarmCondition::INPUT_VOLTAGE_ERR));
    ASSERT_TRUE(m.setAlarm(AlarmType::SHUTDOWN, AlarmCondition::OVERHEATING_ERR));

    ASSERT_FALSE(m.setAlarm(AlarmType::LED, AlarmCondition::NUM_CONDITIONS));

    ASSERT_FALSE(
        m.setAlarm(AlarmType::NUM_TYPES, AlarmCondition::OVERHEATING_ERR)
    );
}

TEST_F(MotorTest, setGoalPositionBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setGoalPosition(150.0));

    ASSERT_FALSE(m.setGoalPosition(dynamixel::MIN_ANGLE - 1));
    ASSERT_FALSE(m.setGoalPosition(dynamixel::MAX_ANGLE + 1));
}

TEST_F(MotorTest, setGoalTorqueBoundsCheckPasses){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setGoalTorque(100.0));

    ASSERT_FALSE(m.setGoalTorque(dynamixel::MIN_TORQUE - 1));
    ASSERT_FALSE(m.setGoalTorque(dynamixel::MAX_TORQUE + 1));
}

TEST_F(MotorTest, CanSetPunch){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    ASSERT_TRUE(m.setPunch(10.0));

    ASSERT_FALSE(m.setPunch(dynamixel::MIN_PUNCH - 1));
    ASSERT_FALSE(m.setPunch(dynamixel::MAX_PUNCH + 1));
}

TEST_F(MotorTest, ParsesReadDataExampleProperly){
    DaisyChain chain(p);
    NiceMock<MotorMock> m(1, &chain, ResolutionDivider::AX12A);

    // AX12A datasheet page 20, section 4-2
    uint8_t expectedTxArray[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2B, 0x01, 0xCC};
    uint8_t mockedRxArray[] = {0xFF, 0xFF, 0x01, 0x03, 0x00, 0x20, 0xDB};

    EXPECT_CALL(uart, transmitPoll(_, _, _, _))
        .With(Args<1, 2>(ElementsAreArray(expectedTxArray)))
        .WillOnce(Return(HAL_OK));

    EXPECT_CALL(uart, receivePoll(_, _, _, _))
        .WillOnce(DoAll(
                SetArrayArgument<1>(mockedRxArray, mockedRxArray + sizeof(mockedRxArray)),
                Return(HAL_OK)
            )
        );

    uint8_t temp = 0;
    ASSERT_TRUE(m.getTemperature(temp));
    ASSERT_TRUE(temp == 32);
}

} // end anonymous namespace



/**
 * @}
 */
/* end - Motor_Test */
