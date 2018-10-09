/**
  *****************************************************************************
  * @file    Dynamixel_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup Dynamixel_test
  * @ingroup  Dynamixel
  * @brief    Dynamixel class unit tests
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "Dynamixel.h"

#include "MockUartInterface.h"
#include "MockOsInterface.h"
#include "MockGpioInterface.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;

using uart::UartDriver;
using mocks::MockOsInterface;
using mocks::MockUartInterface;
using mocks::MockGpioInterface;
using dynamixel::Motor;
using dynamixel::PinConfig;




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
uint16_t dataDirPinNum = 1;
PinConfig pc = {&dataDirPort, dataDirPinNum};




// Functions
// ----------------------------------------------------------------------------
TEST(Dynamixel, CanBeCreated){
    Motor m(1, &UARTxDriver, &gpio, pc);
}

TEST(Dynamixel, CanSetGoalPosition){
    Motor m(1, &UARTxDriver, &gpio, pc);
    m.setGoalPosition(150.0);
}



} // end anonymous namespace




/**
 * @}
 */
/* end - module name */
