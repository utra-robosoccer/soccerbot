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

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::DoAll;
using ::testing::SetArgPointee;
using ::testing::Return;
using ::testing::_;

using uart::UartDriver;
using mocks::MockOsInterface;
using mocks::MockUartInterface;
using dynamixel::Motor;
using dynamixel::PinConfig;




/******************************** File-local *********************************/
namespace{
// Variables
// ----------------------------------------------------------------------------
MockUartInterface uart;
MockOsInterface os;
UART_HandleTypeDef UARTx = {0};
UartDriver UARTxDriver(&os, &uart, &UARTx);

GPIO_TypeDef dataDirPort;
uint16_t dataDirPinNum = 1;
PinConfig pc = {&dataDirPort, dataDirPinNum};




// Functions
// ----------------------------------------------------------------------------
TEST(Dynamixel, ShouldConstruct){
    Motor m(1, &UARTxDriver, pc);
}



} // end anonymous namespace




/**
 * @}
 */
/* end - module name */
