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
    AX12A m1(1, &chain, ResolutionDivider::AX12A);
}

} // end anonymous namespace




/**
 * @}
 */
/* end - AX12A_Test */
