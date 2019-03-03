/**
  *****************************************************************************
  * @file    DaisyChain_test.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup DaisyChain_Test
  * @ingroup  DaisyChain
  * @brief    Unit test driver for DaisyChain
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "DaisyChain/DaisyChain.h"

#include "DaisyChain/DaisyChain.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <GpioInterfaceMock.h>
#include <OsInterfaceMock.h>
#include <UartInterfaceMock.h>


using ::testing::DoAll;
using ::testing::Return;
using ::testing::_;

using uart::UartDriver;
using hal::IO_Type;
using cmsis::gmock::OsInterfaceMock;
using hal::gmock::UartInterfaceMock;
using hal::gmock::GpioInterfaceMock;

using dynamixel::DaisyChainParams;
using dynamixel::DaisyChain;




/******************************** File-local *********************************/
namespace{
// Variables
// ----------------------------------------------------------------------------
UART_HandleTypeDef UARTx = {0};
GPIO_TypeDef dataDirPort;




// Classes & structs
// ----------------------------------------------------------------------------
class DaisyChainShould : public ::testing::Test {
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
    UartInterfaceMock uart;
    OsInterfaceMock os;
    GpioInterfaceMock gpio;
    DaisyChainParams p;
};




// Functions
// ----------------------------------------------------------------------------
TEST_F(DaisyChainShould, Construct){
    DaisyChain chain(p);
}

// We cannot really test much more than this for transmit without totally
// whiteboxing the test, which isn't good as we may change the implementation
// soon to buffer the requests then package them as a sync write command
TEST_F(DaisyChainShould, CallSetGpioFunctionUponTransmissionRequest){
    DaisyChain chain(p);

    char msg[] = "hey!";

    EXPECT_CALL(gpio, writePin);
    auto status = chain.requestTransmission((uint8_t*)msg, sizeof(msg));
    EXPECT_TRUE(status);
}

// We cannot really test much more than this for transmit without totally
// whiteboxing the test, which doesn't tell us anything useful in this case
TEST_F(DaisyChainShould, CallSetGpioFunctionUponReceptionRequest){
    DaisyChain chain(p);

    uint8_t buf[10];

    EXPECT_CALL(gpio, writePin);
    auto status = chain.requestReception(buf, 5);
    EXPECT_TRUE(status);
}

TEST_F(DaisyChainShould, SetIOTypeForDriver){
    DaisyChain chain(p);

    IO_Type typeToSet = IO_Type::DMA;
    chain.setIOType(typeToSet);
    IO_Type actualType = UARTxDriver->getIOType();
    EXPECT_EQ(actualType, typeToSet);

    typeToSet = IO_Type::POLL;
    chain.setIOType(typeToSet);
    actualType = UARTxDriver->getIOType();
    EXPECT_EQ(actualType, typeToSet);
}

TEST_F(DaisyChainShould, GetIOTypeForDriver){
    DaisyChain chain(p);

    IO_Type typeToSet = IO_Type::DMA;
    UARTxDriver->setIOType(typeToSet);
    IO_Type typeGot =  chain.getIOType();
    EXPECT_EQ(typeGot, typeToSet);

    typeToSet = IO_Type::POLL;
    UARTxDriver->setIOType(typeToSet);
    typeGot =  chain.getIOType();
    EXPECT_EQ(typeGot, typeToSet);
}

} // end anonymous namespace




/**
 * @}
 */
/* end - DaisyChain_Test */
