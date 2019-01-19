/**
  *****************************************************************************
  * @file
  * @author Tyler Gamvrelis
  *
  * @ingroup Peripheral Instances
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "PeripheralInstances.h"

#include "HalUartInterface.h"
#include "GpioInterfaceImpl.h"
#include "OsInterfaceImpl.h"
#include "SystemConf.h"

#include "usart.h"
#include "gpio.h"
#include "i2c.h"

using dynamixel::Motor;
using dynamixel::DaisyChain;
using dynamixel::DaisyChainParams;
using uart::HalUartInterface;
using os::OsInterfaceImpl;
using gpio::GpioInterfaceImpl;




/********************************** periph ***********************************/
namespace periph{
// Variables
// ----------------------------------------------------------------------------
HalUartInterface uartif;
OsInterfaceImpl osif;
GpioInterfaceImpl gpioif;

UartDriver upperLeftLegDriver(&osif, &uartif, UART_HANDLE_UpperLeftLeg);
DaisyChainParams upperLeftLegParams = {
    &upperLeftLegDriver,
    &gpioif,
    GPIOA,
    GPIO_PIN_8
};
DaisyChain upperLeftLegDaisyChain(upperLeftLegParams);

UartDriver lowerRightLegDriver(&osif, &uartif, UART_HANDLE_LowerRightLeg);
DaisyChainParams lowerRightLegParams = {
    &lowerRightLegDriver,
    &gpioif,
    GPIOA,
    GPIO_PIN_4
};
DaisyChain lowerRightLegDaisyChain(lowerRightLegParams);

UartDriver headAndArmsDriver(&osif, &uartif, UART_HANDLE_HeadAndArms);
DaisyChainParams headAndArmsParams = {
    &headAndArmsDriver,
    &gpioif,
    GPIOB,
    GPIO_PIN_2
};
DaisyChain headAndArmsDaisyChain(headAndArmsParams);

UartDriver upperRightLegDriver(&osif, &uartif, UART_HANDLE_UpperRightLeg);
DaisyChainParams upperRightLegParams = {
    &upperRightLegDriver,
    &gpioif,
    GPIOC,
    GPIO_PIN_3
};
DaisyChain upperRightLegDaisyChain(upperRightLegParams);

UartDriver lowerLeftLegDriver(&osif, &uartif, UART_HANDLE_LowerLeftLeg);
DaisyChainParams lowerLeftLegParams = {
    &lowerLeftLegDriver,
    &gpioif,
    GPIOC,
    GPIO_PIN_8
};
DaisyChain lowerLeftLegDaisyChain(lowerLeftLegParams);

MX28 motor1(1, &lowerRightLegDaisyChain);
MX28 motor2(2, &lowerRightLegDaisyChain);
MX28 motor3(3, &lowerRightLegDaisyChain);
MX28 motor4(4, &upperRightLegDaisyChain);
MX28 motor5(5, &upperRightLegDaisyChain);
MX28 motor6(6, &upperRightLegDaisyChain);
MX28 motor7(7, &upperLeftLegDaisyChain);
MX28 motor8(8, &upperLeftLegDaisyChain);
MX28 motor9(9, &upperLeftLegDaisyChain);
MX28 motor10(10, &lowerLeftLegDaisyChain);
MX28 motor11(11, &lowerLeftLegDaisyChain);
MX28 motor12(12, &lowerLeftLegDaisyChain);
AX12A motor13(13, &headAndArmsDaisyChain);
AX12A motor14(14, &headAndArmsDaisyChain);
AX12A motor15(15, &headAndArmsDaisyChain);
AX12A motor16(16, &headAndArmsDaisyChain);
AX12A motor17(17, &headAndArmsDaisyChain);
AX12A motor18(18, &headAndArmsDaisyChain);

std::array<Motor*, 18> motors = {
    &motor1,
    &motor2,
    &motor3,
    &motor4,
    &motor5,
    &motor6,
    &motor7,
    &motor8,
    &motor9,
    &motor10,
    &motor11,
    &motor12,
    &motor13,
    &motor14,
    &motor15,
    &motor16,
    &motor17,
    &motor18
};

MPU6050 imuData(&hi2c1);




// Functions
// ----------------------------------------------------------------------------
void initMotorIOType(IO_Type io_type){
    constexpr TickType_t MOTOR_MAX_BLOCK_TIME = pdMS_TO_TICKS(2);

    upperLeftLegDriver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    lowerRightLegDriver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    headAndArmsDriver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    upperRightLegDriver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    lowerLeftLegDriver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);

    upperLeftLegDaisyChain.setIOType(io_type);
    lowerRightLegDaisyChain.setIOType(io_type);
    headAndArmsDaisyChain.setIOType(io_type);
    upperRightLegDaisyChain.setIOType(io_type);
    lowerLeftLegDaisyChain.setIOType(io_type);
}

} // end namespace periph




/**
 * @}
 */
/* end - Peripheral Instances */
