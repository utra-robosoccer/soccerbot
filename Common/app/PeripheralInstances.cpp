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

UartDriver uart1Driver(&osif, &uartif, &huart1);
DaisyChainParams uart1Params = {
    &uart1Driver,
    &gpioif,
    GPIOA,
    GPIO_PIN_8
};
DaisyChain uart1DaisyChain(uart1Params);

UartDriver uart2Driver(&osif, &uartif, &huart2);
DaisyChainParams uart2Params = {
    &uart2Driver,
    &gpioif,
    GPIOA,
    GPIO_PIN_4
};
DaisyChain uart2DaisyChain(uart2Params);

UartDriver uart3Driver(&osif, &uartif, &huart3);
DaisyChainParams uart3Params = {
    &uart3Driver,
    &gpioif,
    GPIOB,
    GPIO_PIN_2
};
DaisyChain uart3DaisyChain(uart3Params);

UartDriver uart4Driver(&osif, &uartif, &huart4);
DaisyChainParams uart4Params = {
    &uart4Driver,
    &gpioif,
    GPIOC,
    GPIO_PIN_3
};
DaisyChain uart4DaisyChain(uart4Params);

UartDriver uart6Driver(&osif, &uartif, &huart6);
DaisyChainParams uart6Params = {
    &uart6Driver,
    &gpioif,
    GPIOC,
    GPIO_PIN_8
};
DaisyChain uart6DaisyChain(uart6Params);

MX28 motor1(1, &uart2DaisyChain);
MX28 motor2(2, &uart2DaisyChain);
MX28 motor3(3, &uart2DaisyChain);
MX28 motor4(4, &uart4DaisyChain);
MX28 motor5(5, &uart4DaisyChain);
MX28 motor6(6, &uart4DaisyChain);
MX28 motor7(7, &uart1DaisyChain);
MX28 motor8(8, &uart1DaisyChain);
MX28 motor9(9, &uart1DaisyChain);
MX28 motor10(10, &uart6DaisyChain);
MX28 motor11(11, &uart6DaisyChain);
MX28 motor12(12, &uart6DaisyChain);
AX12A motor13(13, &uart3DaisyChain);
AX12A motor14(14, &uart3DaisyChain);
AX12A motor15(15, &uart3DaisyChain);
AX12A motor16(16, &uart3DaisyChain);
AX12A motor17(17, &uart3DaisyChain);
AX12A motor18(18, &uart3DaisyChain);

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

    uart1Driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    uart2Driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    uart3Driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    uart4Driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    uart6Driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);

    uart1DaisyChain.setIOType(io_type);
    uart2DaisyChain.setIOType(io_type);
    uart3DaisyChain.setIOType(io_type);
    uart4DaisyChain.setIOType(io_type);
    uart6DaisyChain.setIOType(io_type);
}

} // end namespace periph




/**
 * @}
 */
/* end - Peripheral Instances */
