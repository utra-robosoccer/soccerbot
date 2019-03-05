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

#include "GpioInterfaceImpl.h"
#include "OsInterfaceImpl.h"
#include "SystemConf.h"
#include "UartInterfaceImpl.h"

#include "usart.h"
#include "gpio.h"
#include "i2c.h"

using dynamixel::Motor;
using dynamixel::DaisyChain;
using dynamixel::DaisyChainParams;
using hal::UartInterfaceImpl;
using cmsis::OsInterfaceImpl;
using hal::GpioInterfaceImpl;




/********************************** periph ***********************************/
namespace periph{
// Variables
// ----------------------------------------------------------------------------
UartInterfaceImpl uart_if;
OsInterfaceImpl os_if;
GpioInterfaceImpl gpio_if;

UartDriver upper_left_leg_driver(&os_if, &uart_if, UART_HANDLE_UpperLeftLeg);
DaisyChainParams upper_left_leg_params = {
    &upper_left_leg_driver,
    &gpio_if,
    GPIOA,
    GPIO_PIN_8
};
DaisyChain upper_left_leg_daisy_chain(upper_left_leg_params);

UartDriver lower_right_leg_driver(&os_if, &uart_if, UART_HANDLE_LowerRightLeg);
DaisyChainParams lower_right_leg_params = {
    &lower_right_leg_driver,
    &gpio_if,
    GPIOA,
    GPIO_PIN_4
};
DaisyChain lower_right_leg_daisy_chain(lower_right_leg_params);

UartDriver head_and_arms_driver(&os_if, &uart_if, UART_HANDLE_HeadAndArms);
DaisyChainParams head_and_arms_params = {
    &head_and_arms_driver,
    &gpio_if,
    GPIOB,
    GPIO_PIN_2
};
DaisyChain head_and_arms_daisy_chain(head_and_arms_params);

UartDriver upper_right_leg_driver(&os_if, &uart_if, UART_HANDLE_UpperRightLeg);
DaisyChainParams upper_right_leg_params = {
    &upper_right_leg_driver,
    &gpio_if,
    GPIOC,
    GPIO_PIN_3
};
DaisyChain upper_right_leg_daisy_chain(upper_right_leg_params);

UartDriver lower_left_leg_driver(&os_if, &uart_if, UART_HANDLE_LowerLeftLeg);
DaisyChainParams lower_left_leg_params = {
    &lower_left_leg_driver,
    &gpio_if,
    GPIOC,
    GPIO_PIN_8
};
DaisyChain lower_left_leg_daisy_chain(lower_left_leg_params);

MX28 motor1(1, &lower_right_leg_daisy_chain);
MX28 motor2(2, &lower_right_leg_daisy_chain);
MX28 motor3(3, &lower_right_leg_daisy_chain);
MX28 motor4(4, &upper_right_leg_daisy_chain);
MX28 motor5(5, &upper_right_leg_daisy_chain);
MX28 motor6(6, &upper_right_leg_daisy_chain);
MX28 motor7(7, &upper_left_leg_daisy_chain);
MX28 motor8(8, &upper_left_leg_daisy_chain);
MX28 motor9(9, &upper_left_leg_daisy_chain);
MX28 motor10(10, &lower_left_leg_daisy_chain);
MX28 motor11(11, &lower_left_leg_daisy_chain);
MX28 motor12(12, &lower_left_leg_daisy_chain);
AX12A motor13(13, &head_and_arms_daisy_chain);
AX12A motor14(14, &head_and_arms_daisy_chain);
AX12A motor15(15, &head_and_arms_daisy_chain);
AX12A motor16(16, &head_and_arms_daisy_chain);
AX12A motor17(17, &head_and_arms_daisy_chain);
AX12A motor18(18, &head_and_arms_daisy_chain);

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

MPU6050 imu_data(&hi2c1);




// Functions
// ----------------------------------------------------------------------------
void initMotorIOType(IO_Type io_type){
    constexpr TickType_t MOTOR_MAX_BLOCK_TIME = pdMS_TO_TICKS(2);

    upper_left_leg_driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    lower_right_leg_driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    head_and_arms_driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    upper_right_leg_driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);
    lower_left_leg_driver.setMaxBlockTime(MOTOR_MAX_BLOCK_TIME);

    upper_left_leg_daisy_chain.setIOType(io_type);
    lower_right_leg_daisy_chain.setIOType(io_type);
    head_and_arms_daisy_chain.setIOType(io_type);
    upper_right_leg_daisy_chain.setIOType(io_type);
    lower_left_leg_daisy_chain.setIOType(io_type);
}

} // end namespace periph




/**
 * @}
 */
/* end - Peripheral Instances */
