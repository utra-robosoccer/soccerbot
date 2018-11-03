/**
  *****************************************************************************
  * @file
  * @author Tyler Gamvrelis
  *
  * @defgroup Peripheral Instances
  * @brief Instances of peripherals the system communicates with. Should be
  *        broken up into the appropriate modules in the near future!
  * @{
  *****************************************************************************
  */




#ifndef PERIPHERAL_INSTANCES_H
#define PERIPHERAL_INSTANCES_H




/********************************* Includes **********************************/
#include <array>
#include "AX12A.h"
#include "MX28.h"
#include "MPU6050.h"

using std::array;
using dynamixel::Motor;
using dynamixel::AX12A;
using dynamixel::MX28;
using imu::MPU6050;




/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------




/************************** insert module name here **************************/
namespace periph{
// Types & enums
// ----------------------------------------------------------------------------
enum motorNames_e : uint8_t {
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
    MOTOR6,
    MOTOR7,
    MOTOR8,
    MOTOR9,
    MOTOR10,
    MOTOR11,
    MOTOR12,
    MOTOR13,
    MOTOR14,
    MOTOR15,
    MOTOR16,
    MOTOR17,
    MOTOR18,
    NUM_MOTORS
};




// Variables
// ----------------------------------------------------------------------------
extern std::array<Motor*, 18> motors;
extern MPU6050 imuData;




// Functions
// ----------------------------------------------------------------------------
/**
 * @brief Configures the IO type used for the motors
 * @param io_type The IO type to be used
 */
void initMotorIOType(IO_Type io_type);

} // end namespace periph




/**
 * @}
 */
/* end - Peripheral Instances */

#endif /* PERIPHERAL_INSTANCES_H */
