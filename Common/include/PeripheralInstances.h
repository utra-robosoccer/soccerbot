/**
  *****************************************************************************
  * @file    MotorInstances.h
  * @author  Tyler Gamvrelis
  *
  * @defgroup
  * @{
  *****************************************************************************
  */




#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H




/********************************* Includes **********************************/
#include "AX12A.h"
#include "MX28.h"
#include "MPU6050.h"

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
enum motorNames : uint8_t {
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
extern MX28 motor1;
extern MX28 motor2;
extern MX28 motor3;
extern MX28 motor4;
extern MX28 motor5;
extern MX28 motor6;
extern MX28 motor7;
extern MX28 motor8;
extern MX28 motor9;
extern MX28 motor10;
extern MX28 motor11;
extern MX28 motor12;
extern AX12A motor13;
extern AX12A motor14;
extern AX12A motor15;
extern AX12A motor16;
extern AX12A motor17;
extern AX12A motor18;

extern Motor* motors[18];

extern MPU6050 imuData;

} // end namespace periph




/***************************** Inline functions ******************************/




/**
 * @}
 */
/* end - Header */

#endif /* MOTOR_INSTANCES_H */
