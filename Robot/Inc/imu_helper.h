/**
  *****************************************************************************
  * @file    imu_helper.h
  * @author  Tyler Gamvrelis
  * @brief   Helper file for the IMU thread in the main program flow
  *
  * @defgroup Header
  * @{
  *****************************************************************************
  */




#ifndef IMU_HELPER_H
#define IMU_HELPER_H




/********************************* Includes **********************************/
//#include "MPU6050.h"
#include "../Drivers/MPU6050/MPU6050.h"

using namespace IMUnamespace;




/********************************* Helpers ***********************************/
namespace Helpers{
// Functions
// ----------------------------------------------------------------------------
/**
 * @brief Initialize angular velocity FIR filters for IMU data
 */
void initAngularVelocityFilters();

/**
 * @brief Reads Vz, Vy, and Vx from the MPU6050 object and writes them into
 *        their corresponding FIR filters. The output of each FIR filter is
 *        read and stored in the MPU6050 object where it was previously read
 * @param IMUdata Reference to MPU6050 object
 */
void filterAngularVelocity(IMUStruct& imu);

} // end namespace Helpers




/**
 * @}
 */
/* end - Header */

#endif /* IMU_HELPER_H */
