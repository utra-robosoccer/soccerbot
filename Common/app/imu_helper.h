/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  * @brief   Helper file for the IMU thread in the main program flow
  *
  * @defgroup Header
  * @addtogroup Helpers
  * @{
  *****************************************************************************
  */




#ifndef IMU_HELPER_H
#define IMU_HELPER_H




/********************************* Includes **********************************/
#include "MPU6050/MPU6050.h"




/*********************************** app *************************************/
namespace soccerbot{
namespace app{
// Functions
// ----------------------------------------------------------------------------
/**
 * @brief Initialize the data processor. Right now, this just initializes the
 *        angular velocity FIR filters
 */
void initImuProcessor();

/**
 * @brief Generic processor for IMU data. Right now, it writes Vx, Vy, Vz into
 *        their corresponding FIR filters and reads the output into the same
 *        location these were read from
 * @param[in, out] IMUStruct Reference to IMU data container
 */
void processImuData(imu::ImuStruct_t& imu);

/**
 * @brief   Reads Ax, Ay, Az, Vx, Vy, Vz from IMU sensor (not all of these are
 *          read every cycle; angular velocity is read less frequently as it is
 *          quite noisy)
 * @param   imu_data Reference to the MPU6050 object, which manages interactions
 *          with that sensor
 * @param   num_samples The number of samples that have been acquired so far, up
 *          to some multiple of 16
 * @return  true if the data processor should be run, otherwise false
 */
bool readFromSensor(imu::MPU6050& imu_data, uint8_t* num_samples);

} // end namespace app
} // end namespace soccerbot




/**
 * @}
 */
/* end - Header */

#endif /* IMU_HELPER_H */
