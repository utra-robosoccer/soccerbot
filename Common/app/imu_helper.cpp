/**
  *****************************************************************************
  * @file    imu_helper.cpp
  * @author  Tyler Gamvrelis
  *
  * @addtogroup IMU_Helper IMU Helper
  * @addtogroup Helpers
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "imu_helper.h"
#include "dsp.h"




/******************************** File-local *********************************/
namespace{
// Classes and structs
// ----------------------------------------------------------------------------
/** @brief Indexes into the velocityFilters array */
enum class VFilter : uint8_t{
    VX = 0,
    VY,
    VZ,
    NUM_VFILTERS
};




// Variables
// ----------------------------------------------------------------------------
/** @brief Angular velocity filters along x-, y-, and z-axes */
static dsp::imuVelocityFilter velocityFilters[
    static_cast<int>(VFilter::NUM_VFILTERS)
];

} // end anonymous namespace




/********************************* Helpers ***********************************/
namespace app{
// Functions
// ----------------------------------------------------------------------------
void initImuProcessor(){
    for(auto& filter : velocityFilters){
        filter.init();
    }
}

void processImuData(imu::IMUStruct_t& imu){
    velocityFilters[static_cast<int>(VFilter::VX)].update(
        &imu.x_Gyro,
        &imu.x_Gyro,
        1
    );
    velocityFilters[static_cast<int>(VFilter::VY)].update(
        &imu.y_Gyro,
        &imu.y_Gyro,
        1
    );
    velocityFilters[static_cast<int>(VFilter::VZ)].update(
        &imu.z_Gyro,
        &imu.z_Gyro,
        1
    );
}

bool readFromSensor(imu::MPU6050& IMUdata, uint8_t* numSamples){
    bool retval = false;

    IMUdata.Read_Accelerometer_IT();

    // Gyroscope data is much more volatile/sensitive to changes than
    // acceleration data. To compensate, we feed in samples to the filter
    // slower. Good DSP practise? Not sure. To compensate for the high
    // delays, we also use a filter with fewer taps than the acceleration
    // filters. Ideally: we would sample faster to reduce aliasing, then
    // use a filter with a smaller cutoff frequency. However, the filter
    // designer we are using does not allow us to generate such filters in
    // the free version, so this is the best we can do unless we use other
    // software.
    ++*numSamples;
    if(*numSamples % 16 == 0){
        IMUdata.Read_Gyroscope_IT();
        retval = true;
    }

    return retval;
}

} // end namespace Helpers

/**
 * @}
 */
/* end - IMU_Helper */
