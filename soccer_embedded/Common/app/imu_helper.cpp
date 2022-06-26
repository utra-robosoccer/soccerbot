/**
  *****************************************************************************
  * @file
  * @author  Tyler Gamvrelis
  *
  * @addtogroup IMU_Helper IMU Helper
  * @addtogroup Helpers
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "imu_helper.h"
#include "DSP/dsp.h"




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
static dsp::ImuVelocityFilter velocityFilters[
    static_cast<int>(VFilter::NUM_VFILTERS)
];

} // end anonymous namespace




/*********************************** app *************************************/
namespace soccerbot{
namespace app{
// Functions
// ----------------------------------------------------------------------------
void initImuProcessor(){
    for(auto& filter : velocityFilters){
        filter.init();
    }
}

void processImuData(imu::ImuStruct_t& imu){
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

bool readFromSensor(imu::MPU6050& imu_data, uint8_t* num_samples){
    bool success = false;

    imu_data.Read_Accelerometer_IT();

    // Gyroscope data is much more volatile/sensitive to changes than
    // acceleration data. To compensate, we feed in samples to the filter
    // slower. Good DSP practise? Not sure. To compensate for the high
    // delays, we also use a filter with fewer taps than the acceleration
    // filters. Ideally: we would sample faster to reduce aliasing, then
    // use a filter with a smaller cutoff frequency. However, the filter
    // designer we are using does not allow us to generate such filters in
    // the free version, so this is the best we can do unless we use other
    // software.
    ++*num_samples;
    if(*num_samples % 16 == 0){
        imu_data.Read_Gyroscope_IT();
        success = true;
    }

    return success;
}

} // end namespace app
} // end namespace soccerbot

/**
 * @}
 */
/* end - IMU_Helper */
