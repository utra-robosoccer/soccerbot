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
// Constants
// ----------------------------------------------------------------------------
/**
 * @brief Filter coefficients for angular velocity filter. (11 - 1) / 2 = 5
 *        sample delay
 */
static const float velocityCoefficients[11] =
{
    0.030738841, 0.048424201, 0.083829062, 0.11125669, 0.13424691, 0.14013315,
    0.13424691, 0.11125669, 0.083829062, 0.048424201, 0.030738841
};

/**
 * @brief Number of taps in the velocity filter (equal to number of
 *        coefficients)
 */
static constexpr uint16_t velNumTaps = 11;

/**
 * @brief Number of input samples to be buffered before updating filter
 *        output
 */
static constexpr uint32_t velBlockSize = 1;




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
static dsp::fir_f32<velNumTaps, velBlockSize> velocityFilters[
    static_cast<int>(VFilter::NUM_VFILTERS)
];

} // end anonymous namespace




/********************************* Helpers ***********************************/
namespace app{
// Functions
// ----------------------------------------------------------------------------
void initImuProcessor(){
    for(auto filter : velocityFilters){
        filter.init(velNumTaps, (float*)velocityCoefficients, velBlockSize);
    }
}

void processImuData(IMUStruct& imu){
    velocityFilters[static_cast<int>(VFilter::VX)].update(
        &imu._x_Gyro,
        &imu._x_Gyro,
        1
    );
    velocityFilters[static_cast<int>(VFilter::VY)].update(
        &imu._y_Gyro,
        &imu._y_Gyro,
        1
    );
    velocityFilters[static_cast<int>(VFilter::VZ)].update(
        &imu._z_Gyro,
        &imu._z_Gyro,
        1
    );
}

bool readFromSensor(imu::MPU6050& IMUdata, uint8_t* numSamples){
    bool retval = false;

    IMUdata.Read_Accelerometer_Withoffset_IT();

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
        IMUdata.Read_Gyroscope_Withoffset_IT();
        retval = true;
    }

    return retval;
}

} // end namespace Helpers

/**
 * @}
 */
/* end - IMU_Helper */
