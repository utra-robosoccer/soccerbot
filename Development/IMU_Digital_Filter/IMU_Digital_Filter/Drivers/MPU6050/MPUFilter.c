/**
  *****************************************************************************
  * @file    MPUFilter.c
  * @author  Tyler
  * @brief   Digital filtering data structures and functions, for IMU data.
  *
  * @defgroup IMU_Filter IMU Filter
  * @brief    Digital filtering data structures and functions, for IMU data.
  * @{
  *****************************************************************************
  */

/******************************* SOURCE LICENSE *********************************
Copyright (c) 2018 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/
// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp




/********************************* Includes **********************************/
#include "MPUFilter.h"
#include <stdlib.h> // For malloc/free
#include <string.h> // For memset




/********************************* Constants *********************************/
/**
 * Number of filter taps (equal to number of coefficients here) for
 * acceleration filter
 */
static const uint16_t MPUFilter_numTapsAccel = 21;
static const uint16_t MPUFilter_blockSizeAccel = 1;

/**
 * FIR filter coefficients, which correspond to the filter's impulse response
 * in the discrete time domain. Since there are 21 coefficients, input
 * signals are delayed by (21 - 1)/2 = 10 samples. This filter is used for
 * acceleration data.
 */
static float32_t MPUFilter_coefficientsAccel[21] =
{
    -0.018872079, -0.0011102221, 0.0030367336, 0.014906744, 0.030477359, 0.049086205,
    0.070363952, 0.089952103, 0.10482875, 0.11485946, 0.11869398, 0.11485946,
    0.10482875, 0.089952103, 0.070363952, 0.049086205, 0.030477359, 0.014906744,
    0.0030367336, -0.0011102221, -0.018872079
};

/**
 * Number of filter taps (equal to number of coefficients here) for
 * angular velocity filter
 */
static const uint16_t MPUFilter_numTapsVel = 11;
static const uint16_t MPUFilter_blockSizeVel = 1;

/**
 * Filter coefficients for angular velocity filter. 5 sample delay
 */
static float MPUFilter_coefficientsVel[11] =
{
    0.030738841, 0.048424201, 0.083829062, 0.11125669, 0.13424691, 0.14013315,
    0.13424691, 0.11125669, 0.083829062, 0.048424201, 0.030738841
};




/********************************** Types ************************************/
typedef struct
{
    arm_fir_instance_f32 instance;
    float32_t state[22];
    float32_t output;
} MPUFilterAccelType;

typedef struct
{
    arm_fir_instance_f32 instance;
    float32_t state[12];
    float32_t output;
} MPUFilterVelType;




/***************************** Private Variables *****************************/
/** Filters for acceleration along each axis */
static MPUFilterAccelType axFilter, ayFilter, azFilter;

/** Filters for angular velocity along each axis */
static MPUFilterVelType vxFilter, vyFilter, vzFilter;




/************************ Private Function Prototypes ************************/
static void MPUFilter_initAccel( MPUFilterAccelType * pThis );
static void MPUFilter_initVel( MPUFilterVelType * pThis );
static void MPUFilter_resetAccel( MPUFilterAccelType * pThis );
static void MPUFilter_resetVel( MPUFilterVelType * pThis );




/******************************** Functions **********************************/
// Acceleration
static inline void MPUFilter_writeInputAccel(MPUFilterAccelType * pThis, float input){
    arm_fir_f32( &pThis->instance, &input, &pThis->output, 1 );
};
static inline float MPUFilter_readOutputAccel( MPUFilterAccelType * pThis ){
    return pThis->output;
}

static void MPUFilter_initAccel( MPUFilterAccelType * pThis )
{
	arm_fir_init_f32(&pThis->instance, MPUFilter_numTapsAccel,
	        MPUFilter_coefficientsAccel, pThis->state,
	        MPUFilter_blockSizeAccel
	);
	MPUFilter_resetAccel( pThis );
}

static void MPUFilter_resetAccel( MPUFilterAccelType * pThis )
{
    memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
    pThis->output = 0;                                  // Reset output
}


// Velocity
static inline void MPUFilter_writeInputVel(MPUFilterVelType * pThis, float input){
    arm_fir_f32( &pThis->instance, &input, &pThis->output, 1 );
};
static inline float MPUFilter_readOutputVel( MPUFilterVelType * pThis ){
    return pThis->output;
}

static void MPUFilter_initVel( MPUFilterVelType * pThis )
{
    arm_fir_init_f32(&pThis->instance, MPUFilter_numTapsVel,
            MPUFilter_coefficientsVel, pThis->state,
            MPUFilter_blockSizeVel
    );
    MPUFilter_resetVel( pThis );
}

static void MPUFilter_resetVel( MPUFilterVelType * pThis )
{
    memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
    pThis->output = 0;                                  // Reset output
}

/**
 * @brief  Initialize acceleration and angular velocity FIR filters for IMU
 *         data.
 * @return None
 */
void MPUFilter_InitAllFilters(){
    MPUFilter_initAccel(&axFilter);
    MPUFilter_initAccel(&ayFilter);
    MPUFilter_initAccel(&azFilter);
    MPUFilter_initVel(&vxFilter);
    MPUFilter_initVel(&vyFilter);
    MPUFilter_initVel(&vzFilter);
}

/**
 * @brief  Reads Az, Ay, and Ax from the IMU handle and writes them into their
 *         corresponding FIR filters. The output of each FIR filter is then
 *         read and stored in the IMU handle where it was previously read.
 * @param  IMUdata Pointer to a struct of type MPU6050_HandleTypeDef
 * @return None
 */
void MPUFilter_FilterAcceleration(MPU6050_HandleTypeDef* IMUdata){
    MPUFilter_writeInputAccel(&azFilter, IMUdata->_Z_ACCEL);
    IMUdata->_Z_ACCEL = MPUFilter_readOutputAccel(&azFilter);

    MPUFilter_writeInputAccel(&ayFilter, IMUdata->_Y_ACCEL);
    IMUdata->_Y_ACCEL = MPUFilter_readOutputAccel(&ayFilter);

    MPUFilter_writeInputAccel(&axFilter, IMUdata->_X_ACCEL);
    IMUdata->_X_ACCEL = MPUFilter_readOutputAccel(&axFilter);
}

/**
 * @brief  Reads Vz, Vy, and Vx from the IMU handle and writes them into their
 *         corresponding FIR filters. The output of each FIR filter is then
 *         read and stored in the IMU handle where it was previously read.
 * @param  IMUdata Pointer to a struct of type MPU6050_HandleTypeDef
 * @return None
 */
void MPUFilter_FilterAngularVelocity(MPU6050_HandleTypeDef* IMUdata){
    MPUFilter_writeInputVel(&vzFilter, IMUdata->_Z_GYRO);
    IMUdata->_Z_GYRO = MPUFilter_readOutputVel(&vzFilter);

    MPUFilter_writeInputVel(&vyFilter, IMUdata->_Y_GYRO);
    IMUdata->_Y_GYRO = MPUFilter_readOutputVel(&vyFilter);

    MPUFilter_writeInputVel(&vxFilter, IMUdata->_X_GYRO);
    IMUdata->_X_GYRO = MPUFilter_readOutputVel(&vxFilter);
}

/**
 * @}
 */
/* end IMU_Filter */
