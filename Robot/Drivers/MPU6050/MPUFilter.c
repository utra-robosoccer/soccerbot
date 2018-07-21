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
static const uint16_t MPUFilter_numTaps = 21;
static const uint16_t MPUFilter_blockSize = 16;

static float32_t MPUFilter_coefficients[21] =
{
    -0.018872079, -0.0011102221, 0.0030367336, 0.014906744, 0.030477359, 0.049086205,
    0.070363952, 0.089952103, 0.10482875, 0.11485946, 0.11869398, 0.11485946,
    0.10482875, 0.089952103, 0.070363952, 0.049086205, 0.030477359, 0.014906744,
    0.0030367336, -0.0011102221, -0.018872079
};




/********************************** Types ************************************/
typedef struct
{
    arm_fir_instance_f32 instance;
    float32_t state[37];
    float32_t output;
} MPUFilterType;




/***************************** Private Variables *****************************/
static MPUFilterType axFilter, ayFilter, azFilter, vxFilter, vyFilter, vzFilter;




/************************ Private Function Prototypes ************************/
static void MPUFilter_init( MPUFilterType * pThis );
static void MPUFilter_reset( MPUFilterType * pThis );




/******************************** Functions **********************************/
static inline void MPUFilter_writeInput(MPUFilterType * pThis, float input){
    arm_fir_f32( &pThis->instance, &input, &pThis->output, 1 );
};
static inline float MPUFilter_readOutput( MPUFilterType * pThis ){
    return pThis->output;
}

static void MPUFilter_init( MPUFilterType * pThis )
{
	arm_fir_init_f32( &pThis->instance, MPUFilter_numTaps, MPUFilter_coefficients, pThis->state, MPUFilter_blockSize );
	MPUFilter_reset( pThis );

}

static void MPUFilter_reset( MPUFilterType * pThis )
{
	memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
	pThis->output = 0;									// Reset output

}

/**
 * @brief  Initialize acceleration and angular velocity FIR filters for IMU
 *         data.
 * @return None
 */
void MPUFilter_InitAllFilters(){
    MPUFilter_init(&axFilter);
    MPUFilter_init(&ayFilter);
    MPUFilter_init(&azFilter);
    MPUFilter_init(&vxFilter);
    MPUFilter_init(&vyFilter);
    MPUFilter_init(&vzFilter);
}

/**
 * @brief  Reads Az, Ay, and Ax from the IMU handle and writes them into their
 *         corresponding FIR filters. The output of each FIR filter is then
 *         read and stored in the IMU handle where it was previously read.
 * @param  IMUdata Pointer to a struct of type MPU6050_HandleTypeDef
 * @return None
 */
void MPUFilter_FilterAcceleration(MPU6050_HandleTypeDef* IMUdata){
    MPUFilter_writeInput(&azFilter, IMUdata->_Z_ACCEL);
    IMUdata->_Z_ACCEL = MPUFilter_readOutput(&azFilter);

    MPUFilter_writeInput(&ayFilter, IMUdata->_Y_ACCEL);
    IMUdata->_Y_ACCEL = MPUFilter_readOutput(&ayFilter);

    MPUFilter_writeInput(&axFilter, IMUdata->_X_ACCEL);
    IMUdata->_X_ACCEL = MPUFilter_readOutput(&axFilter);
}

/**
 * @brief  Reads Vz, Vy, and Vx from the IMU handle and writes them into their
 *         corresponding FIR filters. The output of each FIR filter is then
 *         read and stored in the IMU handle where it was previously read.
 * @param  IMUdata Pointer to a struct of type MPU6050_HandleTypeDef
 * @return None
 */
void MPUFilter_FilterAngularVelocity(MPU6050_HandleTypeDef* IMUdata){
    MPUFilter_writeInput(&vzFilter, IMUdata->_Z_GYRO);
    IMUdata->_Z_GYRO = MPUFilter_readOutput(&vzFilter);

    MPUFilter_writeInput(&vyFilter, IMUdata->_Y_GYRO);
    IMUdata->_Y_GYRO = MPUFilter_readOutput(&vyFilter);

    MPUFilter_writeInput(&vxFilter, IMUdata->_X_GYRO);
    IMUdata->_X_GYRO = MPUFilter_readOutput(&vxFilter);
}

/**
 * @}
 */
/* end IMU_Filter */
