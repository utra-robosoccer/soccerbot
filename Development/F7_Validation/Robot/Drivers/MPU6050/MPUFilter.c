/**
  *****************************************************************************
  * @file    MPUFilter.c
  * @author  Tyler
  * @brief   Digital filtering data structures and functions, for IMU data.
  *
  * @defgroup IMU_Filter IMU Filter
  * @ingroup  MPU6050
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
 * @brief Number of filter taps (equal to number of coefficients here) for
 *        angular velocity filter
 */
static const uint16_t MPUFilter_numTapsVel = 11;

/** @brief Number of samples buffered before re-computing output */
static const uint16_t MPUFilter_blockSizeVel = 1;

/**
 * @brief Filter coefficients for angular velocity filter. (11 - 1)/2 = 5
 *        sample delay
 */
static const float MPUFilter_coefficientsVel[11] =
{
    0.030738841, 0.048424201, 0.083829062, 0.11125669, 0.13424691, 0.14013315,
    0.13424691, 0.11125669, 0.083829062, 0.048424201, 0.030738841
};




/********************************** Types ************************************/
/** @brief Container for angular velocity filter */
typedef struct
{
    arm_fir_instance_f32 instance; /**< Filter instance                      */
    float32_t state[12];           /**< Previous inputs and new input        */
    float32_t output;              /**< Output of filter                     */
} MPUFilterVel_t;




/***************************** Private Variables *****************************/
/** @brief Filter for angular velocity along x-axis */
static MPUFilterVel_t vxFilter;

/** @brief Filter for angular velocity along y-axis */
static MPUFilterVel_t vyFilter;

/** @brief Filter for angular velocity along z-axis */
static MPUFilterVel_t vzFilter;




/***************************** Private Functions *****************************/
/**
 * @defgroup IMU_Filter_Private_Functions Private functions
 * @brief Functions used internally for writing inputs and initialization
 * @ingroup IMU_Filter
 * @{
 */

// TODO: figure out how to link to the dsp library so we can use the
// filtering functions

/**
 * @brief  Write an input to the filter
 * @param  pThis Pointer to filter container
 * @param  input The new input to be written
 * @return None
 */
static inline void MPUFilter_writeInputVel(MPUFilterVel_t * pThis, float input){
//    arm_fir_f32( &pThis->instance, &input, &pThis->output, 1 );
};

/**
 * @brief  Read the current filter output
 * @param  pThis Pointer to filter container
 * @return None
 */
static inline float MPUFilter_readOutputVel( MPUFilterVel_t * pThis ){
    return pThis->output;
}

/**
 * @brief  Reset a velocity filter
 * @param  pThis Pointer to filter container
 * @return None
 */
static void MPUFilter_resetVel( MPUFilterVel_t * pThis )
{
    memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
    pThis->output = 0;                                  // Reset output
}

/**
 * @brief  Initialize a velocity filter
 * @param  pThis Pointer to filter container
 * @return None
 */
static void MPUFilter_initVel( MPUFilterVel_t * pThis )
{
//    arm_fir_init_f32(&pThis->instance, MPUFilter_numTapsVel,
//            (float*)MPUFilter_coefficientsVel, pThis->state,
//            MPUFilter_blockSizeVel
//    );
    MPUFilter_resetVel( pThis );
}

/**
 * @}
 */
/* end - IMU_Filter_Private_Functions */




/****************************** Public Functions *****************************/
/**
 * @defgroup IMU_Filter_Public_Functions Public functions
 * @brief Functions used outside the module
 * @ingroup IMU_Filter
 * @{
 */

/**
 * @brief  Initialize acceleration and angular velocity FIR filters for IMU
 *         data.
 * @return None
 */
void MPUFilter_InitAllFilters(){
    MPUFilter_initVel(&vxFilter);
    MPUFilter_initVel(&vyFilter);
    MPUFilter_initVel(&vzFilter);
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
/* end - IMU_Filter_Public_Functions */

/**
 * @}
 */
/* end IMU_Filter */
