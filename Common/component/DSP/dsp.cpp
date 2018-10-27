/**
  *****************************************************************************
  * @file    dsp.cpp
  * @author  Tyler Gamvrelis
  *
  * @defgroup DSP
  * @brief Digital signal processing routines
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "dsp.h"
#include <string.h> // For memset




namespace dsp{
/********************************** fir_f32 **********************************/
// Public
// ----------------------------------------------------------------------------
fir_f32::~fir_f32(){

}

void fir_f32::update(float* dataSrc, float* dataDest, uint32_t blockSize){
    arm_fir_f32(&instance, dataSrc, dataDest, blockSize);
}


/**************************** imuVelocityFilter ******************************/
// Data members
// ----------------------------------------------------------------------------
/******************************* SOURCE LICENSE *********************************
Copyright (c) 2018 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/
// Link with library: libarm_cortexM4_mathL.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path
// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp
/**
 * @brief Filter coefficients for angular velocity filter. Coefficients
 *        generated using MicroModeler DSP, a free online tool
 */
const float32_t imuVelocityFilter::velCoeff[11] =
{
    0.030738841, 0.048424201, 0.083829062, 0.11125669, 0.13424691, 0.14013315,
    0.13424691, 0.11125669, 0.083829062, 0.048424201, 0.030738841
};


// Public
// ----------------------------------------------------------------------------
imuVelocityFilter::imuVelocityFilter(){

}

imuVelocityFilter::~imuVelocityFilter(){

}

void imuVelocityFilter::init(
    float startVal
)
{
    arm_fir_init_f32(
        &instance,
        11,
        const_cast<float32_t*>(velCoeff),
        state,
        1
    );

    if(startVal == 0){
        memset(state, startVal, sizeof(state));
    }
    else{
        for(size_t i = 0; i < sizeof(state); ++i){
            state[i] = startVal;
        }
    }
}

} // end namespace dsp



/**
 * @}
 */
/* end - DSP */
