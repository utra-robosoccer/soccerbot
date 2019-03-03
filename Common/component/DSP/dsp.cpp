/**
  *****************************************************************************
  * @file
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

void fir_f32::update(float* data_src, float* data_dest, uint32_t block_size){
    arm_fir_f32(&m_inst, data_src, data_dest, block_size);
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
const float32_t imuVelocityFilter::m_velocity_coeff[11] =
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
    float start_value
)
{
    arm_fir_init_f32(
        &m_inst,
        11,
        const_cast<float32_t*>(m_velocity_coeff),
        m_state,
        1
    );

    if(start_value == 0){
        memset(m_state, start_value, sizeof(m_state));
    }
    else{
        for(size_t i = 0; i < sizeof(m_state); ++i){
            m_state[i] = start_value;
        }
    }
}

} // end namespace dsp



/**
 * @}
 */
/* end - DSP */
