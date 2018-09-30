/**
  *****************************************************************************
  * @file    MPUFilter.h
  * @author  Tyler
  *
  * @ingroup  DSP
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
// Link with library: libarm_cortexM4_mathL.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path
// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp




#ifndef DSP_H
#define DSP_H




/********************************** Macros ***********************************/
// Note: This needs to be defined before arm_math.h is included
#define ARM_MATH_CM4 // Use ARM Cortex M4




/********************************* Includes **********************************/
#include "stm32f446xx.h" // __FPU_PRESENT == 1 ==> generate FPU instructions
#include <arm_math.h> // Include CMSIS header




/*********************************** dsp *************************************/
namespace dsp{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class Wrapper class for arm_fir_f32 C object. Implements a FIR with float
 * data type
 */
template<uint16_t T_numTaps, uint32_t T_blockSize>
class fir_f32{
public:
    /**
     * @brief Initialize the filter
     * @param numTaps The number of taps in the filter (equal to number of
     *        coefficients
     * @param pCoeffs Pointer to the array of filter coefficients
     * @param blockSize The number of input samples to buffer before updating
     *        the filter output
     */
    inline void init(
        uint16_t numTaps,
        float32_t* pCoeffs,
        uint32_t blockSize
    )
    {
        arm_fir_init_f32(
            &instance,
            numTaps,
            pCoeffs,
            state,
            blockSize
        );
    }

    /**
     * @brief  Write an input into the filter. Writes go into the filter buffer
     *         if blockSize > 1, and after blockSize samples have been written,
     *         the shift register contents are updated
     * @param  dataSrc Array of new data to be written into the filter
     * @param  dataDest Array of output data, where the i-th element is the
     *         filter output after writing the i-th input from dataSrc
     * @param  blockSize Number of samples to be processed in this batch
     * @note   (count(dataSrc) == count(dataDest) == blockSize) must be true
     */
    inline void update(float* dataSrc, float* dataDest, uint32_t blockSize){
        arm_fir_f32(&instance, dataSrc, dataDest, blockSize);
    }

private:
    arm_fir_instance_f32 instance;  /**< Filter instance               */
    float32_t state[T_numTaps + T_blockSize] = {0}; /**< Previous inputs and new input */
};

} // end namespace dsp




/**
 * @}
 */
/* end - Header */

#endif /* DSP_H */
