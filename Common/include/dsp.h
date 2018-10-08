/**
  *****************************************************************************
  * @file    MPUFilter.h
  * @author  Tyler
  *
  * @ingroup  DSP
  *****************************************************************************
  */




#ifndef DSP_H
#define DSP_H




/********************************* Includes **********************************/
#include "SystemConf.h" // Need to include this before arm_math
#include <arm_math.h> // Include CMSIS header
#include <string.h> // For memset




/*********************************** dsp *************************************/
namespace dsp{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class fir_f32 Wrapper for arm_fir_f32 C object. Implements a FIR with float
 *        data type
 */
template<uint16_t T_numTaps, uint32_t T_blockSize>
class fir_f32{
public:
    /**
     * @brief Initialize the filter. Writes go into the filter buffer if
     *        blockSize > 1, and after blockSize samples have been written,
     *        the shift register contents are updated
     * @param numTaps The number of taps in the filter (equal to number of
     *        coefficients
     * @param pCoeffs Pointer to the array of filter coefficients
     * @param blockSize The number of input samples to buffer before updating
     *        the filter output
     * @param startVal The starting value
     */
    inline void init(
        uint16_t numTaps,
        float32_t* pCoeffs,
        uint32_t blockSize,
        float startVal = 0
    )
    {
        arm_fir_init_f32(
            &instance,
            numTaps,
            pCoeffs,
            state,
            blockSize
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

    /**
     * @brief  Write an input (or block of inputs) into the filter
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
    arm_fir_instance_f32 instance;                  /**< Filter instance               */
    float32_t state[T_numTaps + T_blockSize] = {0}; /**< Previous inputs and new input */
};

} // end namespace dsp




/**
 * @}
 */
/* end - Header */

#endif /* DSP_H */
