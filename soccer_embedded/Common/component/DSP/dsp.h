/**
  *****************************************************************************
  * @file
  * @author  Tyler
  *
  * @defgroup Header
  * @ingroup DSP
  *****************************************************************************
  */




#ifndef DSP_H
#define DSP_H




/********************************* Includes **********************************/
#include "SystemConf.h" // Need to include this before arm_math
#include <arm_math.h> // Include CMSIS header




/*********************************** dsp *************************************/
namespace dsp{
// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @class fir_f32 Wrapper for arm_fir_f32 C object. Implements a FIR with float
 *        data type. This class is abstract and it is expected that derived
 *        classes will be filters suited for a specific purpose with fixed-size
 *        state buffers
 */
class fir_f32{
public:
    virtual ~fir_f32();

    /**
     * @brief Initialize the filter
     * @param start_value The starting value
     */
    virtual void init(
        float start_value = 0
    ) = 0;

    /**
     * @brief  Write an input (or block of inputs) into the filter
     * @param  data_src Array of new data to be written into the filter
     * @param  data_dest Array of output data, where the i-th element is the
     *         filter output after writing the i-th input from data_src
     * @param  block_size Number of samples to be processed in this batch
     */
    void update(float* data_src, float* data_dest, uint32_t block_size);

protected:
    arm_fir_instance_f32 m_inst; /**< Filter instance */
};


/**
 * @class imuVelocityFilter FIR filter for angular velocity data from the IMU
 */
class ImuVelocityFilter : public fir_f32{
public:
    ImuVelocityFilter();
    ~ImuVelocityFilter();

    /**
     * @brief Initialize the filter by configuring its coefficients, state
     *        buffer and block size
     * @param startVal The starting value
     */
    void init(
        float startVal = 0
    ) override final;

private:
    /**
     * @brief Filter coefficients for angular velocity filter. Coefficients
     *        generated using MicroModeler DSP, a free online tool
     */
    static const float32_t m_velocity_coeff[11];

    float32_t m_state[12]; /**< Filter state */
};

} // end namespace dsp




/**
 * @}
 */
/* end - Header */

#endif /* DSP_H */
