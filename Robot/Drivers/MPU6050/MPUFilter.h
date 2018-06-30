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

// Begin header file, MPUFilter.h

#ifndef MPUFILTER_H_ // Include guards
#define MPUFILTER_H_

#define ARM_MATH_CM4	// Use ARM Cortex M4
#define __FPU_PRESENT 1		// Does this device have a floating point unit?
#include <arm_math.h>	// Include CMSIS header

// Link with library: libarm_cortexM4_mathL.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path
extern float32_t MPUFilter_coefficients[21];

typedef struct
{
	arm_fir_instance_f32 instance;
	float32_t state[37];
	float32_t output;
} MPUFilterType;

void MPUFilter_init( MPUFilterType * pThis );
void MPUFilter_reset( MPUFilterType * pThis );
inline void MPUFilter_writeInput(MPUFilterType * pThis, float input){
	arm_fir_f32( &pThis->instance, &input, &pThis->output, 1 );
};
inline float MPUFilter_readOutput( MPUFilterType * pThis ){
	return pThis->output;
}


int MPUFilter_filterBlock( MPUFilterType * pThis, float * pInput, float * pOutput, unsigned int count );
#define MPUFilter_outputToFloat( output )  \
	(output)

#define MPUFilter_inputFromFloat( input )  \
	(input)

#endif // MPUFILTER_H_

