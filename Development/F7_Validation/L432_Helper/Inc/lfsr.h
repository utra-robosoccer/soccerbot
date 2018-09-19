/**
  *****************************************************************************
  * @file    lfsr.h
  * @author  Tyler
  *
  * @ingroup LFSR
  * @{
  *****************************************************************************
  */




#ifndef LFSR_H
#define LFSR_H




/********************************* Includes **********************************/
#include <stdint.h>




/****************************** Inline Functions *****************************/
/**
 * @brief   Returns a representation for x^6 + x^5 + 1 over GF2. This
 *          polynomial produces a maximal-length pseudo-random sequence given
 *          its length (period 63)
 * @details 0*x^7 + 1*x^6 + 1*x^5 + 0*x^4 + 0*x^3 + 0*x^2 + 0*x^1 implies a
 *          bitmask of 0110000 since each power of x indicates a bit shift. The
 *          x^0 term always feeds back into the shift register through the taps
 */
#define POLY_MASK_PERIOD_63 0b0110000

/**
 * @brief   Generates a pseudo-random noise sequence based on a linear feedback
 *          shift register (lfsr), which repeats after a period dependent on
 *          the polynomial structure
 * @details Accepting the lfsr as an argument (as opposed to making it static
 *          inside the function) allows the function to be inlined which is
 *          good
 * @param   lfsr Pointer to the linear shift register
 * @param   polynomial The GF2 polynomial which represents the locations of
 *          the taps to the modulo-2 adders from the feedback line
 * @return  Pseudo-random noise byte
 */
inline void lfsr_get_noise(uint32_t* lfsr, const uint32_t polynomial){
    uint32_t feedback_line = *lfsr & 1;
    *lfsr >>= 1;

    // For any binary digit A:
    //     A xor 0 = A
    //     A xor 1 = !A
    //
    // The 1's in the polynomial indicate bits that the feedback line is
    // connected to via modulo-2 adders (i.e. xor). Given the above rules,
    // we only need to compute this addition when the feedback line is a 1
    // since there is no update to the lfsr contents (besides the shift)
    // when the feedback line is 0
    if(feedback_line == 1){
        *lfsr ^= polynomial;
    }
}



/**
 * @}
 */
/* end TODO -- module name defined on line 7 */

#endif /* LFSR_H */
