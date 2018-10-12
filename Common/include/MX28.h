/**
  *****************************************************************************
  * @file    MX28.h
  * @author  Tyler
  *
  * @defgroup MX28
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef MX28_H
#define MX28_H




/********************************* Includes **********************************/
#include "Dynamixel.h"




/*********************************** Motor ***********************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Default register values
// ----------------------------------------------------------------------------


// Value limit definitions
// ----------------------------------------------------------------------------




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------
class MX28 : Motor{
public:
    /** @see Motor */
    MX28(
        uint8_t id,
        DaisyChain* daisyChain,
        ResolutionDivider divider
    );

    ~MX28();



    // Setters (use the WRITE DATA instruction)
    // ------------------------------------------------------------------------
    bool setBaudRate(uint32_t baud) const override;
    bool setGoalVelocity(float goalVelocity) const override;

    // TODO(tyler) add the other ones for PID, etc


    // Getters (use the READ DATA instruction)
    // ------------------------------------------------------------------------
    bool getVelocity(float& retVal) const override;
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - MX28 */

#endif /* MX28_H */
