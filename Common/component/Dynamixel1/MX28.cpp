/**
  *****************************************************************************
  * @file   MX28.cpp
  * @author Tyler Gamvrelis
  *
  * @ingroup MX28
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "MX28.h"




namespace dynamixel{
/*********************************** MX28 ***********************************/
// Public
// ----------------------------------------------------------------------------
MX28::MX28(
    uint8_t id,
    DaisyChain* daisyChain,
    ResolutionDivider divider
)
    :   Motor(id, daisyChain, divider)
{

}

MX28::~MX28(){

}

bool MX28::setBaudRate(uint32_t baud) const{

}

bool MX28::setGoalVelocity(float goalVelocity) const{

}

bool MX28::getVelocity(float& retVal) const{

}

}




/**
 * @}
 */
/* end - MX28 */
