/**
  *****************************************************************************
  * @file   AX12A.cpp
  * @author Tyler Gamvrelis
  *
  * @ingroup AX12A
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "AX12A.h"




namespace dynamixel{
/*********************************** AX12A ***********************************/
// Public
// ----------------------------------------------------------------------------
AX12A::AX12A(
    uint8_t id,
    DaisyChain* daisyChain,
    ResolutionDivider divider
)
    :   Motor(id, daisyChain, divider)
{

}

AX12A::~AX12A(){

}

bool AX12A::setBaudRate(uint32_t baud) const{

}

bool AX12A::setGoalVelocity(float goalVelocity) const{

}

bool AX12A::getVelocity(float& retVal) const{

}

}




/**
 * @}
 */
/* end - AX12A */
