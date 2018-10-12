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
    DaisyChain* daisyChain
)
    :   Motor(id, daisyChain, ResolutionDivider::AX12A)
{

}

AX12A::~AX12A(){

}

bool AX12A::setBaudRate(uint32_t baud) const{
    if((baud < 7844) || (baud > 1000000)){
        return false;
    }

    uint8_t args[2];
    args[0] = REG_BAUD_RATE;
    args[1] = static_cast<uint8_t>((2000000 / baud) - 1);
    return dataWriter(args, sizeof(args));
}

bool AX12A::setGoalVelocity(float goalVelocity) const{
    if(m_isJointMode){
        if((goalVelocity < MIN_VELOCITY) || (goalVelocity > AX12A_MAX_VELOCITY)){
            return false;
        }
    }

    uint16_t normalized_value;
    if(goalVelocity > 0){
        normalized_value = static_cast<uint8_t>(
            goalVelocity / AX12A_MAX_VELOCITY * 1023
        );
    }
    else{
        normalized_value = static_cast<uint16_t>(
            (goalVelocity * -1) / AX12A_MAX_VELOCITY * 1023
        );

        normalized_value |= 0b000010000000000;
    }

    uint8_t lowByte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t highByte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_GOAL_VELOCITY, lowByte, highByte};
    return dataWriter(args, sizeof(args));
}

bool AX12A::getVelocity(float& retVal) const{
    uint16_t raw = 0;
    bool success = dataReader(REG_CURRENT_VELOCITY, 2, raw);

    if(!success){
        return false;
    }

    uint16_t modifier = m_isJointMode ? 1023 : 2047;
    retVal = static_cast<float>(raw / modifier * AX12A_MAX_VELOCITY);

    return success;
}

}




/**
 * @}
 */
/* end - AX12A */
