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




/******************************** File-local *********************************/
namespace{
// Constants
// ----------------------------------------------------------------------------

// Register addresses
// ----------------------------------------------------------------------------
/** @brief Register to fine-tune "0" position */
constexpr uint8_t MX28_REG_MULTI_TURN_OFFSET  = 0x14;

/** @brief Register to change how many bits of resolution are used */
constexpr uint8_t MX28_REG_RESOLUTION_DIVIDER = 0x16;

/** @brief Derivative gain register */
constexpr uint8_t MX28_REG_D_GAIN             = 0x1A;

/** @brief Integral gain register */
constexpr uint8_t MX28_REG_I_GAIN             = 0x1B;

/** @brief Proportional gain register */
constexpr uint8_t MX28_REG_P_GAIN             = 0x1C;

/** @brief Goal acceleration register */
constexpr uint8_t MX28_REG_GOAL_ACCELERATION  = 0x49;

} // end anonymous namespace




namespace dynamixel{
/*********************************** MX28 ***********************************/
// Public
// ----------------------------------------------------------------------------
MX28::MX28(
    uint8_t id,
    DaisyChain* daisyChain
)
    :   Motor(id, daisyChain, ResolutionDivider::MX28)
{

}

MX28::~MX28(){

}

bool MX28::setBaudRate(uint32_t baud) const{
    if((baud < 9600) || (baud > 3500000)){
        return false;
    }

    uint8_t args[2];
    args[0] = REG_BAUD_RATE;

    if(baud >= 2250000){
        if(baud < 2500000){
            args[1] = 250;
        }
        else if(baud < 3000000){
            args[1] = 251;
        }
        else{
            args[1] = 252;
        }
    }
    else{
        args[1] = static_cast<uint8_t>((2000000 / baud) - 1);
    }

    return dataWriter(args, sizeof(args));
}

bool MX28::setGoalVelocity(float goalVelocity) const{
    if(m_isJointMode){
        if((goalVelocity < MIN_VELOCITY) || (goalVelocity > MX28_MAX_VELOCITY)){
            return false;
        }
    }

    uint16_t normalized_value;
    if(goalVelocity > 0){
        normalized_value = static_cast<uint8_t>(
            goalVelocity / MX28_MAX_VELOCITY * 1023
        );
    }
    else{
        normalized_value = static_cast<uint16_t>(
            (goalVelocity * -1) / MX28_MAX_VELOCITY * 1023
        );

        normalized_value |= 0b000010000000000;
    }

    uint8_t lowByte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t highByte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_GOAL_VELOCITY, lowByte, highByte};
    return dataWriter(args, sizeof(args));
}

bool MX28::getVelocity(float& retVal) const{
    uint16_t raw = 0;
    bool success = dataReader(REG_CURRENT_VELOCITY, 2, raw);

    if(!success){
        return false;
    }

    uint16_t modifier = m_isJointMode ? 1023 : 2047;
    retVal = static_cast<float>(raw / modifier * MX28_MAX_VELOCITY);

    return success;
}

bool MX28::setGoalAcceleration(float goalAcceleration) const{
    if((goalAcceleration > 2180) || (goalAcceleration < 0)){
        return false;
    }

    uint8_t accelArg = static_cast<int8_t>(goalAcceleration / 8.583);

    uint8_t args[2] = {MX28_REG_GOAL_ACCELERATION, accelArg};
    return dataWriter(args, sizeof(args));
}

bool MX28::setDGain(uint8_t DGain) const{
    uint8_t args[2] = {MX28_REG_D_GAIN, DGain};
    return dataWriter(args, sizeof(args));
}

bool MX28::setIGain(uint8_t IGain) const{
    uint8_t args[2] = {MX28_REG_I_GAIN, IGain};
    return dataWriter(args, sizeof(args));
}

bool MX28::setPGain(uint8_t PGain) const{
    uint8_t args[2] = {MX28_REG_P_GAIN, PGain};
    return dataWriter(args, sizeof(args));
}

}




/**
 * @}
 */
/* end - MX28 */
