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




/******************************** File-local *********************************/
namespace{
// Constants
// ----------------------------------------------------------------------------

// Register addresses
// ----------------------------------------------------------------------------
/** @brief Clockwise compliance margin register */
constexpr uint8_t AX12A_REG_CW_COMPLIANCE_MARGIN  = 0x1A;

/** @brief Counter-clockwise compliance margin register */
constexpr uint8_t AX12A_REG_CCW_COMPLIANCE_MARGIN = 0x1B;

/** @brief Clockwise compliance slope register */
constexpr uint8_t AX12A_REG_CW_COMPLIANCE_SLOPE   = 0x1C;

/** @brief Counter-clockwise compliance slope register */
constexpr uint8_t AX12A_REG_CCW_COMPLIANCE_SLOPE  = 0x1D;

} // end anonymous namespace




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

bool AX12A::setCwComplianceMargin(uint8_t cwComplianceMargin) const{
    uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_MARGIN, cwComplianceMargin};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setCcwComplianceMargin(uint8_t ccwComplianceMargin) const{
    uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_MARGIN, ccwComplianceMargin};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setCwComplianceSlope(uint8_t cwComplianceSlope) const{
    if(cwComplianceSlope > 7){
        return false;
    }

    uint8_t step = 1 << cwComplianceSlope;

    uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_SLOPE, step};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setCcwComplianceSlope(uint8_t ccwComplianceSlope) const{
    if(ccwComplianceSlope > 7){
        return false;
    }

    uint8_t step = 1 << ccwComplianceSlope;

    uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_SLOPE, step};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setComplianceSlope(uint8_t complianceSlope) const{
    bool success = setCcwComplianceSlope(complianceSlope);

    if(success){
        success = setCwComplianceSlope(complianceSlope);
    }

    return success;
}

bool AX12A::setComplianceMargin(uint8_t complianceMargin) const{
    bool success = setCcwComplianceMargin(complianceMargin);

    if(success){
        success = setCwComplianceMargin(complianceMargin);
    }

    return success;
}

}




/**
 * @}
 */
/* end - AX12A */
