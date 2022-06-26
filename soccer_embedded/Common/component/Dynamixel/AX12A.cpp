/**
  *****************************************************************************
  * @file
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
    DaisyChain* daisy_chain
)
    :   Motor(id, daisy_chain, ResolutionDivider::AX12A)
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

bool AX12A::setGoalVelocity(float goal_velocity) const{
    if(m_is_joint_mode){
        if((goal_velocity < MIN_VELOCITY) || (goal_velocity > AX12A_MAX_VELOCITY)){
            return false;
        }
    }

    uint16_t normalized_value;
    if(goal_velocity > 0){
        normalized_value = static_cast<uint8_t>(
            goal_velocity / AX12A_MAX_VELOCITY * 1023
        );
    }
    else{
        normalized_value = static_cast<uint16_t>(
            (goal_velocity * -1) / AX12A_MAX_VELOCITY * 1023
        );

        normalized_value |= 0b000010000000000;
    }

    uint8_t low_byte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t high_byte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_GOAL_VELOCITY, low_byte, high_byte};
    return dataWriter(args, sizeof(args));
}

bool AX12A::setCwComplianceMargin(uint8_t margin) const{
    uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_MARGIN, margin};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setCcwComplianceMargin(uint8_t margin) const{
    uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_MARGIN, margin};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setCwComplianceSlope(uint8_t slope) const{
    if(slope > 7){
        return false;
    }

    uint8_t step = 1 << slope;

    uint8_t arr[2] = {AX12A_REG_CW_COMPLIANCE_SLOPE, step};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setCcwComplianceSlope(uint8_t slope) const{
    if(slope > 7){
        return false;
    }

    uint8_t step = 1 << slope;

    uint8_t arr[2] = {AX12A_REG_CCW_COMPLIANCE_SLOPE, step};
    return dataWriter(arr, sizeof(arr));
}

bool AX12A::setComplianceSlope(uint8_t slope) const{
    bool success = setCcwComplianceSlope(slope);

    if(success){
        success = setCwComplianceSlope(slope);
    }

    return success;
}

bool AX12A::setComplianceMargin(uint8_t margin) const{
    bool success = setCcwComplianceMargin(margin);

    if(success){
        success = setCwComplianceMargin(margin);
    }

    return success;
}

bool AX12A::getVelocity(float& velocity_out) const{
    uint16_t raw = 0;
    bool ok = dataReader(REG_CURRENT_VELOCITY, 2, raw);

    if(!ok){
        return false;
    }

    uint16_t modifier = m_is_joint_mode ? 1023 : 2047;
    velocity_out = static_cast<float>(raw / modifier * AX12A_MAX_VELOCITY);

    return ok;
}

}




/**
 * @}
 */
/* end - AX12A */
