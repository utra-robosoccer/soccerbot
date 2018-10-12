/**
  *****************************************************************************
  * @file    AX12A.h
  * @author  Tyler
  *
  * @defgroup AX12A
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef AX12A_H
#define AX12A_H




/********************************* Includes **********************************/
#include "Dynamixel.h"




/*********************************** Motor ***********************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Default register values
// ----------------------------------------------------------------------------
// TODO(tyler) convert these to values of the proper type (i.e. stuff the user
// would be able to use
/** @brief Default baud rate register setting */
constexpr uint8_t AX12A_DEFAULT_BAUD_RATE              =  0x01;

/** @brief Default counter-clockwise angle limit */
constexpr uint16_t AX12A_DEFAULT_CCW_ANGLE_LIMIT       =  0x03FF;

/** @brief Default permitted maximum voltage (0xBE = 140 -> 14.0 V) */
constexpr uint8_t AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT  =  0xBE;

/** @brief Default clockwise compliance margin (position error) */
constexpr uint8_t AX12A_DEFAULT_CW_COMPLIANCE_MARGIN   =  0x01;

/** @brief Default counter-clockwise compliance margin (position error) */
constexpr uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN  =  0x01;

/** @brief Default clockwise compliance slope (torque near goal position) */
constexpr uint8_t AX12A_DEFAULT_CW_COMPLIANCE_SLOPE    =  0x20;

/**
 * @brief Default counter-clockwise compliance slope (torque near goal
 *        position)
 */
constexpr uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE   =  0x20;

/** @brief Default punch */
constexpr uint16_t AX12A_DEFAULT_PUNCH                 =  0x0020;

// Value limit definitions
// ----------------------------------------------------------------------------
/** @brief Maximum angular velocity (RPM) */
constexpr uint8_t AX12A_MAX_VELOCITY = 114;




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------
class AX12A : Motor{
public:
    /** @see Motor */
    AX12A(
        uint8_t id,
        DaisyChain* daisyChain
    );

    ~AX12A();


    // Setters (use the WRITE DATA instruction)
    // ------------------------------------------------------------------------

    // EEPROM
    // ------------------------------------------------------------------------
    /**
     * @brief Sets the baud rate for communication with the motor
     * @param baud the baud rate, valid for arguments in range [7844, 1000000]
     * @return true if successful, otherwise false
     */
    bool setBaudRate(uint32_t baud) const override;

    // RAM
    // ------------------------------------------------------------------------
    /**
     * @brief Sets the goal velocity of the motor in RAM
     * @param goalVelocity the goal velocity in RPM. Arguments of 0-114 are
     *        valid when in joint mode. 0 corresponds to MAX motion in joint
     *        mode, and minimum motion in wheel mode. In wheel mode, negative
     *        arguments correspond to CW rotation
     * @return true if successful, otherwise false
     */
    bool setGoalVelocity(float goalVelocity) const override;

    // TODO: Consider making these use enums
    void setCWComplianceMargin(uint8_t CWcomplianceMargin);
    void setCCWComplianceMargin(uint8_t CCWcomplianceMargin);
    void setCWComplianceSlope(uint8_t CWcomplianceSlope);
    void setCCWComplianceSlope(uint8_t CCWcomplianceSlope);

    void setComplianceSlope(uint8_t complianceSlope);
    void setComplianceMargin(uint8_t complianceSlope);


    // Getters (use the READ DATA instruction)
    // ------------------------------------------------------------------------
    /**
     * @brief Reads the angular velocity of the motor, in RPM
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getVelocity(float& retVal) const override;
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - AX12A */

#endif /* AX12A_H */
