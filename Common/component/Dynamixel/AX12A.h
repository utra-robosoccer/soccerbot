/**
  *****************************************************************************
  * @file
  * @author  Tyler
  *
  * @defgroup AX12A
  * @brief AX12A-specific functionality
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef AX12A_H
#define AX12A_H




/********************************* Includes **********************************/
#include "Dynamixel/Dynamixel.h"




/*********************************** AX12A ***********************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Default register values
// ----------------------------------------------------------------------------
/** @brief Default baud rate register setting */
constexpr uint32_t AX12A_DEFAULT_BAUD_RATE = 1000000;

/** @brief Default counter-clockwise angle limit */
constexpr float AX12A_DEFAULT_CCW_ANGLE_LIMIT = 300.0;

/** @brief Default permitted maximum voltage */
constexpr float AX12A_DEFAULT_HIGHEST_VOLTAGE_LIMIT = 14.0;

/** @brief Default clockwise compliance margin */
constexpr uint8_t AX12A_DEFAULT_CW_COMPLIANCE_MARGIN = 0x01;

/** @brief Default counter-clockwise compliance margin */
constexpr uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_MARGIN = 0x01;

/** @brief Default clockwise compliance slope */
constexpr uint8_t AX12A_DEFAULT_CW_COMPLIANCE_SLOPE = 5;

/** @brief Default counter-clockwise compliance slope */
constexpr uint8_t AX12A_DEFAULT_CCW_COMPLIANCE_SLOPE = 5;

/** @brief Default punch */
constexpr float AX12A_DEFAULT_PUNCH = 3.13; // 0x0020 reg val

// Value limit definitions
// ----------------------------------------------------------------------------
/** @brief Maximum angular velocity (RPM) */
constexpr float AX12A_MAX_VELOCITY = 114.0;




// Classes and structs
// ----------------------------------------------------------------------------
class AX12A : public Motor{
public:
    /** @see Motor */
    AX12A(
        uint8_t id,
        DaisyChain* daisy_chain
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
     * @param goal_velocity the goal velocity in RPM. Arguments of 0-114 are
     *        valid when in joint mode. 0 corresponds to MAX motion in joint
     *        mode, and minimum motion in wheel mode. In wheel mode, negative
     *        arguments correspond to CW rotation
     * @return true if successful, otherwise false
     */
    bool setGoalVelocity(float goal_velocity) const override;

    /**
     * @brief Sets the motor's clockwise compliance margin, that is, the
     *        acceptable error between the current and goal position, if the
     *        current position is CW of the goal position
     * @details ~0.29 times the argument passed in equals the the CW compliance
     *          margin that the motor is set to have
     * @param margin the CW compliance margin. Arguments in range
     *        [0, 255]
     * @return true if successful, otherwise false
     */
    bool setCwComplianceMargin(uint8_t margin) const;

    /**
     * @brief Sets the motor's counter-clockwise compliance margin, that is,
     *        the acceptable error between the current and goal position, if
     *        the current position is CW of the goal position
     * @details ~0.29 times the argument passed in equals the the CCW
     *          compliance margin that the motor is set to have
     * @param margin the CCW compliance margin. Arguments in
     *        range [0, 255]
     * @return true if successful, otherwise false
     */
    bool setCcwComplianceMargin(uint8_t margin) const;

    /**
     * @brief Sets the motor's clockwise compliance slope, that is, the level
     *        of torque near the goal position. A higher value indicates that
     *        the level of torque is more heavily reduced as the goal position
     *        is approached; a lower value indicates that the torque will not
     *        be reduced much as the goal position is approached
     * @param slope arguments in range [0, 7], with 0 being the
     *        least flexible
     * @return true if successful, otherwise false
     */
    bool setCwComplianceSlope(uint8_t slope) const;

    /**
     * @brief Sets the motor's counter-clockwise compliance slope, that is, the
     *        level of torque near the goal position. A higher value indicates
     *        that the level of torque is more heavily reduced as the goal
     *        position is approached; a lower value indicates that the torque
     *         will not be reduced much as the goal position is approached
     * @param slope arguments in range [0, 7], with 0 being the
     *        least flexible
     * @return true if successful, otherwise false
     */
    bool setCcwComplianceSlope(uint8_t slope) const;

    /**
     * @brief Sets the motor's counter-clockwise and clockwise compliance
     *        slopes
     * @see setCcwComplianceSlope
     * @see setCwComplianceSlope
     * @param slope arguments in range [0, 7], with 0 being the
     *        least flexible
     * @return true if successful, otherwise false
     */
    bool setComplianceSlope(uint8_t slope) const;

    /**
     * @brief Sets the motor's counter-clockwise and clockwise compliance
     *        margins
     * @see setCcwComplianceMargin
     * @see setCwComplianceMargin
     * @param margin arguments in range [0, 255]
     * @return true if successful, otherwise false
     */
    bool setComplianceMargin(uint8_t margin) const;


    // Getters (use the READ DATA instruction)
    // ------------------------------------------------------------------------
    /**
     * @brief Reads the angular velocity of the motor, in RPM
     * @param[out] velocity_out R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getVelocity(float& velocity_out) const override;
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - AX12A */

#endif /* AX12A_H */
