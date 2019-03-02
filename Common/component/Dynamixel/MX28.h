/**
  *****************************************************************************
  * @file    MX28.h
  * @author  Tyler
  *
  * @defgroup MX28
  * @brief MX28-specific functions. Note that multi-turn mode and resolution
  *        divider tweaking are not supported
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef MX28_H
#define MX28_H




/********************************* Includes **********************************/
#include "Dynamixel/Dynamixel.h"




/*********************************** MX28 ************************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Default register values
// ----------------------------------------------------------------------------
/** @brief Default baud rate register setting */
constexpr uint32_t MX28_DEFAULT_BAUD_RATE = 57600;

/** @brief Default counter-clockwise angle limit */
constexpr float MX28_DEFAULT_CCW_ANGLE_LIMIT = 300.0;

/** @brief Default permitted maximum voltage */
constexpr float MX28_DEFAULT_HIGHEST_VOLTAGE_LIMIT = 16.0;

/** @brief Default derivative gain parameter value */
constexpr uint8_t MX28_DEFAULT_D_GAIN = 0x00;

/** @brief Default integral gain parameter value */
constexpr uint8_t MX28_DEFAULT_I_GAIN = 0x00;

/** @brief Default proportional gain parameter value */
constexpr uint8_t MX28_DEFAULT_P_GAIN = 0x20;

/** @brief Default punch */
constexpr float MX28_DEFAULT_PUNCH = 0.0;

// Value limit definitions
// ----------------------------------------------------------------------------
/** @brief Maximum angular velocity (RPM) */
constexpr float MX28_MAX_VELOCITY = 117.0;




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------
class MX28 : public Motor{
public:
    /** @see Motor */
    MX28(
        uint8_t id,
        DaisyChain* daisyChain
    );

    ~MX28();



    // Setters (use the WRITE DATA instruction)
    // ------------------------------------------------------------------------

    // EEPROM
    // ------------------------------------------------------------------------
    /**
     * @brief Sets the baud rate for communication with the motor
     * @param baud the baud rate, valid for arguments in range [9600, 3500000]
     * @return true if successful, otherwise false
     */
    bool setBaudRate(uint32_t baud) const override;

    // RAM
    // ------------------------------------------------------------------------
    /**
     * @brief Sets the goal velocity of the motor in RAM
     * @param goalVelocity the goal velocity in RPM. Arguments of 0-117 are
     *        valid when in joint mode. 0 corresponds to MAX motion in joint
     *        mode, and minimum motion in wheel mode. In wheel mode, negative
     *        arguments correspond to CW rotation
     * @return true if successful, otherwise false
     */
    bool setGoalVelocity(float goalVelocity) const override;

    /**
     * @brief Sets the goal acceleration. The argument should be in units of
     *        degree/s^2. Direction is determined by the sign of the goal
     *        position or goal velocity (depending on if the motor is in
     *        joint or wheel mode, respectively)...I think
     * @details goalAcceleration of 0 or goalSpeed of 0 means no
     *          control over acceleration (uses max acceleration of motor)
     * @note All arguments are converted to an integer multiple of 8.583
     * @param goalAcceleration the target acceleration in degree/s^2. Values
     *        in range [0, 2180] are permitted
     * @return true if successful, otherwise false
     */
    bool setGoalAcceleration(float goalAcceleration) const;

    /**
     * @brief Sets the value of the derivative gain used in the motor's PID
     *        controller
     * @details kD = DGain / 250
     * @param DGain the derivative gain parameter
     * @return true if successful, otherwise false
     */
    bool setDGain(uint8_t DGain) const;

    /**
     * @brief Sets the value of the integral gain used in the motor's PID
     *        controller
     * @details kI = IGain * 125/256
     * @param IGain the integral gain parameter
     * @return true if successful, otherwise false
     */
    bool setIGain(uint8_t IGain) const;

    /**
     * @brief Sets the value of the proportional gain used in the motor's PID
     *        controller
     * @details kP = PGain / 8
     * @param PGain the proportional gain parameter
     * @return true if successful, otherwise false
     */
    bool setPGain(uint8_t PGain) const;


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
/* end - MX28 */

#endif /* MX28_H */
