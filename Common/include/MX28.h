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




/*********************************** MX28 ************************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Default register values
// ----------------------------------------------------------------------------
/** @brief Default baud rate register setting */
constexpr uint8_t MX28_DEFAULT_BAUD_RATE              =   0x22;

/** @brief Default counter-clockwise angle limit */
constexpr uint16_t MX28_DEFAULT_CCW_ANGLE_LIMIT       =   0x0FFF;

/** @brief Default permitted maximum voltage (0xA0 = 160 -> 16.0 V) */
constexpr uint8_t MX28_DEFAULT_HIGHEST_VOLTAGE_LIMIT  =   0xA0;

/** @brief Default derivative gain parameter value */
constexpr uint8_t MX28_DEFAULT_D_GAIN                 =   0x08;

/** @brief Default integral gain parameter value */
constexpr uint8_t MX28_DEFAULT_I_GAIN                 =   0x00;

/** @brief Default proportional gain parameter value */
constexpr uint8_t MX28_DEFAULT_P_GAIN                 =   0x08;

/** @brief Default punch */
constexpr uint16_t MX28_DEFAULT_PUNCH                 =   0x0000;

// Value limit definitions
// ----------------------------------------------------------------------------
/** @brief Maximum angular velocity (RPM) */
constexpr uint8_t MX28_MAX_VELOCITY = 117;




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------
class MX28 : Motor{
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
     *        degree/s^2
     * @details Special: goalAcceleration of 0 means no control over
     *          acceleration (uses max acceleration of motor)
     * @param goalAcceleration the target acceleration in degree/s^2
     * @return true if successful, otherwise false
     */
    void setGoalAcceleration(float goalAcceleration) const;

    // TODO: Consider making these use enums
    void setMultiTurnOffset(int16_t offset) const;
    void enterMultiTurnMode() const;

    void setDGain(uint8_t DGain) const;
    void setIGain(uint8_t IGain) const;
    void setPGain(uint8_t PGain) const;


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
