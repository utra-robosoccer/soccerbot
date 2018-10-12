/**
  *****************************************************************************
  * @file    Dynamixel.h
  * @author  Tyler
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup Dynamixel
  * @{
  *****************************************************************************
  */




#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H




/********************************* Includes **********************************/
#include <cstdint>
#include "DaisyChain.h"




/*********************************** Motor ***********************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------

// Register addresses
// ----------------------------------------------------------------------------
/** @brief Baud rate register */
constexpr uint8_t REG_BAUD_RATE           = 0x04;

/** @brief Goal velocity register (0x20 = low byte, 0x21 = high byte) */
constexpr uint8_t REG_GOAL_VELOCITY       = 0x20;

/** @brief Current velocity register (0x26 = low byte, 0x27 = high byte) */
constexpr uint8_t REG_CURRENT_VELOCITY    = 0x26;

// Default register values
// ----------------------------------------------------------------------------
/**
 * @brief Motor broadcast ID (i.e. messages sent to this ID will be sent to all
 *        motors on the bus)
 */
constexpr uint8_t BROADCAST_ID                = 0xFE;

/** @brief Default motor ID */
constexpr uint8_t DEFAULT_ID                  = 0x01;

/**
 * @brief Default time motor waits before returning status packet (microseconds)
 */
constexpr uint16_t DEFAULT_RETURN_DELAY       = 500;

/** @brief Default clockwise angle limit */
constexpr float DEFAULT_CW_ANGLE_LIMIT        = 0.0;

/** @brief Default permitted minimum voltage (0x3C = 60 -> 6.0 V) */
constexpr float DEFAULT_LOW_VOLTAGE_LIMIT     = 6.0;

/** @brief Default maximum torque limit (10-bit resolution percentage) */
constexpr float DEFAULT_MAXIMUM_TORQUE        = 100.0;

/**
 * @brief Default condition(s) under which a status packet will be returned
 *        (all)
 */
constexpr uint8_t DEFAULT_STATUS_RETURN_LEVEL = 0x02;

/** @brief Default condition(s) under which the alarm LED will be set */
constexpr uint8_t DEFAULT_ALARM_LED           = 0x24;

/**
 * @brief Default condition(s) under which the motor will shut down due to an
 *        alarm
 */
constexpr uint8_t DEFAULT_ALARM_SHUTDOWN      = 0x24;

/** @brief Default motor power state */
constexpr bool DEFAULT_TORQUE_ENABLE          = false;

/** @brief Default LED state */
constexpr bool DEFAULT_LED_ENABLE             = false;

/** @brief Default value for the EEPROM lock */
constexpr uint8_t DEFAULT_EEPROM_LOCK         = 0x00;


// Value limit definitions
// ----------------------------------------------------------------------------
/** @brief Minimum angular velocity (RPM) */
constexpr float MIN_VELOCITY = 1.0;

/** @brief Maximum angular position (joint mode) */
constexpr float MAX_ANGLE    = 300.0;

/** @brief Minimum angular position (joint mode) */
constexpr float MIN_ANGLE    = 0.0;

/** @brief Maximum torque (percent of maximum) */
constexpr float MAX_TORQUE   = 100.0;

/** @brief Minimum torque (percent of maximum) */
constexpr float MIN_TORQUE   = 0.0;

/** @brief Maximum operating voltage */
constexpr float MAX_VOLTAGE  = 14.0;

/** @brief Minimum operating voltage */
constexpr float MIN_VOLTAGE  = 6.0;

/** @brief Maximum punch (proportional to minimum current) */
constexpr float MAX_PUNCH = 100.0;

/** @brief Minimum punch (proportional to minimum current) */
constexpr float MIN_PUNCH = 0.0;




// Types & enums
// ----------------------------------------------------------------------------
/** @brief Resolution dividers for the motors supported by the library */
enum class ResolutionDivider : uint16_t{
    AX12A = 1023,
    MX28 = 4095
};

/** @brief Voltage limits that can be set. @see setVoltageLimit */
enum class VoltageLimit{
    HIGHEST, /**< Set the highest voltage the motor will operate at */
    LOWEST   /**< Set the lowest voltage the motor will operate at  */
};

/** @brief Cases the motor can be configured to return a status packet for */
enum class StatusReturnLevel : uint8_t{
    PING_ONLY = 0,
    READS_ONLY = 1,
    ALL_COMMANDS = 2,
    NUM_LEVELS
};

/** @brief Actions the motor can take when an alarm condition is satisfied */
enum class AlarmType : uint8_t{
    LED,
    SHUTDOWN,
    NUM_TYPES
};

/**
 * @brief Conditions for which the motor can be programmed to respond with an
 *        alarm action
 */
enum class AlarmCondition : uint8_t{
    INPUT_VOLTAGE_ERR = 0,
    ANGLE_LIMIT_ERR = 1, /**< Goal angle is set beyond the limits */
    OVERHEATING_ERR = 2,
    RANGE_ERR = 3,
    CHECKSUM_ERR = 4,
    OVERLOAD_ERR = 5,    /**< Motor cannot exert enough torque    */
    INSTRUCTION_ERR = 7,
    NUM_CONDITIONS
};



// Classes and structs
// ----------------------------------------------------------------------------
class Motor{
public:
    /**
     * @brief Motor constructor. It is the responsibility of the caller to
     *        ensure that the ID of this data structure matches that of the
     *        motor out in the field (i.e. no I/O is performed to set the ID).
     *        Additionally, the caller must choose the correct
     *        ResolutionDivider for the actuator being used.
     * @note The motor is initialized to use joint mode by default. Using
     *       wheel mode must always be specified afterward via function call
     * @param id The ID used in command packets sent for this motor
     * @param daisyChain I/O manager class for the port this motor is attached
     *        to
     * @param divider Resolution divider for this motor
     */
    Motor(
        uint8_t id,
        DaisyChain* daisyChain,
        ResolutionDivider divider
    );

    virtual ~Motor();

    /**
     * @brief Resets motor control table
     * @details Resets the control table values of the motor to the factory
     *          default value settings. Note that post-reset, the motor ID will
     *          be 1. Thus, if several motors with ID 1 are connected on the
     *          same bus, there will not be a way to assign them unique IDs
     *          without first disconnecting them. Need to wait a short amount
     *          of time before the motor can be commanded again (500 ms is
     *          definitely safe, but you can probably get away with less)
     * @return true if successful, otherwise false
     */
    bool reset();


    // Setters (use the WRITE DATA instruction)
    // ------------------------------------------------------------------------

    // EEPROM
    // ------------------------------------------------------------------------
    /**
     * @brief Sets the ID (identification number) for the current motor
     * @details Note that the instruction will be broadcasted using the current ID.
     *          As such, if the ID is not known, the motor ID should be initialized
     *          to the broadcast ID (0xFE) in the constructor
     *
     *          Default value: 1
     * @param id the number between 0 and 252 or equal to 254 to identify the
     *        motor. If 0xFE (254), any messages broadcasted to that ID will be
     *        broadcasted to all motors
     * @return true if successful, otherwise false
     */
    bool setId(uint8_t id);

    /**
     * @brief Sets the time, in microseconds, that the motor should wait before
     *        returning a status packet
     * @param microSec the time in microseconds to delay. Arguments in range
     *        [2, 508] are valid
     * @return true if successful, otherwise false
     */
    bool setReturnDelayTime(uint16_t microSec) const;

    /**
     * @brief Sets the clockwise angle limit for the current motor
     * @details If maxAngle for CCW angle limit is 0 AND minAngle for CW angle
     *          limit is 0, then motor is in wheel mode where it can
     *          continuously rotate. Otherwise, motor is in joint mode where
     *          its motion is constrained between the set bounds
     * @param minAngle the minimum angle for all motor operations. Arguments
     *        between 0 and 300 are valid
     * @return true if successful, otherwise false
     */
    bool setCWAngleLimit(float minAngle) const;

    /**
     * @brief Sets the counter-clockwise angle limit for the current motor
     * @details If maxAngle for CCW angle limit is 0 AND minAngle for CW angle
     *          limit is 0, then motor is in wheel mode where it can
     *          continuously rotate. Otherwise, motor is in joint mode where
     *          its motion is constrained between the set bounds
     * @param maxAngle the maximum angle for all motor operations. Arguments
     *        between 0 and 300 are valid
     * @return true if successful, otherwise false
     */
    bool setCCWAngleLimit(float maxAngle) const;

    /**
     * @brief Sets the highest or lowest operating voltage limit for the motor
     *        Default register value: 140 (0xBE) which is 14.0 volts
     * @param limit the voltage limit to set
     * @param voltage the value to set the limit to, in volts
     * @return true if successful, otherwise false
     */
    bool setVoltageLimit(VoltageLimit limit, float voltage) const;

    /**
     * @brief Sets the maximum torque limit for all motor operations
     * @details Default value: 0x3FF (100%)
     * @param maxTorque the maximum torque as a percentage (max: 100.0). Gets
     *        converted to a 10-bit number
     * @return true if successful, otherwise false
     */
    bool setMaxTorque(float maxTorque) const;

    /**
     * @brief Sets the conditions under which a status packet will be returned
     * @param level The status return level. @see StatusReturnLevel
     * @return true if successful, otherwise false
     */
    bool setStatusReturnLevel(StatusReturnLevel level) const;

    /**
     * @brief Sets the conditions under which the motor will enter an alarm
     *        state. Each condition which is enabled can be programmed to turn
     *        on the motor's LED and/or disable the motor's torque ("shut
     *        down")
     * @details Register bits may be set simultaneously in the motor
     * @return true if successful, otherwise false
     */
    bool setAlarm(AlarmType type, AlarmCondition condition) const;

    // RAM
    // ------------------------------------------------------------------------
    /**
     * @brief Enables or disables torque for current motor
     * @param isEnabled true if torque should be set to on, otherwise false
     * @return true if successful, otherwise false
     */
    bool enableTorque(bool isEnabled) const;

    /**
     * @brief Sets the state of the motor's LED
     * @param isEnabled true if the LED should be turned on, otherwise false
     * @return true if successful, otherwise false
     */
    bool enableLed(bool isEnabled) const;

    /**
     * @brief Sets the goal position of the motor in RAM
     * @param goalAngle the desired angular position. Arguments between 0 and
     *        300 are valid. Note that 150 corresponds to the middle position
     * @return true if successful, otherwise false
     */
    bool setGoalPosition(float goalAngle) const;

    /**
     * @brief Sets the torque limit for the motor in RAM
     * @param goalTorque The percentage of the maximum possible torque (max:
     *        100). Gets converted into a 10-bit number
     * @return true if successful, otherwise false
     */
    bool setGoalTorque(float goalTorque) const;

    /**
     * @brief Locks the EEPROM until the next power cycle
     * @return true if successful, otherwise false
     */
    bool lockEEPROM() const;

    /**
     * @brief Sets a quantity proportional to the minimum current supplied to the
     *        motor during operation. WARNING: setting this to 100.0 will cause
     *        the actuators to heat up very rapidly!
     * @details Units are not specified in the datasheet, and therefore this
     *          function is not entirely useful without sufficient testing
     * @param punch for now, arguments in range [0, 100.0] are valid
     * @return true if successful, otherwise false
     */
    bool setPunch(float punch) const;


    // Getters (use READ DATA instruction)
    // ------------------------------------------------------------------------
    /**
     * @brief Reads the angular position of the motor in degrees
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getPosition(float& retVal) const;

    /**
     * @brief Reads the "load", the percentage of the maximum torque the motor
     *        is exerting. Counterclockwise loads are positive and clockwise
     *        are negative
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getLoad(float& retVal) const;

    /**
     * @brief   Reads the motor supply voltage
     * @param[out] retVal R-val return type
     * @return true if successful, otherwise false
     */
    bool getVoltage(float& retVal) const;

    /**
     * @brief Reads the internal motor temperature. Results are in degrees
     *        Celsius
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getTemperature(uint8_t& retVal) const;

    /**
     * @brief   Indicates whether the motor is operating in joint mode or wheel
     *          mode
     * @details Reads the CW (addr: 0x06) and CCW (addr: 0x08) angle limits. If
     *          both are 0, motor is in wheel mode and can spin indefinitely.
     *          Otherwise, motor is in joint mode and has angle limits set
     * @param[out] retVal true if in joint mode, false if in wheel mode
     * @return true if successful, otherwise false
     */
    bool isJointMode(bool& retVal);

    // Other motor instruction functions
    /**
     * @brief Implementation of the PING instruction
     * @details Used only for returning a status packet or checking the
     *          existence of a motor with a specified ID. Does not command any
     *          operations
     * @param[out] The motor ID seen in status packet if received a valid
     *          status packet, otherwise the max uint8_t value
     * @return true if successful, otherwise false
     */
    bool ping(uint8_t& retVal) const;

    /**
     * @brief Indicates whether the motor is in motion
     * @param retVal true if motor is moving, otherwise false
     * @return true if successful, otherwise false
     */
    bool isMoving(bool& retVal) const;

    /**
     * @brief Sets the control registers such that the rotational angle of the
     *        motor is not bounded
     * @note To prevent undesired behaviour, the goal velocity should be
     *       set right after calling this function since the motor could easily
     *       rotate out of control after a successful call to this function
     * @return true if successful, otherwise false
     */
    bool enterWheelMode();

    /**
     * @brief Sets the control registers such that the rotational angle of the
     *        motor is constrained between the values in the angle limit
     *        registers
     * @return true if successful, otherwise false
     */
    bool enterJointMode();

    /** @brief See child implementation for details */
    virtual bool setBaudRate(uint32_t baud) const = 0;
    virtual bool setGoalVelocity(float goalVelocity) const = 0;
    virtual bool getVelocity(float& retVal) const = 0;

protected:
    /**
     * @brief Sends an array of data to a motor as per its configuration details
     * @details Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
     * @param args an array of arguments of the form `{ADDR, PARAM_1, ... ,
     *        PARAM_N}`
     * @param numArgs this must be equal to `sizeof(args)`, and must be either 2
     *        or 3
     * @return true if successful, otherwise false
     */
    bool dataWriter(uint8_t* args, size_t numArgs) const;

    /**
     * @brief Reads data of a specified length from a given address in the
     *        motor
     * @details Uses the READ DATA instruction, 0x02, in the motor instruction set.
     *          The status packet returned will be of the following form
     *
     *          @code{.c}
     *          {0xFF, 0xFF, ID, LENGTH, ERR, PARAM_1,...,PARAM_N, CHECKSUM}
     *          @endcode
     *
     *          Where N = readLength
     * @param readAddr The address inside the motor memory table where reading
     *        is to begin
     * @param readLength The number of bytes to be read. Must be either 1 or 2
     * @param[out] val R-value return type
     * @return true if successful and checksums match, otherwise false
     */
    bool dataReader(
        uint8_t readAddr,
        uint8_t readLength,
        uint16_t& retVal
    ) const;

    /** @brief true if motor is in joint mode, false if in wheel mode */
    bool m_isJointMode;

private:
    /** @brief Motor identification (0-252, 0xFE) */
    uint8_t id;

    const uint16_t resolutionDivider; /**< @see ResolutionDivider            */
    const DaisyChain* daisyChain;     /**< @see DaisyChain                   */
};

} // end namespace dynamixel




/**
 * @}
 */
/* end - Dynamixel */

#endif /* DYNAMIXEL_H */
