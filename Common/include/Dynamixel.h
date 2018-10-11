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




/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------




/************************** insert module name here **************************/
namespace dynamixel{
// Constants
// ----------------------------------------------------------------------------




// Types & enums
// ----------------------------------------------------------------------------
// TODO(tyler) could update this to be a "MotorSpec" class, which contains
// a valid set of values (limits, defaults, other constants) for each
// motor supported by the library. Motor would then have these all as
// constants set at construction, or maybe it would contain a pointer to the
// struct of constants (maybe such setters and getters can be inlined)

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

    ~Motor();
//    void reset();

    // Setters (use the WRITE DATA instruction)
    // EEPROM
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

//    void setBaudRate(uint32_t baud);

    /**
     * @brief Sets the time, in microseconds, that the motor should wait before
     *        returning a status packet
     * @param microSec the time in microseconds to delay. Arguments in range
     *        [2, 508] are valid
     * @return true if successful, otherwise false
     */
    bool setReturnDelayTime(uint16_t microSec);

//    void setCWAngleLimit(float minAngle);
//    void setCCWAngleLimit(float maxAngle);

    /**
     * @brief Sets the highest or lowest operating voltage limit for the motor
     *        Default register value: 140 (0xBE) which is 14.0 volts
     * @param limit the voltage limit to set
     * @param voltage the value to set the limit to, in volts
     * @return true if successful, otherwise false
     */
    bool setVoltageLimit(VoltageLimit limit, float voltage);

    /**
     * @brief Sets the maximum torque limit for all motor operations
     * @details Default value: 0x3FF (100%)
     * @param   maxTorque the maximum torque as a percentage (max: 100.0). Gets
     *          converted to a 10-bit number
     * @return true if successful, otherwise false
     */
    bool setMaxTorque(float maxTorque);

    /**
     * @brief Sets the conditions under which a status packet will be returned
     * @param level The status return level. @see StatusReturnLevel
     * @return true if successful, otherwise false
     */
    bool setStatusReturnLevel(StatusReturnLevel level);

    /**
     * @brief Sets the conditions under which the motor will enter an alarm
     *        state. Each condition which is enabled can be programmed to turn
     *        on the motor's LED and/or disable the motor's torque ("shut
     *        down")
     * @details Register bits may be set simultaneously in the motor
     * @return true if successful, otherwise false
     */
    bool setAlarm(AlarmType type, AlarmCondition condition);

    // RAM
    /**
     * @brief Enables or disables torque for current motor
     * @param isEnabled true if torque should be set to on, otherwise false
     * @return true if successful, otherwise false
     */
    bool enableTorque(bool isEnabled);

    /**
     * @brief Sets the state of the motor's LED
     * @param isEnabled true if the LED should be turned on, otherwise false
     * @return true if successful, otherwise false
     */
    bool enableLed(bool isEnabled);

    /**
     * @brief Sets the goal position of the motor in RAM
     * @param goalAngle the desired angular position. Arguments between 0 and
     *        300 are valid. Note that 150 corresponds to the middle position
     * @return true if successful, otherwise false
     */
    bool setGoalPosition(float goalAngle);

//    void setGoalVelocity(float goalVelocity);

    /**
     * @brief Sets the torque limit for the motor in RAM
     * @param goalTorque The percentage of the maximum possible torque (max:
     *        100). Gets converted into a 10-bit number
     * @return true if successful, otherwise false
     */
    bool setGoalTorque(float goalTorque);

    /**
     * @brief Locks the EEPROM until the next power cycle
     * @return true if successful, otherwise false
     */
    bool lockEEPROM();

    /**
     * @brief Sets a quantity proportional to the minimum current supplied to the
     *        motor during operation. WARNING: setting this to 100.0 will cause
     *        the actuators to heat up very rapidly!
     * @details Units are not specified in the datasheet, and therefore this
     *          function is not entirely useful without sufficient testing
     * @param punch for now, arguments in range [0, 100.0] are valid
     * @return true if successful, otherwise false
     */
    bool setPunch(float punch);

    // Getters (use READ DATA instruction)
    /**
     * @brief Reads the angular position of the motor in degrees
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getPosition(float& retVal);


//    bool getVelocity(float& retVal);

    /**
     * @brief Reads the "load", the percentage of the maximum torque the motor
     *        is exerting. Counterclockwise loads are positive and clockwise
     *        are negative
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getLoad(float& retVal);

    /**
     * @brief   Reads the motor supply voltage
     * @param[out] retVal R-val return type
     * @return true if successful, otherwise false
     */
    bool getVoltage(float& retVal);

    /**
     * @brief Reads the internal motor temperature. Results are in degrees
     *        Celsius
     * @param[out] retVal R-val return type (not modified upon failure)
     * @return true if successful, otherwise false
     */
    bool getTemperature(uint8_t& retVal);
//    bool isRegistered();
//    bool isMoving();
//    bool isJointMode();
//
//    // Other motor instruction functions
//    int8_t ping();
//
//    // Interfaces for previously-defined functions
//    void enterWheelMode(float goalVelocity);
//    void enterJointMode();

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
    bool dataWriter(uint8_t* args, size_t numArgs);

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
    bool dataReader(uint8_t readAddr, uint8_t readLength, uint16_t& retVal);

private:
    uint8_t id;                       /**< Motor identification (0-252, 0xFE) */
//    bool isJointMode;                 /**< true if motor is in joint mode, false if in wheel mode */
    const uint16_t resolutionDivider; /**< @see ResolutionDivider             */
    const DaisyChain* daisyChain;     /**< @see DaisyChain                    */
};



// Functions
// ----------------------------------------------------------------------------




} // end namespace Dynamixel




/***************************** Inline functions ******************************/




/**
 * @}
 */
/* end - Dynamixel */

#endif /* DYNAMIXEL_H */
