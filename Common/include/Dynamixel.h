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
/**
 * @brief Enumerates the resolution dividers of the motors supported by the
 *        library
 */
enum class ResolutionDivider : uint16_t{
    AX12A = 1023,
    MX28 = 4095
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
//
    // Setters (use the WRITE DATA instruction)
//    // EEPROM
//    void setID(uint8_t ID);
//    void setBaudRate(uint32_t baud);
//    void setReturnDelayTime(uint16_t microSec);
//    void setCWAngleLimit(float minAngle);
//    void setCCWAngleLimit(float maxAngle);
//    void setHighestVoltageLimit(float highestVoltage);
//    void setLowestVoltageLimit(float lowestVoltage);
//    void setMaxTorque(float maxTorque);
//    void setStatusReturnLevel(uint8_t status_data);
//    void setAlarmLED(uint8_t alarm_LED_data);
//    void setAlarmShutdown(uint8_t alarm_shutdown_data);

    // RAM
    /**
     * @brief Enables or disables torque for current motor
     * @param isEnabled
     *            - if 1, then generates torque by impressing power to the motor
     *            - if 0, then interrupts power to the motor to prevent it from
     *              generating torque
     * @return true if successful, otherwise false
     */
    bool enableTorque(bool isEnabled);

    /**
     * @brief Sets the state of the motor's LED
     * @param isEnabled
     *            - if 1, LED is on
     *            - if 0, LED is off
     * @return true if successful, otherwise false
     */
    bool enableLed(bool isEnabled);

    /**
     * @brief Sets the goal position of the motor in RAM
     * @details Takes a double between 0 and 300, encodes this position in an
     *          upper and lower byte pair, and sends this information (along
     *          with requisites) to the motor
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
//    void setPunch(float punch);
//
    // Getters (use READ DATA instruction)
    /**
     * @brief Reads the angular position of the motor in degrees
     * @param[out] retVal R-val return type
     * @return true if successful, otherwise false
     */
    bool getPosition(float& retVal);
//    void getVelocity();
//    void getLoad();
//    float getVoltage();
//    uint8_t getTemperature();
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
     * @brief   Sends an array of data to a motor as per its configuration details
     * @details Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
     * @param   args an array of arguments of the form `{ADDR, PARAM_1, ... ,
     *          PARAM_N}`
     * @param   numArgs this must be equal to `sizeof(args)`, and must be either 2
     *          or 3
     * @return  true if successful, otherwise false
     */
    bool dataWriter(uint8_t* args, size_t numArgs);

    /**
     * @brief   Reads data of a specified length from a given address in the
     *          motor
     * @details Uses the READ DATA instruction, 0x02, in the motor instruction set.
     *          The status packet returned will be of the following form
     *
     *          @code{.c}
     *          {0xFF, 0xFF, ID, LENGTH, ERR, PARAM_1,...,PARAM_N, CHECKSUM}
     *          @endcode
     *
     *          Where N = readLength
     * @param   readAddr The address inside the motor memory table where reading
     *          is to begin
     * @param   readLength The number of bytes to be read. Must be either 1 or 2
     * @param[out] val R-value return type
     * @return  true if successful and checksums match, otherwise false
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
