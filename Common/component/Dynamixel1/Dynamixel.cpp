/**
  *****************************************************************************
  * @file    Dynamixel.cpp
  * @author  Tyler Gamvrelis
  *
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "Dynamixel.h"




/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------




/******************************** File-local *********************************/
namespace{
// Constants
// ----------------------------------------------------------------------------
// Instruction set definitions
constexpr uint8_t INST_PING       = 0x01; /**< Gets a status packet  */
constexpr uint8_t INST_READ_DATA  = 0x02; /**< Reads data from a motor register */
constexpr uint8_t INST_WRITE_DATA = 0x03; /**< Writes data for immediate execution */
constexpr uint8_t INST_REG_WRITE  = 0x04; /**< Registers an instruction to be executed at a later time */
constexpr uint8_t INST_ACTION     = 0x05; /**< Triggers instructions registered by INST_REG_WRITE */
constexpr uint8_t INST_RESET      = 0x06; /**< Resets the control tables of the Dynamixel actuator(s) specified */
constexpr uint8_t INST_SYNC_WRITE = 0x83; /**< Writes on a specified address with a specified data length on multiple devices */

// Register addresses
constexpr uint8_t REG_ID                  = 0x03; /**< Motor ID register */
constexpr uint8_t REG_BAUD_RATE           = 0x04; /**< Baud rate register */
constexpr uint8_t REG_RETURN_DELAY_TIME   = 0x05; /**< Status packet return delay time register */
constexpr uint8_t REG_CW_ANGLE_LIMIT      = 0x06; /**< Clockwise angle limit register (0x06 = low byte, 0x07 = high byte) */
constexpr uint8_t REG_CCW_ANGLE_LIMIT     = 0x08; /**< Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high byte) */
constexpr uint8_t REG_HIGH_VOLTAGE_LIMIT  = 0x0C; /**< Maximum voltage limit register */
constexpr uint8_t REG_LOW_VOLTAGE_LIMIT   = 0x0D; /**< Minimum voltage limit register */
constexpr uint8_t REG_MAX_TORQUE          = 0x0E; /**< Maximum torque limit register (0x0E = low byte, 0x0F = high byte) */
constexpr uint8_t REG_STATUS_RETURN_LEVEL = 0x10; /**< Status packet return condition(s) register */
constexpr uint8_t REG_ALARM_LED           = 0x11; /**< Alarm LED condition(s) register */
constexpr uint8_t REG_ALARM_SHUTDOWN      = 0x12; /**< Alarm shutdown condition(s) register */
constexpr uint8_t REG_TORQUE_ENABLE       = 0x18; /**< Motor power control register */
constexpr uint8_t REG_LED_ENABLE          = 0x19; /**< LED control register */
constexpr uint8_t REG_GOAL_POSITION       = 0x1E; /**< Goal position register (0x1E = low byte, 0x1F = high byte) */
constexpr uint8_t REG_GOAL_VELOCITY       = 0x20; /**< Goal velocity register (0x20 = low byte, 0x21 = high byte) */
constexpr uint8_t REG_GOAL_TORQUE         = 0x22; /**< Goal torque register (0x22 = low byte, 0x23 = high byte) */
constexpr uint8_t REG_LOCK_EEPROM         = 0x2F; /**< EEPROM lock register */
constexpr uint8_t REG_PUNCH               = 0x30; /**< Punch (0x30 = low register, 0x31 = high register) */
constexpr uint8_t REG_CURRENT_POSITION    = 0x24; /**< Current position register (0x24 = low byte, 0x25 = high byte) */
constexpr uint8_t REG_CURRENT_VELOCITY    = 0x26; /**< Current velocity register (0x26 = low byte, 0x27 = high byte) */
constexpr uint8_t REG_CURRENT_LOAD        = 0x28; /**< Current load register (0x28 = low byte, 0x29 = high byte) */
constexpr uint8_t REG_CURRENT_VOLTAGE     = 0x2A; /**< Current voltage register */
constexpr uint8_t REG_CURRENT_TEMPERATURE = 0x2B; /**< Current temperature register */
constexpr uint8_t REG_REGISTERED          = 0x2C; /**< Command execution status register */
constexpr uint8_t REG_MOVING              = 0x2E; /**< Motor motion register */




/********************************* Constants *********************************/
// Default register values
constexpr uint8_t BROADCAST_ID                = 0xFE;       /**< Motor broadcast ID (i.e. messages sent to this ID will be sent to all motors on the bus) */
constexpr uint8_t DEFAULT_ID                  = 0x01;       /**< Default motor ID */
constexpr uint8_t DEFAULT_RETURN_DELAY        = 0xFA;       /**< Default time motor waits before returning status packet (microseconds) */
constexpr uint16_t DEFAULT_CW_ANGLE_LIMIT     = 0x0000;     /**< Default clockwise angle limit */
constexpr uint8_t DEFAULT_LOW_VOLTAGE_LIMIT   = 0x3C;       /**< Default permitted minimum voltage (0x3C = 60 -> 6.0 V) */
constexpr uint16_t DEFAULT_MAXIMUM_TORQUE     = 0x03FF;     /**< Default maximum torque limit (10-bit resolution percentage) */
constexpr uint8_t DEFAULT_STATUS_RETURN_LEVEL = 0x02;       /**< Default condition(s) under which a status packet will be returned (all) */
constexpr uint8_t DEFAULT_ALARM_LED           = 0x24;       /**< Default condition(s) under which the alarm LED will be set */
constexpr uint8_t DEFAULT_ALARM_SHUTDOWN      = 0x24;       /**< Default condition(s) under which the motor will shut down due to an alarm */
constexpr uint8_t DEFAULT_TORQUE_ENABLE       = 0x00;       /**< Default motor power state */
constexpr uint8_t DEFAULT_LED_ENABLE          = 0x00;       /**< Default LED state */
constexpr uint8_t DEFAULT_EEPROM_LOCK         = 0x00;       /**< Default value for the EEPROM lock */

// Value limit definitions
constexpr float MIN_VELOCITY = 1.0;        /**< Minimum angular velocity (RPM) */
constexpr float MAX_ANGLE    = 300.0;      /**< Maximum angular position (joint mode) */
constexpr float MIN_ANGLE    = 0.0;        /**< Minimum angular position (joint mode) */
constexpr float MAX_TORQUE   = 100.0;      /**< Maximum torque (percent of maximum) */
constexpr float MIN_TORQUE   = 0.0;        /**< Minimum torque (percent of maximum) */
constexpr float MAX_VOLTAGE  = 14.0;       /**< Maximum operating voltage */
constexpr float MIN_VOLTAGE  = 6.0;        /**< Minimum operating voltage */
constexpr uint16_t MAX_PUNCH = 1023;       /**< Maximum punch (proportional to minimum current) */
constexpr uint16_t MIN_PUNCH = 0;          /**< Minimum punch (proportional to minimum current) */




// Types & enums
// ----------------------------------------------------------------------------




// Classes and structs
// ----------------------------------------------------------------------------




// Variables
// ----------------------------------------------------------------------------




// Functions
// ----------------------------------------------------------------------------
/**
 * @brief  Compute the checksum for data passes in, according to a modular
 *         checksum algorithm employed by the Dynamixel V1.0 protocol
 * @param  arr the array to be ran through the checksum function
 * @param  length the total length of the array arr
 * @return The 1-byte number that is the checksum
 */
static inline uint8_t computeChecksum(uint8_t *arr, size_t length){
    uint8_t accumulate = 0;

    /* Loop through the array starting from the 2nd element of the array and
     * finishing before the last since the last is where the checksum will
     * be stored */
    for(uint8_t i = 2; i < length - 1; i++){
        accumulate += arr[i];
    }

    return (~accumulate) & 0xFF; // Lower 8 bits of the logical NOT of the sum
}

} // end anonymous namespace




namespace dynamixel{
/*********************************** Motor ***********************************/
// Public
// ----------------------------------------------------------------------------
Motor::Motor(
    uint8_t id,
    DaisyChain* daisyChain
)
    :
        id(id),
        daisyChain(daisyChain)
{
    lastReadIsValid = false;
}

Motor::~Motor(){

}

void Motor::setGoalPosition(float goalAngle){
    /* Check for input validity. If input not valid, replace goalAngle with closest
     * valid value to ensure code won't halt. */
    if((goalAngle < MIN_ANGLE) || (goalAngle > MAX_ANGLE)){
        if(goalAngle > MIN_ANGLE){
            goalAngle = MAX_ANGLE;
        }
        else{
            goalAngle = MIN_ANGLE;
        }
    }

    /* Translate the angle from degrees into a 10- or 12-bit number. */
//    uint16_t normalized_value = 0;
//    if(hdynamixel -> _motorType == AX12ATYPE){
//        normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 1023);
//    }
//    else if(hdynamixel -> _motorType == MX28TYPE){
//        normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 4095);
//    }
    uint16_t normalized_value = (uint16_t)(goalAngle / MAX_ANGLE * 1023);

    uint8_t lowByte = (uint8_t)(normalized_value & 0xFF); // Low byte of goal position
    uint8_t highByte = (uint8_t)((normalized_value >> 8) & 0xFF); // High byte of goal position

    /* Write data to motor. */
    uint8_t args[3] = {REG_GOAL_POSITION, lowByte, highByte};
    dataWriter(args, sizeof(args));
}




// Protected
// ----------------------------------------------------------------------------
/**
 * @brief   Sends an array of data to a motor as per its configuration details
 * @details Uses the WRITE DATA instruction, 0x03, in the motor instruction set.
 * @param   args an array of arguments of the form `{ADDR, PARAM_1, ... ,
 *          PARAM_N}`
 * @param   numArgs this must be equal to `sizeof(args)`, and must be either 2
 *          or 3
 * @return  None
 */
void Motor::dataWriter(
    uint8_t* args,
    size_t numArgs
)
{
    // Check validity so that we don't accidentally write something invalid
    if(numArgs <= 3){
        uint8_t arrTransmit[9];

        arrTransmit[0] = 0xFF;
        arrTransmit[1] = 0xFF;
        arrTransmit[2] = id;
        arrTransmit[3] = 2 + numArgs;
        arrTransmit[4] = INST_WRITE_DATA;

        for(uint8_t i = 0; i < numArgs; i ++){
            arrTransmit[5 + i] = args[i];
        }

        // Checksum
        arrTransmit[4 + numArgs + 1] = computeChecksum(arrTransmit, 4 + numArgs + 2);

        // Transmit
        daisyChain->requestTransmission(arrTransmit, 4 + numArgs + 2);
    }
}




// Private
// ----------------------------------------------------------------------------


}




/**
 * @}
 */
/* end - Dynamixel */
