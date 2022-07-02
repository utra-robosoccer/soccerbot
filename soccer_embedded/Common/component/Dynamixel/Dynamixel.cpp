/**
  *****************************************************************************
  * @file
  * @author Tyler Gamvrelis
  * @author Gokul Dharan
  * @author Hannah Lee
  *
  * @ingroup Dynamixel
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "Dynamixel.h"
#include <math.h>




/******************************** File-local *********************************/
namespace{
// Constants
// ----------------------------------------------------------------------------

// Instruction set definitions
// ----------------------------------------------------------------------------
/** @brief Gets a status packet  */
constexpr uint8_t INST_PING       = 0x01;

/** @brief Reads data from a motor register */
constexpr uint8_t INST_READ_DATA  = 0x02;

/** @brief Writes data for immediate execution */
constexpr uint8_t INST_WRITE_DATA = 0x03;

/** @brief Registers an instruction to be executed at a later time */
constexpr uint8_t INST_REG_WRITE  = 0x04;

/** @brief Triggers instructions registered by INST_REG_WRITE */
constexpr uint8_t INST_ACTION     = 0x05;

/** @brief Resets the control tables of the Dynamixel actuator(s) specified */
constexpr uint8_t INST_RESET      = 0x06;

/**
 * @brief Writes on a specified address with a specified data length on multiple
 *        devices
 */
constexpr uint8_t INST_SYNC_WRITE = 0x83;

// Register addresses
// ----------------------------------------------------------------------------
/** @brief Motor ID register */
constexpr uint8_t REG_ID                  = 0x03;

/** @brief Baud rate register */
constexpr uint8_t REG_BAUD_RATE           = 0x04;

/** @brief Status packet return delay time register */
constexpr uint8_t REG_RETURN_DELAY_TIME   = 0x05;

/**
 * @brief Clockwise angle limit register (0x06 = low byte, 0x07 = high byte)
 */
constexpr uint8_t REG_CW_ANGLE_LIMIT      = 0x06;

/**
 * @brief Counter-clockwise angle limit register (0x08 = low byte, 0x09 = high
 *        byte)
 */
constexpr uint8_t REG_CCW_ANGLE_LIMIT     = 0x08;

/** @brief Maximum voltage limit register */
constexpr uint8_t REG_HIGH_VOLTAGE_LIMIT  = 0x0C;

/** @brief Minimum voltage limit register */
constexpr uint8_t REG_LOW_VOLTAGE_LIMIT   = 0x0D;

/** @brief Maximum torque limit register (0x0E = low byte, 0x0F = high byte) */
constexpr uint8_t REG_MAX_TORQUE          = 0x0E;

/** @brief Status packet return condition(s) register */
constexpr uint8_t REG_STATUS_RETURN_LEVEL = 0x10;

/** @brief Alarm LED condition(s) register */
constexpr uint8_t REG_ALARM_LED           = 0x11;

/** @brief Alarm shutdown condition(s) register */
constexpr uint8_t REG_ALARM_SHUTDOWN      = 0x12;

/** @brief Motor power control register */
constexpr uint8_t REG_TORQUE_ENABLE       = 0x18;

/** @brief LED control register */
constexpr uint8_t REG_LED_ENABLE          = 0x19;

/** @brief Goal position register (0x1E = low byte, 0x1F = high byte) */
constexpr uint8_t REG_GOAL_POSITION       = 0x1E;

/** @brief Goal torque register (0x22 = low byte, 0x23 = high byte) */
constexpr uint8_t REG_GOAL_TORQUE         = 0x22;

/** @brief EEPROM lock register */
constexpr uint8_t REG_LOCK_EEPROM         = 0x2F;

/** @brief Punch (0x30 = low register, 0x31 = high register) */
constexpr uint8_t REG_PUNCH               = 0x30;

/** @brief Current position register (0x24 = low byte, 0x25 = high byte) */
constexpr uint8_t REG_CURRENT_POSITION    = 0x24;

/** @brief Current load register (0x28 = low byte, 0x29 = high byte) */
constexpr uint8_t REG_CURRENT_LOAD        = 0x28;

/** @brief Current voltage register */
constexpr uint8_t REG_CURRENT_VOLTAGE     = 0x2A;

/** @brief Current temperature register */
constexpr uint8_t REG_CURRENT_TEMPERATURE = 0x2B;

/** @brief Command execution status register */
constexpr uint8_t REG_REGISTERED          = 0x2C;

/** @brief Motor motion register */
constexpr uint8_t REG_MOVING              = 0x2E;




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
    uint8_t m_id,
    DaisyChain* m_daisy_chain,
    ResolutionDivider m_res_divider
)
    :
        m_id(m_id),
        m_daisy_chain(m_daisy_chain),
        m_res_divider(static_cast<uint16_t>(m_res_divider))
{
    m_is_joint_mode = true;
}

Motor::~Motor(){

}

bool Motor::reset(){
    uint8_t arr_transmit[6];

    arr_transmit[0] = 0xff;
    arr_transmit[1] = 0xff;
    arr_transmit[2] = m_id;
    arr_transmit[3] = 2;
    arr_transmit[4] = INST_RESET;
    arr_transmit[5] = computeChecksum(arr_transmit, 6);

    bool success = m_daisy_chain->requestTransmission(
        arr_transmit,
        sizeof(arr_transmit)
    );

    if(success){
        m_id = DEFAULT_ID;
    }

    return success;
}

bool Motor::setId(uint8_t id){
    if((id == 253) || (id == 255)){
        return false;
    }

    // Write data to motor
    uint8_t args[2] = {REG_ID, id};
    bool success = dataWriter(args, sizeof(args));

    if(success){
        this->m_id = id;
    }

    return success;
}

bool Motor::setReturnDelayTime(uint16_t micro_sec) const{
    if((micro_sec < 2) || (micro_sec > 508)){
        return false;
    }

    uint8_t motor_data = static_cast<uint8_t>(micro_sec >> 1);

    /* Write data to motor. */
    uint8_t args[2] = {REG_RETURN_DELAY_TIME, motor_data};
    return dataWriter(args, sizeof(args));
}

bool Motor::setCwAngleLimit(float min_angle) const{
    if((min_angle < MIN_ANGLE) || (min_angle > MAX_ANGLE)){
        return false;
    }

    uint16_t normalized_value = static_cast<uint16_t>(
        min_angle / MAX_ANGLE * m_res_divider
    );

    uint8_t lowByte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t highByte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_CW_ANGLE_LIMIT, lowByte, highByte};
    return dataWriter(args, sizeof(args));
}

bool Motor::setCcwAngleLimit(float max_angle) const{
    if((max_angle < MIN_ANGLE) || (max_angle > MAX_ANGLE)){
        return false;
    }

    uint16_t normalized_value = static_cast<uint16_t>(
            max_angle / MAX_ANGLE * m_res_divider
    );

    uint8_t lowByte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t highByte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_CCW_ANGLE_LIMIT, lowByte, highByte};
    return dataWriter(args, sizeof(args));
}

bool Motor::setVoltageLimit(VoltageLimit limit, float voltage) const{
    if((voltage < MIN_VOLTAGE) || (voltage > MAX_VOLTAGE)){
        return false;
    }

    uint8_t args[2];
    switch(limit){
        case VoltageLimit::HIGHEST:
            args[0] = REG_HIGH_VOLTAGE_LIMIT;
            break;
        case VoltageLimit::LOWEST:
            args[0] = REG_LOW_VOLTAGE_LIMIT;
            break;
        default:
            return false;
    }

    args[1] = static_cast<uint8_t>(voltage * 10);

    // Write data to motor
    return dataWriter(args, sizeof(args));
}

bool Motor::setMaxTorque(float max_torque) const{
    if((max_torque < MIN_TORQUE) || (max_torque > MAX_TORQUE)){
        return false;
    }

    // Translate the input from percentage into a 10-bit number
    uint16_t normalized_value = static_cast<uint16_t>(
        max_torque / 100 * 1023
    );

    uint8_t low_byte = normalized_value & 0xFF;
    uint8_t high_byte = (normalized_value >> 8) & 0xFF;

    // Write data to motor
    uint8_t args[3] = {REG_MAX_TORQUE, low_byte, high_byte};
    return dataWriter(args, sizeof(args));
}

bool Motor::setStatusReturnLevel(StatusReturnLevel level) const{
    if(level >= StatusReturnLevel::NUM_LEVELS){
        return false;
    }

    // Write data to motor
    uint8_t args[2] = {REG_STATUS_RETURN_LEVEL, static_cast<uint8_t>(level)};
    return dataWriter(args, sizeof(args));
}

bool Motor::setAlarm(AlarmType type, AlarmCondition condition) const{
    if((type >= AlarmType::NUM_TYPES) ||
       (condition >= AlarmCondition::NUM_CONDITIONS))
    {
        return false;
    }

    uint8_t args[2];
    switch(type){
        case AlarmType::LED:
            args[0] = REG_ALARM_LED;
            break;
        case AlarmType::SHUTDOWN:
            args[0] = REG_ALARM_SHUTDOWN;
            break;
        default:
            return false;
    }

    // Write data to motor
    args[1] = static_cast<uint8_t>(condition);
    return dataWriter(args, sizeof(args));
}

bool Motor::enableTorque(bool is_enabled) const{
    // Write data to motor
    uint8_t args[2] = {REG_TORQUE_ENABLE, static_cast<uint8_t>(is_enabled)};
    return dataWriter(args, sizeof(args));
}

bool Motor::enableLed(bool is_enabled) const{
    // Write data to motor
    uint8_t args[2] = {REG_LED_ENABLE, static_cast<uint8_t>(is_enabled)};
    return dataWriter(args, sizeof(args));
}

bool Motor::setGoalPosition(float goal_angle) const{
    if((goal_angle < MIN_ANGLE) || (goal_angle > MAX_ANGLE)){
        return false;
    }

    // Translate the angle from degrees into a binary code with the resolution
    // selected at construction
    uint16_t normalized_value = static_cast<uint16_t>(
        (goal_angle / MAX_ANGLE) * m_res_divider
    );

    uint8_t low_byte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t high_byte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_GOAL_POSITION, low_byte, high_byte};
    return dataWriter(args, sizeof(args));
}

bool Motor::setGoalTorque(float goal_torque) const{
    if((goal_torque < 0) || (goal_torque > 100.0)){
        return false;
    }

    // Translate the input from percentage into and 10-bit number
    uint16_t normalized_value = static_cast<uint16_t>(
        goal_torque / 100 * 1023
    );

    uint8_t low_byte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t high_byte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_GOAL_TORQUE, low_byte, high_byte};
    return dataWriter(args, sizeof(args));
}

bool Motor::lockEEPROM() const{
    // Write data to motor
    uint8_t args[2] = {REG_LOCK_EEPROM, 1};
    return dataWriter(args, sizeof(args));
}

bool Motor::setPunch(float punch) const{
    if((punch < 0) || (punch > 100.0)){
        return false;
    }

    // Translate the punch into a 10-bit number
    uint16_t normalized_value = static_cast<uint16_t>(punch / 100.0 * 1023);

    uint8_t low_byte = static_cast<uint8_t>(normalized_value & 0xFF);
    uint8_t high_byte = static_cast<uint8_t>((normalized_value >> 8) & 0xFF);

    // Write data to motor
    uint8_t args[3] = {REG_PUNCH, low_byte, high_byte};
    return dataWriter(args, sizeof(args));
}

bool Motor::getPosition(float& position_out) const{
    // Read data from motor
    uint16_t raw = 0;
    bool success = dataReader(REG_CURRENT_POSITION, 2, raw);

    // Parse data and write it into R-val
    if(success){
        position_out = static_cast<float>(raw * MAX_ANGLE / m_res_divider);
    }

    return success;
}

bool Motor::getLoad(float& load_out) const{
    // Read data from motor
    uint16_t raw = 0;
    bool success = dataReader(REG_CURRENT_LOAD, 2, raw);

    // Parse data and write it into R-val
    if(success){
        bool is_negative = static_cast<bool>((raw >> 9) & 0x1);
        if(raw > 1023){
            raw = raw - 1023;
        }

        float retVal = static_cast<float>(raw / 1023.0 * 100.0);
        if(is_negative){
            retVal *= -1;
        }
    }

    return success;
}

bool Motor::getVoltage(float& voltage_out) const{
    // Read data from motor
    uint16_t raw = 0;
    bool success = dataReader(REG_CURRENT_VOLTAGE, 1, raw);

    // Parse data and write it into R-val
    if(success){
        voltage_out = static_cast<float>(raw / 10.0);
    }

    return success;
}

bool Motor::getTemperature(uint8_t& temp_out) const{
    uint16_t raw = 0;
    bool success = dataReader(REG_CURRENT_TEMPERATURE, 1, raw);

    if(success){
        temp_out = static_cast<uint8_t>(raw);
    }

    return success;
}

bool Motor::isJointMode(bool& is_joint_mode_out){
    // Read data from motor
    uint16_t ret_val_cw;
    bool success = dataReader(REG_CW_ANGLE_LIMIT, 2, ret_val_cw);

    if(!success){
        return false;
    }

    uint16_t retValCCW;
    success = dataReader(REG_CCW_ANGLE_LIMIT, 2, retValCCW);

    if(!success){
        return false;
    }

    is_joint_mode_out = (ret_val_cw == retValCCW);
    m_is_joint_mode = is_joint_mode_out;

    return success;
}

bool Motor::ping(uint8_t& id_out) const{
    uint8_t arr_transmit[6];

    arr_transmit[0] = 0xff;
    arr_transmit[1] = 0xff;
    arr_transmit[2] = m_id;
    arr_transmit[3] = 2;
    arr_transmit[4] = INST_PING;
    arr_transmit[5] = computeChecksum(arr_transmit, 6);

    // Transmit read request
    bool success = m_daisy_chain->requestTransmission(
        arr_transmit,
        sizeof(arr_transmit)
    );

    if(!success){
        return false;
    }

    // Receive requested data
    success = m_daisy_chain->requestReception(
        arr_transmit,
        sizeof(arr_transmit)
    );

    if(success){
        id_out = arr_transmit[2];
    }

    return success;
}

bool Motor::isMoving(bool& is_moving_out) const{
    uint16_t raw = 0;
    bool success = dataReader(REG_MOVING, 1, raw);

    if(success){
        is_moving_out = static_cast<bool>(raw & 1);
    }

    return success;
}

bool Motor::enterWheelMode(){
    bool success = setCwAngleLimit(0);
    if(!success){
        return false;
    }

    success = setCcwAngleLimit(0);
    if(!success){
        return false;
    }

    m_is_joint_mode = 0;

    return success;
}

bool Motor::enterJointMode(){
    bool success = setCwAngleLimit(MIN_ANGLE);
    if(!success){
        return false;
    }

    success = setCcwAngleLimit(MAX_ANGLE);
    if(!success){
        return false;
    }

    m_is_joint_mode = true;

    return success;
}




// Protected
// ----------------------------------------------------------------------------
bool Motor::dataWriter(
    uint8_t* args,
    size_t num_args
)  const
{
    // Check validity so that we don't accidentally make a read request that is
    // invalid or we cannot support due to our implementation.
    // We cannot dynamically allocate an array to hold all the data to be
    // transmitted, so 3 args was chosen as the cutoff since our use cases
    // don't require more than that
    if(num_args > 3){
        return false;
    }

    uint8_t arr_transmit[9];

    arr_transmit[0] = 0xFF;
    arr_transmit[1] = 0xFF;
    arr_transmit[2] = m_id;
    arr_transmit[3] = 2 + num_args;
    arr_transmit[4] = INST_WRITE_DATA;

    for(uint8_t i = 0; i < num_args; i ++){
        arr_transmit[5 + i] = args[i];
    }

    // Checksum
    arr_transmit[4 + num_args + 1] = computeChecksum(
        arr_transmit,
        4 + num_args + 2
    );

    // Transmit
    return m_daisy_chain->requestTransmission(arr_transmit, 4 + num_args + 2);
}

bool Motor::dataReader(
    uint8_t read_addr,
    uint8_t read_length,
    uint16_t& ret_val_out
) const
{
    // Check validity so that we don't accidentally make a read request that is
    // invalid or we cannot support due to our implementation.
    // Since we cannot dynamically allocate an array to hold all the returned
    // data, we chose 2 as the cutoff. This supports our use cases just fine,
    // but in the future it could be tweaked if need be
    if(read_length > 2){
        return false;
    }

    uint8_t arr[8];

    arr[0] = 0xFF;
    arr[1] = 0xFF;
    arr[2] = m_id;
    arr[3] = 4;
    arr[4] = INST_READ_DATA;
    arr[5] = read_addr;
    arr[6] = read_length;
    arr[7] = computeChecksum(arr, 8);

    // Transmit read request
    if(!m_daisy_chain->requestTransmission(arr, 8)){
        return false;
    }

    // Receive requested data
    uint8_t rx_packet_size = (read_length == 1) ? 7 : 8;
    if(!m_daisy_chain->requestReception(arr, rx_packet_size)){
        return false;
    }

    // Check data integrity before passing data to application
    uint8_t recv_checksum = arr[rx_packet_size - 1];
    uint8_t computed_checksum = computeChecksum(arr, rx_packet_size);
    bool success = (computed_checksum == recv_checksum);

    if(success){
        ret_val_out = static_cast<uint16_t>(arr[5]);
        if(read_length == 2){
            ret_val_out |= (arr[6] << 8);
        }
    }

    return success;
}




// Private
// ----------------------------------------------------------------------------


}




/**
 * @}
 */
/* end - Dynamixel */
