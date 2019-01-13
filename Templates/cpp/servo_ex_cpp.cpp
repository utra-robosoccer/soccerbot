/**
 * @file
 * @author John Doe
 *
 * @ingroup Servo
 */

/********************************* Includes **********************************/
#include "servo_ex_cpp.h"
#include "uart.h"

namespace{
// Constants
// ----------------------------------------------------------------------------
/** @brief Servo's goal position register address */
const uint8_t REG_GOAL_POS = 0x24;

/** @brief Servo's current position register address */
const uint8_t REG_CURRENT_POS = 0x26;

/** @brief Number of angles the motor can distinguish */
const uint16_t POS_RESOLUTION = 256;

} // end anonymous namespace

namespace peripherals{
// Constants
// ----------------------------------------------------------------------------
const float MAX_POS = 300.0;
const float MIN_POS = 0.0;

// Public functions
// ----------------------------------------------------------------------------
bool servo::set_position(float angle){
    if(angle < MIN_POS || angle > MAX_POS){
        return false;
    }

    uint16_t raw = static_cast<uint16_t>(
        (angle / MAX_POS) * (POS_RESOLUTION - 1)
    );

    uart::send_byte(m_id);
    uart::send_byte(REG_GOAL_POS);
    uart::send_byte(raw & 0xFF);
    uart::send_byte((raw >> 8) & 0xFF);
}

void servo::read_position(){
    uart::send_byte(m_id);
    uart::send_byte(REG_CURRENT_POS);
    uint16_t raw = uart::recv_byte();
    uint16_t raw |= uart::recv_byte() << 8;

    last_position = (raw / (POS_RESOLUTION - 1)) * MAX_POS;
}

} // end namespace peripherals
