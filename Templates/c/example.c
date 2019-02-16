/**
 * @file
 * @author John Doe
 *
 * @ingroup Servo
 * @{
 */

/********************************* Includes **********************************/
#include "servo_ex_c.h"
#include "uart.h"

/******************************** Constants **********************************/
/** @brief Servo's goal position register address */
static const uint8_t REG_GOAL_POS = 0x24;

/** @brief Servo's current position register address */
static const uint8_t REG_CURRENT_POS = 0x26;

/** @brief Number of angles the motor can distinguish */
static const uint16_t POS_RESOLUTION = 256;

const float MAX_POS = 300.0;
const float MIN_POS = 0.0;

/***************************** Public Functions ******************************/
bool servo_set_position(servo_t* h_servo, float angle){
    if(angle < MIN_POS || angle > MAX_POS){
        return false;
    }

    uint16_t raw = (uint16_t)((angle / MAX_POS) * (POS_RESOLUTION - 1));

    uart_send_byte(h_servo->m_id);
    uart_send_byte(REG_GOAL_POS);
    uart_send_byte(raw & 0xFF);
    uart_send_byte((raw >> 8) & 0xFF);
}

void servo_read_position(servo_t* h_servo){
    uart_send_byte(h_servo->m_id);
    uart_send_byte(REG_CURRENT_POS);
    uint16_t raw = uart_recv_byte();
    uint16_t raw |= uart_recv_byte() << 8;

    last_position = (raw / (POS_RESOLUTION - 1)) * MAX_POS;
}

/**
 * @}
 */
