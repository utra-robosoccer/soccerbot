/**
 * @file
 * @author John Doe
 *
 * @defgroup Servo
 * @brief Control and communication with smart servos. Provides interfaces for
 *        setting and getting data, and implements the low-level protocols.
 * @ingroup Peripherals
 * @{
 */

#ifndef SERVO_EX_C_H
#define SERVO_EX_C_H

/********************************* Includes **********************************/
#include <stdint.h>

/*************************** Constant Declarations ***************************/
extern const float MAX_POS;
extern const float MIN_POS;

/********************************** Types ************************************/
/**
 * @brief Interface for communication with a smart servo
 */
typedef struct{
	const uint8_t m_id; /**< Unique identifier for motor, used to address it */
	float last_position; /**< Last-read position */
}servo_t;

/************************ Public Function Prototypes *************************/
/**
 * @brief Moves the servo to the specified position
 * @param h_servo Handle for the target servo
 * @param angle The desired angle in degrees (min: MIN_POS, max: MAX_POS)
 * @return true if successful, otherwise false
 */
bool servo_set_position(servo_t* h_servo, float angle);

/**
 * @brief Reads the current position from the servo
 * @param h_servo Handle for the target servo
 * @note The position read is stored in the `last_position` member
 */
void servo_read_position(servo_t* h_servo);

/**
 * @}
 */

#endif /* SERVO_EX_C_H */