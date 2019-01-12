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

#ifndef SERVO_EX_CPP_H
#define SERVO_EX_CPP_H

/********************************* Includes **********************************/
#include <stdint.h>

namespace peripherals{
// Constant Declarations
// ----------------------------------------------------------------------------
extern const float MAX_POS;
extern const float MIN_POS;

// Classes and structs
// ----------------------------------------------------------------------------
/**
 * @brief Interface for communication with a smart servo
 */
class servo{
public:
	/**
	 * @brief Servo constructor
	 * @param id Unique identifier for motor, used to address it in commands
	 */
	servo(uint8_t id) : m_id(id) {}
	~servo(){}

    /**
     * @brief Moves the servo to the specified position
     * @param angle The desired angle in degrees (min: MIN_POS, max: MAX_POS)
     * @return true if successful, otherwise false
     */
	bool set_position(float angle);

	/**
	 * @brief Reads the current position from the servo
	 * @note The position read is stored in the `last_position` member
	 */
	void read_position();

	float last_position; /**< Last-read position; updated by `read_position` */

private:
	const uint8_t m_id; /**< Unique identifier for motor, used to address it */
}

} // end namespace peripherals

/**
 * @}
 */

#endif /* SERVO_EX_CPP_H */