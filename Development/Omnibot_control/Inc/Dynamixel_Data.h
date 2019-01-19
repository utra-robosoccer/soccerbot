/**
 * @file Dynamixel_Data.h
 * @author Tyler
 *
 * @defgroup Dynamixel_Data Data
 * @brief Shared data within library (constants, macros, etc.)
 * @ingroup DynamixelProtocolV1
 * @{
 */




#ifndef DYNAMIXEL_H_DYNAMIXEL_DATA_H_
#define DYNAMIXEL_H_DYNAMIXEL_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif




/********************************* Includes **********************************/
#include <stdint.h>




/********************************** Macros ***********************************/
/**
 * @brief Used to determine buffer sizes. Also used in application-level code
 */
#define NUM_MOTORS 18




/********************************* Constants *********************************/
// Default register values
extern const uint8_t BROADCAST_ID;
extern const uint8_t DEFAULT_ID;
extern const uint8_t DEFAULT_RETURN_DELAY;
extern const uint16_t DEFAULT_CW_ANGLE_LIMIT;
extern const uint8_t DEFAULT_LOW_VOLTAGE_LIMIT;
extern const uint16_t DEFAULT_MAXIMUM_TORQUE;
extern const uint8_t DEFAULT_STATUS_RETURN_LEVEL;
extern const uint8_t DEFAULT_ALARM_LED;
extern const uint8_t DEFAULT_ALARM_SHUTDOWN;
extern const uint8_t DEFAULT_TORQUE_ENABLE;
extern const uint8_t DEFAULT_LED_ENABLE;
extern const uint8_t DEFAULT_EEPROM_LOCK;

// Value limit definitions
extern const float MIN_VELOCITY;
extern const float MAX_ANGLE;
extern const float MIN_ANGLE;
extern const float MAX_TORQUE;
extern const float MIN_TORQUE;
extern const float MAX_VOLTAGE;
extern const float MIN_VOLTAGE;
extern const uint16_t MAX_PUNCH;
extern const uint16_t MIN_PUNCH;

/**
 * @}
 */
/* end - Dynamixel_Data */

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIXEL_H_DYNAMIXEL_DATA_H_ */
