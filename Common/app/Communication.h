/**
  ******************************************************************************
  * @file
  * @author  Jason
  * @author  Tyler
  * @brief   Header for top-level communication module
  *
  * @defgroup CommunicationHeader Communication (Header)
  * @brief    Header for communication, showing the public content
  * @ingroup  Communication
  * @{
  ******************************************************************************
  */




#ifndef COMMUNICATION_H
#define COMMUNICATION_H




/********************************** Includes **********************************/
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"


namespace soccerbot{
namespace comm{
// Constant Declarations
// ----------------------------------------------------------------------------
/** Buffer offset at which MPU6050 data is inserted */
constexpr uint8_t ROBOT_STATE_MPU_DATA_OFFSET = 48;


// Types & enums
// ----------------------------------------------------------------------------
/**
 * @brief Data structure sent from the PC to the MCU. Contains "goal" motor
 *        positions
 */
typedef struct {
    uint32_t id;  /**< Message ID */
    char msg[80]; /**< Raw message data as bytes sent over a serial terminal */
} RobotGoal_t;

/** @brief Data structure sent from the MCU to the PC. Contains sensor data */
typedef struct {
    uint32_t start_seq; /**< Start sequence to attach to message (for data
                             integrity purposes)                             */
    uint32_t id;        /**< Message ID */
    char msg[80];       /**< Raw message data as bytes sent over a serial
                             terminal                                        */
    uint32_t end_seq;   /**< End sequence to attach to message (for data
                             integrity purposes)                             */
} RobotState_t;


// Public Function Prototypes
// ----------------------------------------------------------------------------
RobotGoal_t& get_robot_goal(void);
RobotState_t& get_robot_state(void);

} // end namespace comm
} // end namespace soccerbot

/**
 * @}
 */
/* end CommunicationHeader */

#endif /* __COMMUNICATION_H__ */
