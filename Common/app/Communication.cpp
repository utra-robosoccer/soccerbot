/**
  ******************************************************************************
  * @file
  * @author  Jason
  * @author  Tyler
  * @brief   Top-level communcation module
  *
  * @defgroup Communication Communication
  * @brief    Everything related to communication data structures
  * @{
  ******************************************************************************
  */




/********************************** Includes **********************************/
#include "Communication.h"

namespace{
// Variables
// ----------------------------------------------------------------------------
/**
 * This is the container for the goal state of the robot. Each control cycle
 * begins by refreshing this data structure (by receiving a new goal from the
 * control systems running on the PC) and then sending commands to actuators
 * stop reduce the error between the current state and the goal state
 */
soccerbot::comm::RobotGoal_t robot_goal = {0};

/**
 * This is the container for the current state of the robot. Each control cycle
 * ends once this data structure is populated with fresh sensor data, at which
 * time this container is serialized and sent back to the PC for feedback
 * control
 */
soccerbot::comm::RobotState_t robot_state = {0};

} // end anonymous namespace

namespace soccerbot{
namespace comm{
// Public Functions
// ----------------------------------------------------------------------------
RobotGoal_t& getRobotGoal(void){
    return robot_goal;
}

RobotState_t& getRobotState(void){
    return robot_state;
}

} // end namespace comm
} // end namespace soccerbot

/**
 * @}
 */
/* end Communication */
