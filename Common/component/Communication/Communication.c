/**
  ******************************************************************************
  * @file    Communication.c
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




/******************************* Public Variables *****************************/
/**
 * This is the container for the goal state of the robot. Each control cycle
 * begins by refreshing this data structure (by receiving a new goal from the
 * control systems running on the PC) and then sending commands to actuators
 * stop reduce the error between the current state and the goal state
 */
RobotGoal robotGoal = {0};

/**
 * This is the container for the current state of the robot. Each control cycle
 * ends once this data structure is populated with fresh sensor data, at which
 * time this container is serialized and sent back to the PC for feedback
 * control
 */
RobotState robotState = {0};

/**
 * @}
 */
/* end Communication */
