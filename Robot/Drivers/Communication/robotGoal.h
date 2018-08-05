/**
  ******************************************************************************
  * @file    robotGoal.h
  * @author  Jason
  * @brief   Defines the RobotGoal data structure used in communication with
  *          the high-level software. RobotGoal is the data structure sent from
  *          the PC to the MCU; it contains motor trajectories
  ******************************************************************************
  */

#ifndef ROBOTGOAL_H_
#define ROBOTGOAL_H_

/**
 * @brief Data structure sent from the PC to the MCU. Contains "goal" motor
 *        positions
 */
typedef struct robot_goal{
	uint32_t id;        /**< Message ID */
	char msg[80];       /**< Raw message data as bytes sent over a serial
	                         terminal                                        */
} RobotGoal;

#endif /* ROBOTGOAL_H_ */
