/*
 * RobotGoal.h
 *
 *  Created on: Dec 19, 2017
 *      Author: Admin
 */

#ifndef ROBOTGOAL_H_
#define ROBOTGOAL_H_

#include "main.h"
#include "stm32f4xx_hal.h"

typedef struct robot_goal{
	uint32_t id;
	char msg[80];
} RobotGoal;

#endif /* ROBOTGOAL_H_ */
