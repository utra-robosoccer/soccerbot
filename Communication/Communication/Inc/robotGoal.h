/*
 * RobotGoal.h
 *
 *  Created on: Dec 19, 2017
 *      Author: Admin
 */

#ifndef ROBOTGOAL_H_
#define ROBOTGOAL_H_

#include "main.h"
#include "stm32l4xx_hal.h"

typedef struct robot_goal{
	uint32_t id;
	char msg[12];
} RobotGoal;

#endif /* ROBOTGOAL_H_ */
