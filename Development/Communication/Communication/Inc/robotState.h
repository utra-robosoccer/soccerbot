/*
 * RobotState.h
 *
 *  Created on: Dec 19, 2017
 *      Author: Admin
 */

#ifndef ROBOTSTATE_H_
#define ROBOTSTATE_H_

#include "main.h"
#include "stm32l4xx_hal.h"

typedef struct robot_state {
	uint32_t start_seq;
	uint32_t id;
	char msg[80];
	uint32_t end_seq;
} RobotState;

#endif /* ROBOTSTATE_H_ */
