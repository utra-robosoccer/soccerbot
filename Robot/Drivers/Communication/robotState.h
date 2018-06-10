/*
 * RobotState.h
 *
 *  Created on: Dec 19, 2017
 *      Author: Admin
 */

#ifndef ROBOTSTATE_H_
#define ROBOTSTATE_H_

typedef struct robot_state {
	uint32_t start_seq;
	uint32_t id;
	char msg[80 + 24];
	uint32_t end_seq;
} RobotState;

#define ROBOT_STATE_MPU_DATA_OFFSET 80

#endif /* ROBOTSTATE_H_ */
