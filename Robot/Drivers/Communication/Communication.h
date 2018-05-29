/*
 * Communication.h
 *
 *  Created on: May 28, 2018
 *      Author: Admin
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

#include <stdint.h>
#include <string.h>
#include "usart.h"
#include "robotState.h"
#include "robotGoal.h"

extern volatile RobotGoal robotGoal;
extern volatile RobotState robotState;

void Comm_Init(volatile RobotGoal* robotGoal, volatile RobotState* robotState);

#endif /* COMMUNICATION_COMMUNICATION_H_ */
