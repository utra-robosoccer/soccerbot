/*
 * Communication.h
 *
 *  Created on: May 28, 2018
 *      Author: Admin
 */

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

/********************************** Includes **********************************/
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "robotState.h"
#include "robotGoal.h"


/******************************* Public Variables *******************************/
extern volatile RobotGoal robotGoal;
extern volatile RobotState robotState;
extern volatile uint8_t buf[10];


/***************************** Function prototypes ****************************/
void Comm_Init(volatile RobotGoal* robotGoal, volatile RobotState* robotState);

#endif /* COMMUNICATION_COMMUNICATION_H_ */
