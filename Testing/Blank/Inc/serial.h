/*
 * serial.h
 *
 *  Created on: Dec 25, 2017
 *      Author: vuwij
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <robotState.h>
#include <robotGoal.h>
#include "stm32f4xx_hal.h"

// Sends a message to the computer
void send_state(RobotState* robotstate);

RobotGoal receive_state();

// The UART handler for serial messages
extern UART_HandleTypeDef huart5;

#define START_PATTERN 0xAA

#endif /* SERIAL_H_ */
