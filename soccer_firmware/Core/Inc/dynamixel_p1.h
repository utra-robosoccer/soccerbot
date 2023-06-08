/*
 * dynamixel_v1.h
 *
 *  Created on: Jun 7, 2023
 *      Author: nam
 */

#ifndef INC_DYNAMIXEL_P1_H_
#define INC_DYNAMIXEL_P1_H_

#include "main.h"

void update_motor_position(MotorPort *port, uint8_t id, uint16_t angle);
void motor_ping_p1(MotorPort *port, uint8_t id);

#endif /* INC_DYNAMIXEL_P1_H_ */
