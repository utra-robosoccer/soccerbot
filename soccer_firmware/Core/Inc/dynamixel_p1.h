/*
 * dynamixel_v1.h
 */

#ifndef INC_DYNAMIXEL_P1_H_
#define INC_DYNAMIXEL_P1_H_

#include "main.h"

void write_goal_position_p1(MotorPort *port, uint8_t id, uint16_t angle);
void read_present_position_p1(MotorPort *port, uint8_t id, uint16_t angle);

// functions meant for internal use
void _motor_write_p1(MotorPort *p, uint8_t id, uint8_t addr, uint8_t* data, uint8_t dataLen);
void _motor_read_p1(MotorPort *p, uint8_t id, uint8_t addr, uint8_t* data, uint8_t dataLen);
void _motor_ping_p1(MotorPort *port, uint8_t id);

#endif /* INC_DYNAMIXEL_P1_H_ */
