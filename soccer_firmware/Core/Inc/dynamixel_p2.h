/*
 * dynamixel_p2.h
 *
 *  Created on: Jun 9, 2023
 *      Author: nam
 */

#ifndef INC_DYNAMIXEL_P2_H_
#define INC_DYNAMIXEL_P2_H_

#include "stm32f4xx_hal.h"
#include "main.h"

void write_goal_position_p2(MotorPort *p, uint8_t id, uint16_t angle);
void update_motor_led_p2(MotorPort *p, uint8_t id, uint8_t val);
void motor_torque_en_p2(MotorPort *p, uint8_t id, uint8_t val);
void _motor_write_p2(MotorPort *p, uint8_t id, uint16_t addr, uint8_t* data, uint8_t dataLen);
unsigned short _update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);


#endif /* INC_DYNAMIXEL_P2_H_ */
