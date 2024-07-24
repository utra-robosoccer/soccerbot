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
void update_motor_id_p2(MotorPort *p, uint8_t id, uint8_t new_id);
void update_baud_rate_p2(MotorPort *p, uint8_t id, uint8_t rate);
void motor_torque_en_p2(MotorPort *p, uint8_t id, uint8_t val);

void read_motor_id_p2(MotorPort * p);
void read_motor_present_position_p2(MotorPort * p, uint8_t id);
void write_min_position_limit_p2(MotorPort *port, uint8_t id, uint32_t limit);
void write_max_position_limit_p2(MotorPort *port, uint8_t id, uint32_t limit);

void _motor_write_p2(MotorPort *p, uint8_t id, uint16_t addr, uint8_t* data, uint16_t dataLen);
void _motor_read_p2(MotorPort *p, uint8_t id, uint16_t addr, uint16_t dataLen);
void _motor_get_status_p2(MotorPort *p, uint16_t packetLen);
unsigned short _update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);


#endif /* INC_DYNAMIXEL_P2_H_ */
