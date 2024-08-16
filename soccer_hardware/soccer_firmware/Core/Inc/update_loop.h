/*
 * update_loop.h
 */

#ifndef INC_UPDATE_LOOP_H_
#define INC_UPDATE_LOOP_H_

#include "stm32f4xx_hal.h"

void update(void);
void command_motors(void);
void read_imu(uint8_t *rxBuf);
void read_motors(uint8_t *rxBuf);


#endif /* INC_UPDATE_LOOP_H_ */
