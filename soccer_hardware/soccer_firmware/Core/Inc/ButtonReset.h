/*
 * ButtonReset.h
 *
 *  Created on: Jan 25, 2025
 *      Author: Jingling Hou
 */

#ifndef INC_BUTTONRESET_H_
#define INC_BUTTONRESET_H_

#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "dynamixel_p2.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

GPIO_PinState ReadButtonState(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void MotorTest1();
void MotorTest2();
void MotorTest3();
void MotorTest4(uint16_t rotv);
void set_Rotational_Speed(MotorPort* port, uint8_t motor_ID, uint16_t rotationalSpeed);
void set_Angle(MotorPort* port, uint8_t motorID, uint16_t defaultAngle);


#endif /* INC_BUTTONRESET_H_ */
