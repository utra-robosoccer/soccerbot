/*
 * bringup_tests.h
 *
 *  Created on: May 26, 2023
 *      Author: lkune
 */

#ifndef INC_BRINGUP_TESTS_H_
#define INC_BRINGUP_TESTS_H_

#include "stm32f4xx_hal.h"

void test_usb_tx(void);
void test_all_ports1(void);
void update_motor_position(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h, uint8_t id, uint16_t angle);
void update_motor_id(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h, uint8_t id);
void update_motor_led(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h, uint8_t id, uint8_t val);
void test_motor_sweep1(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h);
void test_motor_sweep2(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h);
void test_ping1(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h);
void test_ping2(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h);

#endif /* INC_BRINGUP_TESTS_H_ */
