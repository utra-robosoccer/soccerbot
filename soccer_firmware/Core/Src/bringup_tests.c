/*
 * bringup_tests.c
 */

#include "main.h"
#include "bringup_tests.h"
#include "dynamixel_p1.h"
#include "dynamixel_p2.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

void test_usb_tx() {
	uint8_t txBuf[8];
	uint8_t count = 1;

	while (1){
//		sprintf(txBuf, "%u\r\n", count);
		count++;
		txBuf[0] = count;
		if (count > 100) {
			count = 1;
		}

		CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));

		HAL_Delay(100);
	}
}

void test_usb_rx_tx() {
//	char rxBuf[8];

	while (1){

//		CDC_Transmit_FS((uint8_t *) txBuf, strlen(txBuf));

//		HAL_Delay(100);
	}
}

void id_motor_and_blink_led(uint8_t id) {
  update_motor_id(&port1, id);
  HAL_Delay(1000);

  for(uint8_t i = 0; i < 5; i++) {
    update_motor_led_p1(&port1, id, 1);
    HAL_Delay(100);
    update_motor_led_p1(&port1, id, 0);
    HAL_Delay(100);
  }
}

void dynamixel_test() {
  for (uint8_t i = 0; i < 1; i++) {
    update_motor_led_p1(motorPorts[i], 1, 0);
    HAL_Delay(100);
    update_motor_led_p1(motorPorts[i], 1, 1);
    HAL_Delay(100);
    test_motor_sweep1(motorPorts[i], 1);
    HAL_Delay(100);
    _motor_ping_p1(motorPorts[i], 1);
    HAL_Delay(2000);
  }
}

/*
 * Dynamixel 2.0
 * set baud rate to 57600bps
 */
void test_led_p2(MotorPort *port) {
  for(uint16_t i = 0; i < 6; i++)
  {
      update_motor_led_p2(port, 0x1, i%2);
      HAL_Delay(100);
  }
}

/*
 * Dynamixel 1.0
 * 0xFF   0xFF  0x01  0x04  0x02  0x2B  0x01  0xCC
 */
void read_motor_position(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h, uint8_t id, uint8_t* rxBuf, uint8_t len) {
  // setup message packet
  uint8_t txBuf[8];
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = id; // Packet ID (0xFE = broadcast)
  txBuf[3] = 0x04; // length
  txBuf[4] = 0x02; // write instruction
  txBuf[5] = 36;   // address
  txBuf[6] = 2; // register length
  txBuf[7] = ~(txBuf[2] + txBuf[3] + txBuf[4] + txBuf[5] + txBuf[6]); // checksum

  // set write direction
  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(&h, txBuf, sizeof(txBuf), 1000);

  // set read direction
  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_RESET);

  uint8_t i = 0;
  while(i++ < 100){
    HAL_UART_Receive(&h, rxBuf, len, 10);
  }
}

/*
 * Dynamixel 1.0
 */
void test_motor_sweep1(MotorPort *port, uint8_t id) {
	uint16_t angle = 0;
	uint16_t count = 0;
	int16_t dir = 1;
	motor_torque_en_p2(&port1, 10, 1);
	motor_torque_en_p2(&port1, 11, 1);
	motor_torque_en_p2(&port1, 12, 1);
	while (1)
	{
		  if(dir) angle += 5;
		  else angle -= 5;
		  if(angle >= 0x3ff) {
		    count++;
		    dir = 0; // reverse direction
		    angle = 0x3fe;
		  }
		  if(count == 2) break; // end test
		  angle %= 0x3ff;

		  HAL_Delay(5);

		  write_goal_position_p1(&port2, 0x1, angle);
		  write_goal_position_p2(&port1, 10, angle);
	      HAL_Delay(10);
		  write_goal_position_p1(&port2, 0x2, angle);
		  write_goal_position_p2(&port1, 11, angle);
		  HAL_Delay(10);
		  write_goal_position_p1(&port2, 0x3, angle);
		  write_goal_position_p2(&port1, 12, angle);
//		  if(angle % 100 == 0) led = led ^ 1;
//		  update_motor_led_p2(&port4, 12, led);
		  HAL_Delay(10);
	}
}

/*
 * Dynamixel 2.0
 * set baud rate to 57600bps
 */
void test_motor_sweep2(MotorPort *port) {
	uint16_t angle = 0;
	uint16_t count = 0;
  int16_t dir = 1;


  motor_torque_en_p2(port, 0x1, 1);
  HAL_Delay(5);


	while(1)
	{
	  if(dir) angle += 5;
    else angle -= 5;
    if(angle >= 0xfff) {
      count++;
      dir = 0; // reverse direction
      angle = 0xffe;
    }
    if(count == 2) break; // end test
    angle %= 0xfff;

    write_goal_position_p2(port, 0x1, angle);

    HAL_Delay(3);
	}
}

/*
 * Dynamixel 2.0
 * set baud rate to 57600bps
 */
void test_ping2(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h) {
	for(uint16_t i = 0; i < 6; i++)
	{
		  // setup message packet
		  uint8_t txBuf[10];
		  txBuf[0] = 0xFF; // header
		  txBuf[1] = 0xFF;
		  txBuf[2] = 0xFD;
		  txBuf[3] = 0x00;
		  txBuf[4] = 10; // Packet ID (0xFE = broadcast)
		  txBuf[5] = 0x03; // length L
		  txBuf[6] = 0x00; // length H
		  txBuf[7] = 0x01; // ping instruction
		  uint16_t crc = _update_crc(0, txBuf, 8);
		  txBuf[8] = (crc & 0xFF); // crc L
		  txBuf[9] = (crc >> 8) & 0xFF; // crc H

		  uint8_t rxBuf[14];
		  for(uint8_t i = 0; i < sizeof(rxBuf); i++)rxBuf[i] = 0;

		  // set write direction
		  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_SET);

		  // Write to Motor on UART2
		  HAL_UART_Transmit(&h, txBuf, sizeof(txBuf), 1000);

		  // set read direction
		  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_RESET);


		  while(HAL_UART_Receive(&h, rxBuf, sizeof(rxBuf), 100) != HAL_OK){

		  }

		  HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
		  HAL_Delay(10);

//		  while(rxBuf[7] != 0x55) {
//			  // wrong value, stop
//		  }
//		  HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
//		  HAL_Delay(100);
	}
}
