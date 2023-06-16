/*
 * dynamixel_p2.c
 *
 *  Created on: Jun 9, 2023
 *      Author: nam
 */

#include "dynamixel_p2.h"
#include "main.h"


/*
 * Dynamixel 2.0
 */
void write_goal_position_p2(MotorPort *port, uint8_t id, uint16_t angle) {
  uint8_t data[2] = {angle & 0xff, (angle>>8) & 0xff, 0, 0};
  uint16_t dataLen = 4;
  uint16_t addr = 116;
  _motor_write_p2(port, id, addr, data, dataLen);
  //TODO: handle status packet
}


/*
 * Dynamixel 2.0
 */
void motor_torque_en_p2(MotorPort *p, uint8_t id, uint8_t val) {
  uint8_t data[1] = {val};
  uint16_t dataLen = 1;
  uint16_t addr = 64;
  _motor_write_p2(p, id, addr, data, dataLen);
}

/*
 * Dynamixel 2.0
 */
void update_motor_led_p2(MotorPort *p, uint8_t id, uint8_t val) {
  uint8_t data[1] = {val};
  uint16_t dataLen = 1;
  uint16_t addr = 65;
  _motor_write_p2(p, id, addr, data, dataLen);
}

/*
 * Write to specified address. Will be used by most functions to interact with motors
 * https://emanual.robotis.com/docs/en/dxl/protocol1/
 */
void _motor_write_p2(MotorPort *p, uint8_t id, uint16_t addr, uint8_t* data, uint8_t dataLen) {
  // setup message packet
  uint8_t packetLen = 12 + dataLen;
  uint8_t txBuf[50]; //set size to something much bigger than we will need
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0xFD;
  txBuf[3] = 0x00;
  txBuf[4] = id; // Packet ID (0xFE = broadcast)

  uint8_t length = packetLen - 7;
  txBuf[5] = length & 0xff; // length
  txBuf[6] = (length >> 8) & 0xff;

  txBuf[7] = 0x03; // write instruction

  txBuf[8] = addr & 0xff;   // address
  txBuf[9] = (addr>>8) & 0xff;

  for (uint8_t i = 0; i<dataLen; i++)
    txBuf[10+i] = data[i];

  uint16_t crc = HAL_CRC_Calculate(&hcrc, txBuf, 10+dataLen);
  txBuf[packetLen-2] = crc & 0xff;
  txBuf[packetLen-1] = (crc >> 8) & 0xff;

  // set write direction
  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(p->huart, txBuf, packetLen, 1000);
}
