/*
 * dynamixel_p2.c
 *
 *  Created on: Jun 9, 2023
 *      Author: nam
 */

#include "dynamixel_p2.h"
//#include "main.h"


/*
 * Dynamixel 2.0
 */

void write_min_position_limit_p2(MotorPort *port, uint8_t id, uint32_t limit) {
  uint8_t data[4] = {limit & 0xff, (limit>>8) & 0xff, 0, 0};
  uint16_t dataLen = 4;
  uint16_t addr = 48;
  _motor_write_p2(port, id, addr, data, dataLen);
}

void write_max_position_limit_p2(MotorPort *port, uint8_t id, uint32_t limit) {
  uint8_t data[4] = {limit & 0xff, (limit>>8) & 0xff, 0, 0};
  uint16_t dataLen = 4;
  uint16_t addr = 52;
  _motor_write_p2(port, id, addr, data, dataLen);
}
/*
 * Dynamixel 2.0
 */
//void update_cw_angle_limit_p2(MotorPort *p, uint8_t id, uint8_t val) {
//  uint8_t data[1] = {val};
//  uint16_t dataLen = ;
//  uint16_t addr = ;
//  _motor_write_p2(p, id, addr, data, dataLen);
//}
//
///*
// * Dynamixel 2.0
// */
//void update_ccw_angle_limit_p2(MotorPort *p, uint8_t id, uint8_t val) {
//  uint8_t data[1] = {val};
//  uint16_t dataLen = ;
//  uint16_t addr = ;
//  _motor_write_p2(p, id, addr, data, dataLen);
//}

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
 * Dynamixel 2.0
 */
void update_motor_id_p2(MotorPort *p, uint8_t id, uint8_t new_id) {
  uint8_t data[1] = {new_id};
  uint16_t dataLen = 1;
  uint16_t addr = 7;
  _motor_write_p2(p, id, addr, data, dataLen);
}

void update_baud_rate_p2(MotorPort *p, uint8_t id, uint8_t rate) {
  uint8_t data[1] = {rate};
  uint16_t dataLen = 1;
  uint16_t addr = 8;
  _motor_write_p2(p, id, addr, data, dataLen);
}

void read_motor_id_p2(MotorPort * p) {
  uint16_t dataLen = 1;
  uint16_t addr = 7;
  _motor_read_p2(p, 0x2, addr, dataLen);
}

void read_motor_present_position_p2(MotorPort * p, uint8_t id) {
  uint16_t dataLen = 4;
  uint16_t addr = 132;
  _motor_read_p2(p, id, addr, dataLen);
}

void write_goal_position_p2(MotorPort *port, uint8_t id, uint16_t angle) {
  uint8_t data[4] = {angle & 0xff, (angle>>8) & 0xff, 0, 0};
  uint16_t dataLen = 4;
  uint16_t addr = 116;
  _motor_write_p2(port, id, addr, data, dataLen);
}


/*
 * Write to specified address. Will be used by most functions to interact with motors
 * https://emanual.robotis.com/docs/en/dxl/protocol2/
 */
void _motor_write_p2(MotorPort *p, uint8_t id, uint16_t addr, uint8_t* data, uint16_t dataLen) {
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

  uint16_t crc = _update_crc(0, txBuf, 10+dataLen);
  txBuf[packetLen-2] = crc & 0xff;
  txBuf[packetLen-1] = (crc >> 8) & 0xff;

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_WRITE);

  HAL_UART_Transmit(p->huart, txBuf, packetLen, 1000);

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_READ);
}




/*
 * Write to specified address. Will be used by most functions to interact with motors
 * https://emanual.robotis.com/docs/en/dxl/protocol2/
 */
void _motor_read_p2(MotorPort *p, uint8_t id, uint16_t addr, uint16_t dataLen) {
  // setup message packet
  uint8_t packetLen = 14;
  uint8_t txBuf[50]; //set size to something much bigger than we will need
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0xFD;
  txBuf[3] = 0x00;
  txBuf[4] = id; // Packet ID (don't use 0xFE = broadcast, doesn't work for read instruction)

  uint8_t length = 7;
  txBuf[5] = length & 0xff; // length
  txBuf[6] = (length >> 8) & 0xff;

  txBuf[7] = 0x02; // read instruction

  txBuf[8] = addr & 0xff;   // address
  txBuf[9] = (addr>>8) & 0xff;

  txBuf[10] = dataLen & 0xff; // data length
  txBuf[11] = (dataLen >> 8) & 0xff;

  uint16_t crc = _update_crc(0, txBuf, 12);
  txBuf[12] = crc & 0xff;
  txBuf[13] = (crc >> 8) & 0xff;

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_WRITE);

  HAL_UART_Transmit(p->huart, txBuf, packetLen, 1000);

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_READ);

//  p->rxPacketLen = 11 + dataLen;
//  HAL_UART_Receive_DMA(p->huart, p->rxBuffer, p->rxPacketLen);
//
//  uint32_t tLast = HAL_GetTick();
//  uint32_t TIMEOUT = 10; // milliseconds
//  while(p->dmaDoneReading == false && HAL_GetTick() - tLast < TIMEOUT){
//  }
//  crc = _update_crc(0, p->rxBuffer, 13);
//  p->dmaDoneReading = false;
}

void sync_read_motor_present_position_p2(MotorPort * p) {
  uint16_t dataLen = 4;
  uint16_t addr = 132;
  _motor_sync_read_p2(p, addr, dataLen);
}

void _motor_sync_read_p2(MotorPort *p, uint16_t addr, uint16_t dataLen) {
  // setup message packet
  uint8_t packetLen = 8 + 4 + p->numMotors + 2;
  uint8_t txBuf[50]; //set size to something much bigger than we will need
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0xFD;
  txBuf[3] = 0x00;
  txBuf[4] = 0xFE; // Packet ID (don't use 0xFE = broadcast, doesn't work for read instruction)

  uint8_t length = 3 + 4 + p->numMotors;
  txBuf[5] = length & 0xff; // length
  txBuf[6] = (length >> 8) & 0xff;

  txBuf[7] = 0x82; // read instruction

  txBuf[8] = addr & 0xff;   // address
  txBuf[9] = (addr>>8) & 0xff;

  txBuf[10] = dataLen & 0xff; // data length
  txBuf[11] = (dataLen >> 8) & 0xff;

  for (uint8_t i = 0; i<p->numMotors; i++)
      txBuf[12+i] = p->motorIds[i];

  uint16_t crc = _update_crc(0, txBuf, 12 + p->numMotors);
  txBuf[12 + p->numMotors] = crc & 0xff;
  txBuf[13 + p->numMotors] = (crc >> 8) & 0xff;

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_WRITE);

  HAL_UART_Transmit(p->huart, txBuf, packetLen, 1000);

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_READ);

//  p->rxPacketLen = 11 + dataLen;
//  HAL_UART_Receive_DMA(p->huart, p->rxBuffer, p->rxPacketLen);
//
//  uint32_t tLast = HAL_GetTick();
//  uint32_t TIMEOUT = 10; // milliseconds
//  while(p->dmaDoneReading == false && HAL_GetTick() - tLast < TIMEOUT){
//  }
//  crc = _update_crc(0, p->rxBuffer, 13);
//  p->dmaDoneReading = false;
}

void sync_write_goal_position_p2(MotorPort *port) {

  uint16_t dataLen = 4;
  uint16_t addr = 116;
  _motor_sync_write_p2(port, addr, dataLen);
}


/*
 * Write to specified address. Will be used by most functions to interact with motors
 * https://emanual.robotis.com/docs/en/dxl/protocol2/
 */
void _motor_sync_write_p2(MotorPort *p, uint16_t addr, uint16_t dataLen) {
  // setup message packet
  uint8_t packetLen = 8 + 4 + p->numMotors*(1+dataLen) + 2 ;
  uint8_t txBuf[100]; //set size to something much bigger than we will need
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0xFD;
  txBuf[3] = 0x00;
  txBuf[4] = 0xFE; // Packet ID (0xFE = broadcast)

  uint8_t length = 3 + 4 + p->numMotors*(1+dataLen);
  txBuf[5] = length & 0xff; // length
  txBuf[6] = (length >> 8) & 0xff;

  txBuf[7] = 0x83; // write instruction

  txBuf[8] = addr & 0xff;   // address
  txBuf[9] = (addr>>8) & 0xff;

  txBuf[10] = dataLen & 0xff; // data length
  txBuf[11] = (dataLen >> 8) & 0xff;

  //uint8_t data[4] = {angle & 0xff, (angle>>8) & 0xff, 0, 0};
  for (uint8_t i = 0; i< p->numMotors; i++) {
        txBuf[12+i*(1+dataLen)] = p->motorIds[i];
  	    txBuf[13+i*(1+dataLen)] = p->angles[i]& 0xff;
  	    txBuf[14+i*(1+dataLen)] = (p->angles[i]>>8) & 0xff;
  	    txBuf[15+i*(1+dataLen)] = 0;
  	    txBuf[16+i*(1+dataLen)] = 0;
  }
//
//  for (uint8_t i = 0; i<dataLen; i++)
//    txBuf[10+i] = data[i];

  uint16_t crc = _update_crc(0, txBuf, packetLen - 2);
  txBuf[packetLen-2] = crc & 0xff;
  txBuf[packetLen-1] = (crc >> 8) & 0xff;

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_WRITE);

  HAL_UART_Transmit(p->huart, txBuf, packetLen, 1000);

  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, BUFFER_READ);
}


void wait_for_dma_read_p2(MotorPort *p) {

}

/*
 * Read status after a Write command has been issued
 * https://emanual.robotis.com/docs/en/dxl/protocol2/
 */
void _motor_get_status_p2(MotorPort *p, uint16_t packetLen) {
  // set write direction
  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, GPIO_PIN_RESET);

  // Write to Motor on UART
  HAL_UART_Receive_DMA(p->huart, p->rxBuffer, packetLen);
}

unsigned short _update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
