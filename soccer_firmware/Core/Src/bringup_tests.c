/*
 * bringup_tests.c
 */

#include "main.h"
#include "bringup_tests.h"
#include "dynamixel_p1.h"
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

		CDC_Transmit_FS((uint8_t *) txBuf, strlen(txBuf));

		HAL_Delay(100);
	}
}

void test_usb_rx_tx() {
	char rxBuf[8];

	while (1){

//		CDC_Transmit_FS((uint8_t *) txBuf, strlen(txBuf));

//		HAL_Delay(100);
	}
}

void dynamixel_test() {
  for (uint8_t i = 0; i < 1; i++) {
    update_motor_led(motorPorts[i], 1, 0);
    HAL_Delay(100);
    update_motor_led(motorPorts[i], 1, 1);
    HAL_Delay(100);
    test_motor_sweep1(motorPorts[i], 1);
    HAL_Delay(100);
    motor_ping_p1(motorPorts[i], 1);
    HAL_Delay(2000);
  }
}


/*
 * Dynamixel 1.0
 */
void update_motor_id(MotorPort* p, uint8_t id) {
  // setup message packet
  uint8_t txBuf[8];
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0xFE; // Packet ID (0xFE = broadcast)
  txBuf[3] = 0x04; // length
  txBuf[4] = 0x03; // write instruction
  txBuf[5] = 3;   // address
  txBuf[6] = id; // value
  txBuf[7] = ~(txBuf[2] + txBuf[3] + txBuf[4] + txBuf[5] + txBuf[6]); // checksum

  // set write direction
  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(p->huart, txBuf, sizeof(txBuf), 1000);
}

/*
 * Dynamixel 1.0
 */
void update_motor_led(MotorPort *p, uint8_t id, uint8_t val) {
  uint8_t data[1] = {val};
  uint8_t dataLen = 1;
  uint8_t addr = 25;
  dynamixel_write_p1(p, id, addr, data, dataLen);
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

		  update_motor_position(port, id, angle);
	}
}

/*0xFF  0xFF  0x00  0x02  0x06  0xF7
 * Dynamixel 1.0
 */
void factory_reset(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h) {
  uint8_t txBuf[6];
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0x00; // Packet ID (0xFE = broadcast)
  txBuf[3] = 0x02; // length
  txBuf[4] = 0x06; // PING instruction
  txBuf[5] = 0xF7;// checksum

  // set write direction
  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(&h, txBuf, sizeof(txBuf), 1000);
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
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


/*
 * Dynamixel 2.0
 * set baud rate to 57600bps
 */
void test_motor_sweep2(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h) {
	uint16_t angle = 0;
	uint16_t count = 0;
	while(1)
	{
		  angle += 5;
		  count++;
		  if(count == 1000) break; // end test
		  angle %= 0xfff;

		  HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
		  HAL_Delay(10);

		  uint8_t txBuf[16];
      txBuf[0] = 0xFF; // header
      txBuf[1] = 0xFF; // header
      txBuf[2] = 0xFD; // header
      txBuf[3] = 0x00; // reserved
      txBuf[4] = 0x01; // Packet ID (0xFE = broadcast)
      txBuf[5] = 0x09; // length L
      txBuf[6] = 0x00; // length H
      txBuf[7] = 0x03; // write instruction
      txBuf[8] = 0x74; // address L
      txBuf[9] = 0x00; // address H
      txBuf[10] = 0xff; // P1
      txBuf[11] = 0xff; // P2
      txBuf[12] = 0xff; // P3
      txBuf[13] = 0xff; // P4
      uint16_t crc = update_crc(0, txBuf, 14);
      txBuf[14] = (crc & 0xFF); // crc
      txBuf[15] = (crc >> 8) & 0xFF;

		  // setup message packet
//		  uint8_t txBuf[16];
//		  txBuf[0] = 0xFF; // header
//		  txBuf[1] = 0xFF; // header
//		  txBuf[2] = 0xFD; // header
//		  txBuf[3] = 0x00; // reserved
//		  txBuf[4] = 0xFE; // Packet ID (0xFE = broadcast)
//		  txBuf[5] = 0x09; // length L
//		  txBuf[6] = 0x00; // length H
//		  txBuf[7] = 0x03; // write instruction
//		  txBuf[8] = 1Position Control Mode16; // address L
//		  txBuf[9] = 0x00; // address H
//		  txBuf[10] = 0x00; // P1
//		  txBuf[11] = 0x02; // P2
//		  txBuf[12] = 0x00; // P3
//		  txBuf[13] = 0x00; // P4
//		  uint16_t crc = update_crc(0, txBuf, 14);
//		  txBuf[14] = (crc & 0xFF); // crc
//		  txBuf[15] = (crc >> 8) & 0xFF;

		  // set write direction
		  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_SET);

		  // Write to Motor on UART
		  HAL_UART_Transmit(&h, txBuf, 16, 1000);

		  // set read direction
      HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_RESET);

      uint8_t rxBuf[11];
      uint8_t i = 0;
      while(i++ < 100){
        HAL_UART_Receive(&h, rxBuf, sizeof(rxBuf), 10);
      }

		  HAL_Delay(100);
	}
}

/*
 * Dynamixel 2.0
 * set baud rate to 57600bps
 */
void test_led2(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h) {
	for(uint16_t i = 0; i < 6; i++)
	{
		  // setup message packet
		  uint8_t txBuf[16];
		  txBuf[0] = 0xFF; // header
		  txBuf[1] = 0xFF;
		  txBuf[2] = 0xFD;
		  txBuf[3] = 0x00;
		  txBuf[4] = 0xFE; // Packet ID (0xFE = broadcast)
		  txBuf[5] = 0x06; // length L
		  txBuf[6] = 0x00; // length H
		  txBuf[7] = 0x03; // write instruction
		  txBuf[8] = 65; // address L
		  txBuf[9] = 0x00; // address H
		  txBuf[10] = i % 2; // P1
		  uint16_t crc = update_crc(0, txBuf, 11);
		  txBuf[11] = (crc & 0xFF); // crc L
		  txBuf[12] = (crc >> 8) & 0xFF; // crc H

		  // set write direction
		  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_SET);

		  // Write to Motor on UART
		  HAL_UART_Transmit(&h, txBuf, sizeof(txBuf), 1000);

		  HAL_Delay(100);
	}
}

/*
 * Dynamixel 2.0
 * set baud rate to 57600bps
 */
void write_motor2(GPIO_TypeDef *uart_port, uint16_t pin, UART_HandleTypeDef h, uint8_t* rxBuf, uint8_t bufLen, uint8_t addr) {
  uint8_t txBuf[14];
  txBuf[0] = 0xFF; // header
  txBuf[1] = 0xFF; // header
  txBuf[2] = 0xFD; // header
  txBuf[3] = 0x00; // reserved
  txBuf[4] = 0x01; // Packet ID (0xFE = broadcast)
  txBuf[5] = 0x07; // length L
  txBuf[6] = 0x00; // length H
  txBuf[7] = 0x02; // write instruction
  txBuf[8] = addr | 0xff; // address L
  txBuf[9] = (addr>>8) | 0xff ; // address H
  txBuf[10] = bufLen | 0xff; // P1
  txBuf[11] = (bufLen>>8) | 0xff; // P2
  uint16_t crc = update_crc(0, txBuf, 14);
  txBuf[12] = (crc & 0xFF); // crc
  txBuf[13] = (crc >> 8) & 0xFF;

  // set write direction
  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(&h, txBuf, 16, 1000);

  // set read direction
  HAL_GPIO_WritePin(uart_port, pin, GPIO_PIN_RESET);


  uint8_t i = 0;
  while(i++ < 100){
    HAL_UART_Receive(&h, rxBuf, sizeof(rxBuf), 10);
  }

  HAL_Delay(100);
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
		  txBuf[4] = 1; // Packet ID (0xFE = broadcast)
		  txBuf[5] = 0x03; // length L
		  txBuf[6] = 0x00; // length H
		  txBuf[7] = 0x01; // ping instruction
		  uint16_t crc = update_crc(0, txBuf, 8);
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

		  while(rxBuf[7] != 0x55) {
			  // wrong value, stop
		  }
		  HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
		  HAL_Delay(100);
	}
}
