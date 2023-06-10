/*
 * dynamixel_p1.c
 *
 * protocol 1.0 functions to talk to AX/MX motors
 */

#include "dynamixel_p1.h"
#include "main.h"

/*
 * Dynamixel 1.0
 * https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
 */
void write_goal_position_p1(MotorPort *port, uint8_t id, uint16_t angle) {
  uint8_t data[2] = {angle & 0xff, (angle>>8) & 0xff};
  uint8_t dataLen = 2;
  uint8_t addr = 30;
  _motor_write_p1(port, id, addr, data, dataLen);
  //TODO: handle status packet
}

/*
 * trigger dma to read incoming data
 */
void dma_rx_read_p1() {
  // how does this work?
}

/*
 * when dma is done reading it will call this function
 */
void dma_rx_complete_p1() {
  // how does this work?
}

/*
 * Write to specified address. Will be used by most functions to interact with motors
 * https://emanual.robotis.com/docs/en/dxl/protocol1/
 */
void _motor_write_p1(MotorPort *p, uint8_t id, uint8_t addr, uint8_t* data, uint8_t dataLen) {
  // setup message packet
  uint8_t packetLen = 7 + dataLen;
  uint8_t txBuf[50]; //set size to something much bigger than we will need
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = id; // Packet ID (0xFE = broadcast)
  txBuf[3] = 3 + dataLen; // length
  txBuf[4] = 0x03; // write instruction
  txBuf[5] = addr;   // address

  for (uint8_t i = 0; i<dataLen; i++)
    txBuf[6+i] = data[i];

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < packetLen-1; i++)
    checksum += txBuf[i];
  txBuf[packetLen-1] = ~checksum;

  // set write direction
  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(p->huart, txBuf, packetLen, 1000);
}


/*
 * Read from specified address
 * https://emanual.robotis.com/docs/en/dxl/protocol1/
 */
void _motor_read_p1(MotorPort *p, uint8_t id, uint8_t addr, uint8_t* buf, uint8_t dataLen) {
  // setup message packet
  uint8_t txBuf[8]; //set size to something much bigger than we will need
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = id; // Packet ID (0xFE = broadcast)
  txBuf[3] = 0x04; // length
  txBuf[4] = 0x02; // write instruction
  txBuf[5] = addr;   // address
  txBuf[6] = dataLen;

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < 7; i++)
    checksum += txBuf[i];
  txBuf[7] = ~checksum;

  // set write direction
  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, GPIO_PIN_SET);

  // Write to Motor on UART
  HAL_UART_Transmit(p->huart, txBuf, sizeof(txBuf), 1000);

  // set read direction
  HAL_GPIO_WritePin(p->pinPort, p->dirPinNum, GPIO_PIN_RESET);

  uint8_t rxBuf[10];
  uint8_t packetLen = 6 + dataLen;
  HAL_UART_Receive_DMA(p->huart, rxBuf, packetLen);

  //TODO: add timeout
  while(p->dmaDoneReading == false){
  }
  p->dmaDoneReading = false;

  // TODO: check crc

  uint8_t err = rxBuf[4];
  for(uint8_t i = 0; i < dataLen; i++) {
    buf[i] = rxBuf[5+i];
  }
}


/*
 * Ping motor and get status packet
 * 0xFF  0xFF  0x00  0x02  0x06  0xF7
 * https://emanual.robotis.com/docs/en/dxl/protocol1/
 */
void _motor_ping_p1(MotorPort *port, uint8_t id) {
  uint8_t txBuf[6];
  txBuf[0] = 0xFF;
  txBuf[1] = 0xFF;
  txBuf[2] = 0xFE; // Packet ID (0xFE = broadcast)
  txBuf[3] = 0x02; // length
  txBuf[4] = 0x01; // PING instruction
  txBuf[5] = ~(txBuf[2] + txBuf[3] + txBuf[4]); // checksum

  uint8_t rxBuf[6];
  for(uint8_t i = 0; i < sizeof(rxBuf); i++)rxBuf[i] = 0;

  // set write direction
  HAL_GPIO_WritePin(port->pinPort, port->dirPinNum, GPIO_PIN_SET);

  // Write to Motor on UART2
  HAL_UART_Transmit(port->huart, txBuf, sizeof(txBuf), 1000);

  // set read direction
  HAL_GPIO_WritePin(port->pinPort, port->dirPinNum, GPIO_PIN_RESET);

  HAL_UART_Receive_DMA(port->huart, rxBuf, sizeof(rxBuf));

  while(port->dmaDoneReading == false){
  }
  port->dmaDoneReading = false;

  if(rxBuf[0] == 0xff && rxBuf[4] == 0) {
    for(uint8_t i = 0; i < 10; i++){
      HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
      HAL_Delay(100);
    }
  }
}

/*
 * callback gets called when dma is done reading from uart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == port1.huart->Instance) {
    port1.dmaDoneReading = true;
  }
}
