/*
 * update_loop.c
 */

#include "update_loop.h"
#include "dynamixel_p1.h"

void update() {
  while(1) {
//    if (usb_received) {
//      uint16_t angle = rxBuffer[2] | (rxBuffer[3] << 8);
//      update_motor_position(&port1, 0xfe, angle);

      uint8_t txBuf[2];
      _motor_read_p1(&port1, 0x01, 36, txBuf, 2);
      uint8_t result = CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));

      HAL_Delay(1000);
//      usb_received = 0;
//    }
  }
}
