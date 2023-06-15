/*
 * update_loop.c
 */


#include "update_loop.h"

#include <stdbool.h>
#include "dynamixel_p1.h"
#include "main.h"
#include "usbd_cdc_if.h"

void update() {
  while(1) {
    if (usb_received) {
      uint16_t angle = usbRxBuffer[2] | (usbRxBuffer[3] << 8);




//      read motor angle
      uint8_t txBuf[2];
      _motor_read_p1(&port1, 0xfe, 36, txBuf, 2);
      CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));

      HAL_Delay(10);
      usb_received = false;
    }
  }
}
