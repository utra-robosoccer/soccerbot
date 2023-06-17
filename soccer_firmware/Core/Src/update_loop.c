/*
 * update_loop.c
 */


#include "update_loop.h"

#include <stdbool.h>
#include "dynamixel_p1.h"
#include "dynamixel_p2.h"
#include "main.h"
#include "usbd_cdc_if.h"

void update() {
  while(1) {

    motor_torque_en_p2(motorPorts[1], 1, 1);

    if (usb_received) {
      HAL_Delay(10);

      for (uint16_t i = 0; i < 6; i++) {


        uint8_t idx = motorPorts[i]->currMotor;
        uint8_t motorId = motorPorts[i]->motorIds[idx];
        uint8_t protocol = motorPorts[i]->protocol[idx];


        if (idx >= motorPorts[i]->numMotors){
          continue;
        }

        uint16_t angle = usbRxBuffer[motorId * 2] | (usbRxBuffer[motorId * 2 + 1] << 8);

        if(protocol == 1) {
          write_goal_position_p1(motorPorts[i], motorId, angle);
        } else {
          write_goal_position_p2(motorPorts[i], motorId, angle);
        }
//        motorPorts[i]->currMotor = idx + 1;
      }


//      read motor angle
      uint8_t txBuf[2] = {0x11, 0x22};
//      _motor_read_p1(&port1, 0xfe, 36, txBuf, 2);
      CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));

      usb_received = false;
    }
  }
}
