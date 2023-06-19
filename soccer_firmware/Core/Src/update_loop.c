/*
 * update_loop.c
 */


#include "update_loop.h"

#include <stdbool.h>
#include "dynamixel_p1.h"
#include "dynamixel_p2.h"
#include "main.h"
#include "usbd_cdc_if.h"

#define UPDATE_PERIOD 10 // milliseconds

void update()
{
  uint32_t lastTime = HAL_GetTick();
  while(1)
  {
    if (usb_received) // when we receive USB packet, service it right away
    {
      command_motors();
      usb_received = false;
    }

    if(HAL_GetTick() - lastTime > UPDATE_PERIOD) // send IMU/ANGLES periodically back to main computer
    {
      lastTime = HAL_GetTick();
      uint8_t txBuf[18 * 2 + 6] = {0}; // 18 motors * 2 bytes each + 6 bytes for IMU

      read_motors(txBuf);
      read_imu(txBuf);
      CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));

    }
  }
}

/*
 * Send commands in parallel on all ports to reduce latency
 * Keep sending angles until all motors have been commanded
 */
void command_motors() {
  for (uint16_t i = 0; i < 6; i++) {// reset variables
    motorPorts[i]->currMotor = 0;
  }

  while(1)
  {
    bool doneWithAllMotors = true;
    for (uint16_t i = 0; i < 6; i++)
    {
      uint8_t idx = motorPorts[i]->currMotor;
      uint8_t motorId = motorPorts[i]->motorIds[idx];
      uint8_t protocol = motorPorts[i]->protocol[idx];

      if (idx >= motorPorts[i]->numMotors){ // skip port if all motors already serviced
        continue;
      } else {
        doneWithAllMotors = false;
      }

      // angle format depends on how Python script
      uint16_t angle = usbRxBuffer[motorId * 2] | (usbRxBuffer[motorId * 2 + 1] << 8);

      if(protocol == 1) {
        write_goal_position_p1(motorPorts[i], motorId, angle);
      } else {
        write_goal_position_p2(motorPorts[i], motorId, angle);
      }

      motorPorts[i]->currMotor = idx + 1;
    }

    HAL_Delay(10); // delay enough for motors to have time to respond

    if(doneWithAllMotors) return; // all motors serviced, peace out
  }
}

void read_imu(uint8_t *rxBuf) {
  //TODO: fill this out
}


void read_motors(uint8_t *rxBuf) {
  for (uint16_t i = 0; i < 6; i++) {// reset variables
    motorPorts[i]->currMotor = 0;
    motorPorts[i]->dmaDoneReading = false;
    motorPorts[i]->readRequestSent = false;
  }

  while(1)
  {
    bool doneWithAllMotors = true;
    for (uint16_t i = 0; i < 2; i++)
    {
      uint8_t idx = motorPorts[i]->currMotor;
      uint8_t motorId = motorPorts[i]->motorIds[idx];
      uint8_t protocol = motorPorts[i]->protocol[idx];

      if (idx >= motorPorts[i]->numMotors){ // skip port if all motors already serviced
        continue;
      } else {
        doneWithAllMotors = false;
      }

      if(protocol == 1) read_present_position_p1(motorPorts[i], motorId);
      else              read_present_position_p1(motorPorts[i], motorId);
      rxBuf[motorId * 2] = motorPorts[i]->rxBuffer[0];
      rxBuf[motorId * 2 + 1] = motorPorts[i]->rxBuffer[1];
      motorPorts[i]->currMotor = idx + 1;
    }

    if(doneWithAllMotors) return; // all motors serviced, peace out
  }
}
