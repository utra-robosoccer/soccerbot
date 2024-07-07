/*
 * update_loop.c
 */


#include "update_loop.h"

#include <stdbool.h>
#include "dynamixel_p1.h"
#include "dynamixel_p2.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "MPU6050.h"
#include "BMI088.h"

#define UPDATE_PERIOD 1 // milliseconds

BMI088 imu;

void update()
{
  uint8_t txBuf[2 + 18 * 2 + 6 + 6] = {0}; // 18 motors * 2 bytes each + 6 bytes for IMU
  txBuf[0] = txBuf[1] = 0xfe;
  uint32_t lastTime = HAL_GetTick();
  while(1)
  {
    if(HAL_GetTick() - lastTime > UPDATE_PERIOD) // send IMU/ANGLES periodically back to main computer
    {
      lastTime = HAL_GetTick();

      // we read want to read 1 motor position from each port
      read_motors(txBuf);

      // get current linear Acceleration and Rotational speed
      read_imu(txBuf);

      CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));
      HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
    }

    if (usb_received) // when we receive USB packet, service it right away
    {
      // check for header packet
      if(usbRxBuffer[0] != 0xff || usbRxBuffer[1] != 0xff) {
    	  usb_received = false;
    	  continue;
      }

      command_motors();
      usb_received = false;

      HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
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


      // angle format depends on how Python script. Bit wise OR
      uint16_t angle = usbRxBuffer[2 + motorId * 2] | (usbRxBuffer[2 + motorId * 2 + 1] << 8);

      if(protocol == 1) {
        write_goal_position_p1(motorPorts[i], motorId, angle);
      } else {
        write_goal_position_p2(motorPorts[i], motorId, angle);
      }

      motorPorts[i]->currMotor = idx + 1;
    }

    HAL_Delay(1); // delay enough for motors to have time to respond

    if(doneWithAllMotors) return; // all motors serviced, peace out
  }
}

void read_imu(uint8_t *rxBuf) {
  uint8_t gyrBuff[6] = {0};
  Read_Gyroscope(gyrBuff);
  HAL_Delay(1); // wait some time before sending next request

  uint8_t accBuff[6] = {0};
  Read_Accelerometer_IT(accBuff);

  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;


  BMI088_ReadAccelerometer(&imu, &hi2c1, &accX, &accY, &accZ);

  // Read gyro after accel, otherwise we get HAL_error (not sure why :/)
  BMI088_ReadGyroscope(&hi2c1, &gyroX, &gyroY, &gyroZ);

  rxBuf[2 + 18 * 2 + 0] = (accX >> 8) & 0xFF; // MSB first means big endian
  rxBuf[2 + 18 * 2 + 1] = accX & 0xFF;
  rxBuf[2 + 18 * 2 + 2] = (accY >> 8) & 0xFF;
  rxBuf[2 + 18 * 2 + 3] = accY & 0xFF;
  rxBuf[2 + 18 * 2 + 4] = (accZ >> 8) & 0xFF;
  rxBuf[2 + 18 * 2 + 5] = accZ & 0xFF;

  rxBuf[2 + 18 * 2 + 6 + 0] = (gyroX >> 8) & 0xFF;
  rxBuf[2 + 18 * 2 + 6 + 1] = gyroX & 0xFF;
  rxBuf[2 + 18 * 2 + 6 + 2] = (gyroY >> 8) & 0xFF;
  rxBuf[2 + 18 * 2 + 6 + 3] = gyroY & 0xFF;
  rxBuf[2 + 18 * 2 + 6 + 4] = (gyroZ >> 8) & 0xFF;
  rxBuf[2 + 18 * 2 + 6 + 5] = gyroZ & 0xFF;

}

void read_motors(uint8_t *rxBuf) {
  for (uint16_t i = 0; i < 6; i++) {// reset variables
    motorPorts[i]->dmaDoneReading = false;
    motorPorts[i]->timeout = 0;
    motorPorts[i]->motorServiced = false;
  };

  // send read command to 1 motor on each port
  uint8_t numMotorsRequested = 0;
  for (uint8_t i = 0; i < 6; i ++) {
    MotorPort *p = motorPorts[i];
    uint8_t currMotor = p->currReadMotor;
    uint8_t motorId = p->motorIds[currMotor];
    uint8_t protocol = p->protocol[currMotor];

    if (p->numMotors == 0) {
      continue;
    }

    if (protocol == 1) { // expect different length based on protocol
      motor_torque_en_p1(p, motorId, 1);
      HAL_Delay(1);
      p->rxPacketLen = 8;
      HAL_UART_Receive_DMA(p->huart, p->rxBuffer, p->rxPacketLen);
      read_motor_present_position_p1(p, motorId);
    } else {
      motor_torque_en_p2(p, motorId, 1);
      HAL_Delay(1);
      p->rxPacketLen = 15;
      HAL_UART_Receive_DMA(p->huart, p->rxBuffer, p->rxPacketLen);
      read_motor_present_position_p2(p, motorId);
    }
    p->timeout = HAL_GetTick();
    numMotorsRequested++; // keep track of how many motors we are reading
  }

  // now we wait for all motors to respond back
  uint8_t numMotorsReceived = 0;
  while(1)
  {
    for (uint16_t i = 0; i < 6; i++)
    {
      MotorPort *p = motorPorts[i];
      uint8_t idx = p->currReadMotor;
      uint8_t motorId = p->motorIds[idx];
      uint8_t protocol = p->protocol[idx];

      if (p->numMotors == 0 || p->motorServiced) {
        continue;
      }

      if (p->dmaDoneReading) {
        if (protocol == 1) {
          rxBuf[2 + motorId * 2] = p->rxBuffer[5];
          rxBuf[2 + motorId * 2 + 1] = p->rxBuffer[6];
        } else {
          rxBuf[2 + motorId * 2] = p->rxBuffer[9];
          rxBuf[2 + motorId * 2 + 1] = p->rxBuffer[10];
        }
        p->dmaDoneReading = false;
        numMotorsReceived++;
        p->motorServiced = true;
        p->currReadMotor = (p->currReadMotor + 1) % p->numMotors;
      } else {
        //timeout logic
        if(HAL_GetTick() - p->timeout > 10) { // units in milliseconds
          HAL_UART_DMAStop(p->huart);
          numMotorsReceived++; // unsuccesful but we still count as received
          p->motorServiced = true;
          p->currReadMotor = (p->currReadMotor + 1) % p->numMotors;
        }
      }
    }

    if(numMotorsReceived == numMotorsRequested) return; // all motors serviced, peace out
  }
}
