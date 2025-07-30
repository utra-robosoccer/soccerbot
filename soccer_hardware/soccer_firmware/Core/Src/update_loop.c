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

#define UART_SEND 1000 //Send uart debug packet every one second

BMI088 imu;

extern Voltage* voltage[2];

void update()
{
  uint8_t txBuf[2 + 20 * 2 + 6 + 6] = {0}; // 20 motors * 2 bytes each + 6 bytes for IMU
  txBuf[0] = txBuf[1] = 0xfe;
  uint32_t lastTime = HAL_GetTick();
  // Just for read sync debug
//  uint32_t sync_debug_send = HAL_GetTick();

  while(1)
  {
    if(HAL_GetTick() - lastTime > UPDATE_PERIOD) // send IMU/ANGLES periodically back to main computer
    {
      lastTime = HAL_GetTick();

      // we read want to read 1 motor position from each port
      read_motors(txBuf); //this is where should be changed
//      read_motors_sync(txBuf);

      // get current linear Acceleration and Rotational speed
      read_imu(txBuf);

      CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));
      HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
    }
///////////////// UART DEBUG
//    if(HAL_GetTick() - sync_debug_send > UART_SEND){
////    	char message[100];
////    	uint32_t motor_angle = 0;
////    	uint8_t motor_count = 0;
////    	for (int i = 10; i < 22; i += 2){
////    		motor_count ++;
////    		motor_angle = txBuf[i] | (txBuf[i + 1] << 8);
////    		sprintf(message, "Motor #%d: %d\n", motor_count, (int)(motor_angle * 0.088));
////    		CDC_Transmit_FS(message, strlen(message));
////
////    	}
//    	sync_debug_send = HAL_GetTick();
//    	CDC_Transmit_FS((uint8_t *) txBuf, sizeof(txBuf));
//    	HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
//    }
////////////////
    if (usb_received) // when we receive USB packet, service it right away
    {
      // check for header packet
      if(usbRxBuffer[0] != 0xff || usbRxBuffer[1] != 0xff) {
    	  usb_received = false;
    	  continue;
      }

      command_motors();
//      command_motors_sync();
      usb_received = false;

//      HAL_GPIO_TogglePin(GPIOA, GREEN_LED_Pin);
    }

  }
}

void set_robot_motor_limits() {
	// position limits 0 - 4095 (1 rotation)
	// min position address => 48
	// max position address => 52

	// use index as motorID!! // ID 7 left knee and 13 right knee
	//                        0	    1     2	     3	    4	  5	    6	  7	    8	  9	    10	  11    12	  13	14	  15	16	17	18	  19
	uint32_t minLimits[20] = {1510, 1050, 2000,  2000,  1213, 1536, 1650,  760,  1610, 1825, 1100, 1550, 770, 2057, 920,  1870, 0,  0,  0,    1060};
	uint32_t maxLimits[20] = {2400, 3100, 2945,  4096,  3155, 2569, 3230, 2070, 3160, 2260, 3060, 2550, 3560, 3300, 2580, 2245, 0,  0,  3000, 4096};

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
//	      uint8_t protocol = motorPorts[i]->protocol[idx];

	      if (idx >= motorPorts[i]->numMotors){ // skip port if all motors already serviced
	        continue;
	      } else {
	        doneWithAllMotors = false;
	      }

	      // angle format depends on how Python script. Bit wise OR
//	      uint16_t angle = usbRxBuffer[2 + motorId * 2] | (usbRxBuffer[2 + motorId * 2 + 1] << 8);

	      write_min_position_limit_p2(motorPorts[i], motorId, minLimits[motorId]);
	      write_max_position_limit_p2(motorPorts[i], motorId, maxLimits[motorId]);

	      motorPorts[i]->currMotor = idx + 1;
	    }

	    HAL_Delay(1); // delay enough for motors to have time to respond

	    if(doneWithAllMotors) return; // all motors serviced, peace out
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

      if (idx >= motorPorts[i]->numMotors){ // skip port if all motors already serviced
        continue;
      } else {
        doneWithAllMotors = false;
      }


      // angle format depends on how Python script. Bit wise OR
      uint16_t angle = usbRxBuffer[2 + motorId * 2] | (usbRxBuffer[2 + motorId * 2 + 1] << 8);


      write_goal_position_p2(motorPorts[i], motorId, angle);


      motorPorts[i]->currMotor = idx + 1;
    }


    if(doneWithAllMotors)  {
    	//HAL_Delay(1); // delay enough for motors to have time to respond
    return; // all motors serviced, peace out
    }

  }
}

void command_motors_sync() {
//	char dummy[5] = {0xff};
//	CDC_Transmit_FS(dummy, 5);

	for (uint8_t i = 0; i < 6; i++) {// reset variables
	    motorPorts[i]->motorWrite = false;
	  }
  while(1)
  {
    for (uint16_t i = 0; i < 6; i++)
    {

    	MotorPort *p = motorPorts[i];
      if (p->motorWrite){ // skip port if all motors already serviced
        continue;
      }


      for (uint8_t j = 0; j< p->numMotors; j++) {
    	  p->angles[j] = usbRxBuffer[2 + p->motorIds[j] * 2] | (usbRxBuffer[2 + p->motorIds[j] * 2 + 1] << 8);
      }


      sync_write_goal_position_p2(p);
      p->motorWrite = true;
    }

    int count = 0;
    for (uint8_t i = 0; i < 6; i++) {
    	if (motorPorts[i]->motorWrite){
    	    ++count;
    	 }

  }
    if (count >= 6)
    		return;

}
}

void read_imu(uint8_t *rxBuf) {
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;

  BMI088_ReadAccelerometer(&imu, &hi2c1, &accX, &accY, &accZ);

  // Read gyro after accel, otherwise we get HAL_error (not sure why :/)
  BMI088_ReadGyroscope(&hi2c1, &gyroX, &gyroY, &gyroZ);

  rxBuf[2 + 20 * 2 + 0] = (accX >> 8) & 0xFF; // MSB first means big endian
  rxBuf[2 + 20 * 2 + 1] = accX & 0xFF;
  rxBuf[2 + 20 * 2 + 2] = (accY >> 8) & 0xFF;
  rxBuf[2 + 20 * 2 + 3] = accY & 0xFF;
  rxBuf[2 + 20 * 2 + 4] = (accZ >> 8) & 0xFF;
  rxBuf[2 + 20 * 2 + 5] = accZ & 0xFF;

  rxBuf[2 + 20 * 2 + 6 + 0] = (gyroX >> 8) & 0xFF;
  rxBuf[2 + 20 * 2 + 6 + 1] = gyroX & 0xFF;
  rxBuf[2 + 20 * 2 + 6 + 2] = (gyroY >> 8) & 0xFF;
  rxBuf[2 + 20 * 2 + 6 + 3] = gyroY & 0xFF;
  rxBuf[2 + 20 * 2 + 6 + 4] = (gyroZ >> 8) & 0xFF;
  rxBuf[2 + 20 * 2 + 6 + 5] = gyroZ & 0xFF;

}

void read_motors_sync(uint8_t *rxBuf) {
  for (uint16_t i = 0; i < 6; i++) {// reset variables
    motorPorts[i]->dmaDoneReading = false;
    motorPorts[i]->timeout = 0;
    motorPorts[i]->motorServiced = false;
  };

  // send read command to 1 motor on each port
  uint8_t numMotorsRequested = 0;
  for (uint8_t i = 0; i < 6; i ++) {
    MotorPort *p = motorPorts[i];
//    uint8_t currMotor = p->currReadMotor;
//    uint8_t motorId = p->motorIds[currMotor];
//    uint8_t protocol = p->protocol[currMotor];

    if (p->numMotors == 0) {
      continue;
    }

//      motor_torque_en_p2(p, motorId, 1);
//      HAL_Delay(1);

	p->rxPacketLen = 15 * p->numMotors;
	HAL_UART_Receive_DMA(p->huart, p->rxBuffer, p->rxPacketLen);
    sync_read_motor_present_position_p2(p);

//    read_motor_present_position_p2(p, motorId);
//
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
//      uint8_t idx = p->currReadMotor;
//      uint8_t motorId = p->motorIds[idx];
//      uint8_t protocol = p->protocol[idx];

      if (p->numMotors == 0 || p->motorServiced) {
        continue;
      }

      if (p->dmaDoneReading) {
    	for (uint8_t i = 0; i < p->numMotors; i ++) {
    		rxBuf[2 + p->motorIds[i] * 2] = p->rxBuffer[(15 * i) + 9];
    		rxBuf[2 + p->motorIds[i] * 2 + 1] = p->rxBuffer[(15 * i) +10];
    	}

        p->dmaDoneReading = false;
        numMotorsReceived++;
        p->motorServiced = true;
//        p->currReadMotor = (p->currReadMotor + 1) % p->numMotors;
      } else {
        //timeout logic
        if(HAL_GetTick() - p->timeout > 10) { // units in milliseconds
          HAL_UART_DMAStop(p->huart);
          numMotorsReceived++; // unsuccesful but we still count as received
          p->motorServiced = true;
//          rxBuf[2 + p->motorIds[i] * 2] = 0xFF;
//          rxBuf[2 + p->motorIds[i] * 2 + 1] = 0xFF;

//          p->currReadMotor = (p->currReadMotor + 1) % p->numMotors;
        }
      }
    }
    if(numMotorsReceived == numMotorsRequested) return; // all motors serviced, peace out
  }
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


      p->rxPacketLen = 15;
      HAL_UART_Receive_DMA(p->huart, p->rxBuffer, p->rxPacketLen);
      read_motor_present_position_p2(p, motorId);

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

void update_voltage(void){
	Voltage *v_1 = voltage[0];
	Voltage *v_2 = voltage[1];


	//uint16_t readValue_1;
	uint16_t readValue_2;
	uint16_t readValue;

	//float v_bat; //IN 8
	//float v_shunt_read;// = v_2->v_read; //IN 7
	//float intensity_shunt;// = v_2->intensity;// intensity
	//(*v_1->hadc).Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;// |= allows the continuous display

	HAL_ADC_Start(v_1->hadc);
	if (HAL_ADC_PollForConversion(v_1->hadc, 10) == HAL_OK){
		readValue = HAL_ADC_GetValue(v_1->hadc);
		voltage[0]->v_read = (float)readValue/4095*3.3;
	}

	HAL_ADC_Start(v_2->hadc);
	if (HAL_ADC_PollForConversion(v_2->hadc, 10) == HAL_OK){
		readValue_2 = HAL_ADC_GetValue(v_2->hadc);
		voltage[1]->v_read = (float)readValue_2/4095*3.3;
		voltage[1]-> intensity = (float)(voltage[1]-> v_read)/(20)/0.01;  //3.3; //4095*16.5;

	}

	HAL_Delay(100); // time between each change

}

