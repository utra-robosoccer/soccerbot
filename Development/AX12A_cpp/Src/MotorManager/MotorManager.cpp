/*
 * MotorManager.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Admin
 */

#include "MotorManager.h"


/******************************* Public Variables *******************************/
/* Buffer for data received from motors. */
uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = {{0}};

/* Bytes to be transmitted to motors are written to this array. */
// TODO: Since the AX-12A and MX-28 motors have different packet sizes,
// replace this array by an array of pointers to arrays of the appropriate size
uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE] = {
	{0xFF, 0xFF, 0xFE, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 1, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 2, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 3, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 4, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 5, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 6, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 7, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 8, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 9, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 10, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 11, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 12, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 13, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 14, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 15, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 16, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 17, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0xFF, 18, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00}
};

/* Sync write buffer. */
uint8_t arrSyncWrite[NUM_UARTS][BUFF_SIZE_SYNC_WRITE];


static MotorInitData motorInitData [NUM_MOTORS] = {
	{1, &huart2, GPIOD, GPIO_PIN_7},
	{2, &huart2, GPIOD, GPIO_PIN_7},
	{3, &huart2, GPIOD, GPIO_PIN_7},
	{4, &huart1, GPIOB, GPIO_PIN_10},
	{5, &huart1, GPIOB, GPIO_PIN_10},
	{6, &huart1, GPIOB, GPIO_PIN_10},
	{7, &huart7, GPIOA, GPIO_PIN_15},
	{8, &huart7, GPIOA, GPIO_PIN_15},
	{9, &huart7, GPIOA, GPIO_PIN_15},
	{10, &huart5, GPIOB, GPIO_PIN_0},
	{11, &huart5, GPIOB, GPIO_PIN_0},
	{12, &huart5, GPIOB, GPIO_PIN_0},
	{13, &huart4, GPIOA, GPIO_PIN_0},
	{14, &huart4, GPIOA, GPIO_PIN_0},
	{15, &huart4, GPIOA, GPIO_PIN_0},
	{16, &huart4, GPIOA, GPIO_PIN_0},
	{17, &huart4, GPIOA, GPIO_PIN_0},
	{18, &huart4, GPIOA, GPIO_PIN_0}
};


MotorManager::MotorManager() {
	/* Instantiate motors and initialize object fields. */

	for(int motor = MOTOR1; motor <= MOTOR18; motor++){
		this->motorTable[motor] = new AX12A(motorInitData[motor]);
	}
}

MotorManager::~MotorManager() {
	// TODO Auto-generated destructor stub
}

