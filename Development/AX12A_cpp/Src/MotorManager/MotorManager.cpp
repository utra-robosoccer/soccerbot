/*
 * MotorManager.cpp
 *
 *  Created on: Mar 17, 2018
 *      Author: Admin
 */




/********************************** Includes **********************************/
#include "MotorManager.h"
#include <cassert>




/******************************* Public Variables *******************************/
///* Buffer for data received from motors. */
//uint8_t arrReceive[NUM_MOTORS][BUFF_SIZE_RX] = {{0}};
//
///* Bytes to be transmitted to motors are written to this array. */
//// TODO: Since the AX-12A and MX-28 motors have different packet sizes,
//// replace this array by an array of pointers to arrays of the appropriate size
//uint8_t arrTransmit[NUM_MOTORS + 1][TX_PACKET_SIZE] = {
//	{0xFF, 0xFF, 0xFE, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 1, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 2, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 3, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 4, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 5, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 6, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 7, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 8, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 9, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 10, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 11, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 12, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 13, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 14, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 15, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 16, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 17, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00},
//	{0xFF, 0xFF, 18, 0x00, INST_WRITE_DATA, 0x00, 0x00, 0x00, 0x00}
//};
//
///* Sync write buffer. */
//uint8_t arrSyncWrite[NUM_UARTS][BUFF_SIZE_SYNC_WRITE];




/****************************** Private Variables *******************************/
static const double PI = M_PI; // From math.h

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

// TODO: Bake these constants into each individual motor; assign them at initialization
// time and then automatically take care of them in all position commands
MotorCalibrationConstants motorCalibrationConstants [NUM_MOTORS] = {
		{-180 / PI, 150 - 1}, // Motor1
		{-180 / PI, 150 + 3}, // Motor2
		{-180 / PI, 150 + 1}, // Motor3
		{180 / PI, 150 + 2},  // Motor4
		{180 / PI, 150 - 0},  // Motor5
		{-180 / PI, 150 - 0}, // Motor6
		{180 / PI, 150 - 0},  // Motor7
		{180 / PI, 150 - 3},  // Motor8
		{-180 / PI, 150 - 0}, // Motor9
		{180 / PI, 150 + 4},  // Motor10
		{180 / PI, 150 + 1},  // Motor11
		{-180 / PI, 150 + 3}, // Motor12
		{180 / PI, 150 - 0},  // Motor13 -- NOT CALIBRATED
		{180 / PI, 150 - 0},  // Motor14 -- NOT CALIBRATED
		{180 / PI, 150 - 0},  // Motor15 -- NOT CALIBRATED
		{180 / PI, 150 - 0},  // Motor16 -- NOT CALIBRATED
		{180 / PI, 150 - 0},  // Motor17 -- NOT CALIBRATED
		{180 / PI, 150 - 0}   // Motor18 -- NOT CALIBRATED
};




/**************************** Method Implementations ****************************/
MotorManager::MotorManager() {
	/* Instantiate motors and initialize object fields. */

	assert(NUM_MOTORS == 18); // The assignments in motorInitData must be hardcoded for NUM_MOTORS motors

	for(int motor = MOTOR1; motor <= MOTOR18; motor++){
		AX12A* motorPtr = new AX12A(&motorInitData[motor]);
		this -> motorTable[motor] = motorPtr;
	}
}

MotorManager::~MotorManager() {
	// TODO Auto-generated destructor stub
}

