/*
 * Communication.c
 *
 *  Created on: May 28, 2018
 *      Author: Admin
 */


#include "Communication.h"

volatile RobotGoal robotGoal;
volatile RobotState robotState;
static volatile RobotGoal *robotGoalPtr;
static volatile uint8_t robotGoalData[sizeof(RobotGoal)];
static volatile uint8_t *robotGoalDataPtr;
static volatile uint8_t buf[10];
static volatile uint8_t startSeqCount;
static volatile uint8_t totalBytesRead;
static volatile RobotState *robotStatePtr;

extern UART_HandleTypeDef huart5;

void Comm_Init(volatile RobotGoal* robotGoal, volatile RobotState* robotState){
	// Receiving
	robotGoal -> id = 0;
	robotGoalPtr = robotGoal;
	robotGoalDataPtr = robotGoalData;
	startSeqCount = 0;
	totalBytesRead = 0;

	// Sending
	robotState -> id = 0;
	robotStatePtr = robotState;
	robotState -> start_seq = UINT32_MAX;
	robotState -> end_seq = 0;

	HAL_UART_Receive_IT(&huart5, (uint8_t *) &buf, sizeof(buf));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	if (huart == &huart5) {
		for(uint8_t i = 0; i < sizeof(buf); ++i) {
			if(startSeqCount == 4) {
				*robotGoalDataPtr = buf[i];
				robotGoalDataPtr++;
				totalBytesRead++;

				if(totalBytesRead == sizeof(RobotGoal)) {

					// Process RobotGoal here
					memcpy(&robotGoal, &robotGoalData, sizeof(RobotGoal));

					robotGoalDataPtr = robotGoalData;
					startSeqCount = 0;
					totalBytesRead = 0;
					continue;
				}
			}
			else {
				if(buf[i] == 0xFF)
					startSeqCount++;
				else
					startSeqCount = 0;
			}
		}

		HAL_UART_AbortReceive_IT(&huart5);
		HAL_UART_Receive_IT(&huart5, (uint8_t *) &buf, sizeof(buf));
	}
}
