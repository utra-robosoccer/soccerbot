/*
 * Communication.c
 *
 *  Created on: May 28, 2018
 *      Author: Admin
 */


/********************************** Includes **********************************/
#include "Communication.h"


/******************************* Public Variables *******************************/
volatile RobotGoal robotGoal;
volatile RobotState robotState;


/******************************* Private Variables ******************************/
static volatile RobotGoal *robotGoalPtr;
static volatile uint8_t robotGoalData[sizeof(RobotGoal)];
static volatile uint8_t *robotGoalDataPtr;
static volatile uint8_t buf[10];
static volatile uint8_t startSeqCount;
static volatile uint8_t totalBytesRead;
static volatile RobotState *robotStatePtr;


/*********************************** Externs **********************************/
extern UART_HandleTypeDef huart5;
extern osSemaphoreId semPCRxBuffHandle;
//extern osSemaphoreId semControlTaskHandle;


/******************************** Functions ************************************/
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
}

/* StartRxTask function */
void StartRxTask(void const * argument)
{
  /* USER CODE BEGIN StartRxTask */
//  xSemaphoreTake(semPCRxBuffHandle, portMAX_DELAY);
  /* Infinite loop */
  for(;;)
  {
	  HAL_UART_Receive_IT(&huart5, (uint8_t *) &buf, sizeof(buf));
//	  xSemaphoreTake(semPCRxBuffHandle, portMAX_DELAY);
  }
  /* USER CODE END StartRxTask */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	if (huart == &huart5){
//		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
//		xSemaphoreGiveFromISR(semPCRxBuffHandle, &xHigherPriorityTaskWoken);
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

	//					xSemaphoreGive(semControlTaskHandle);
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
		  HAL_UART_Receive_IT(&huart5, (uint8_t *) &buf, sizeof(buf));
	}
}
