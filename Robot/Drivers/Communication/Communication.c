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
static volatile uint8_t buffRx[92];
static volatile uint8_t startSeqCount;
static volatile uint8_t totalBytesRead;
static volatile RobotState *robotStatePtr;
static volatile uint32_t error;

/*********************************** Externs **********************************/
extern UART_HandleTypeDef huart5;
extern osMutexId PCUARTHandle;
extern osThreadId rxTaskHandle;
extern osThreadId defaultTaskHandle;

/******************************** Functions ************************************/
void Comm_Init(volatile RobotGoal* robotGoal, volatile RobotState* robotState) {
	// Receiving
	robotGoal->id = 0;
	robotGoalPtr = robotGoal;
	robotGoalDataPtr = robotGoalData;
	startSeqCount = 0;
	totalBytesRead = 0;

	// Sending
	robotState->id = 0;
	robotStatePtr = robotState;
	robotState->start_seq = UINT32_MAX;
	robotState->end_seq = 0;
}

/* StartRxTask function */
void StartRxTask(void const * argument) {
	/* USER CODE BEGIN StartRxTask */
	xTaskNotifyWait(UINT32_MAX, UINT32_MAX, NULL, portMAX_DELAY);

	HAL_StatusTypeDef status;

	uint32_t notification;

	HAL_UART_Receive_DMA(&huart5, (uint8_t*)buffRx, sizeof(buffRx));

	/* Infinite loop */
	for (;;) {
		// Wait until notified from ISR. Clear no bits on entry in case the notification
		// comes before this statement is executed (which is rather unlikely as long as
		// this task has the highest priority, but overall this is a better decision in
		// case priorities are changed in the future and someone forgets about this.
		do{
			xTaskNotifyWait(0, 0x80, &notification, portMAX_DELAY);
		}while((notification & 0x80) != 0x80);

		do{
			// This do-while loop with the mutex inside of it makes calls to the UART module
			// responsible for PC communication atomic. This attempts to solve the following
			// scenario: the TX thread is in the middle of executing the call to HAL_UART_Transmit
			// when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
			// returns immediately when it detects that the uart module is already locked. Then
			// the RX thread blocks itself and never wakes up since a RX transfer was never
			// initialized.
			xSemaphoreTake(PCUARTHandle, 1);
			status = HAL_UART_Receive_DMA(&huart5, (uint8_t*)buffRx, sizeof(buffRx));
			xSemaphoreGive(PCUARTHandle);
		}while(status != HAL_OK);

		for (uint8_t i = 0; i < sizeof(buffRx); i++) {
			if (startSeqCount == 4) {
				// This control block is entered when the header sequence of
				// 0xFFFFFFFF has been received; thus we know the data we
				// receive will be in tact

				*robotGoalDataPtr = buffRx[i];
				robotGoalDataPtr++;
				totalBytesRead++;

				if (totalBytesRead == sizeof(RobotGoal)) {
					// If, after the last couple of receive interrupts, we have
					// received sizeof(RobotGoal) bytes, then we copy the data
					// buffer into the robotGoal structure and wake the control
					// thread to distribute states to each actuator
					memcpy(&robotGoal, robotGoalData, sizeof(RobotGoal));

					// Reset the variables to help with reception of a RobotGoal
					robotGoalDataPtr = robotGoalData;
					startSeqCount = 0;
					totalBytesRead = 0;

					xTaskNotify(defaultTaskHandle, 0x40, eSetBits); // Wake control task
					continue;
				}
			}else{
				// This control block is used to verify that the data header is in tact
				if (buffRx[i] == 0xFF) {
					startSeqCount++;
				} else {
					startSeqCount = 0;
				}
			}
		}
	}
	/* USER CODE END StartRxTask */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	if (huart == &huart5) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(rxTaskHandle, 0x80, eSetBits,
				&xHigherPriorityTaskWoken);

		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
     error = HAL_UART_GetError(huart);
}
