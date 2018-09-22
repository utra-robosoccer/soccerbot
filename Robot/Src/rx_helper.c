/**
 *****************************************************************************
 * @file    rx_helper.c
 * @author  TODO -- your name here
 * @brief   TODO -- brief description of file
 *
 * @defgroup TODO -- module name
 * @brief    TODO -- description of module
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "rx_helper.h"
#include "Notification.h"
#include "cmsis_os.h"
#include "../Drivers/Communication/robotGoal.h"
#include "../Drivers/Communication/robotState.h"
#include "../Drivers/Communication/Communication.h"
#include "usart.h"

/********************************** Macros ***********************************/

/********************************* Constants *********************************/

/********************************** Types ************************************/

/****************************** Public Variables *****************************/
extern osThreadId CommandTaskHandle;
extern osThreadId IMUTaskHandle;
extern osThreadId PCUARTHandle;

/***************************** Private Variables *****************************/
static uint8_t robotGoalData[sizeof(RobotGoal)];
static uint8_t *robotGoalDataPtr;
static uint8_t buffRx[92];
static uint8_t startSeqCount;
static uint8_t totalBytesRead;

HAL_StatusTypeDef status;
uint32_t notification;

/************************ Private Function Prototypes ************************/

/*******************************  Function  *****************************/
void initializeVars(void) {
	//sending
	robotGoal.id = 0;
	robotGoalDataPtr = robotGoalData;
	startSeqCount = 0;
	totalBytesRead = 0;
	//receiving
	robotState.id = 0;
	robotState.start_seq = UINT32_MAX;
	robotState.end_seq = 0;
}

void initiateDMATransfer(void) {
	HAL_UART_Receive_DMA(&huart5, (uint8_t*) buffRx, sizeof(buffRx));
}

void waitForNotificationRX(void) {
	// Wait until notified from ISR. Clear no bits on entry in case the notification
	// comes before this statement is executed (which is rather unlikely as long as
	// this task has the highest priority, but overall this is a better decision in
	// case priorities are changed in the future and someone forgets about this.
	do {
		xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, portMAX_DELAY);
	} while ((notification & NOTIFIED_FROM_RX_ISR) != NOTIFIED_FROM_RX_ISR);
}

void updateStatusToPC(void) {//
	do {
		// This do-while loop with the mutex inside of it makes calls to the UART module
		// responsible for PC communication atomic. This attempts to solve the following
		// scenario: the TX thread is in the middle of executing the call to HAL_UART_Transmit
		// when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
		// returns immediately when it detects that the uart module is already locked. Then
		// the RX thread blocks itself and never wakes up since a RX transfer was never
		// initialized.
		xSemaphoreTake(PCUARTHandle, 1);
		status = HAL_UART_Receive_DMA(&huart5, (uint8_t*) buffRx,
				sizeof(buffRx));
		xSemaphoreGive(PCUARTHandle);
	} while (status != HAL_OK);
}

void parsePacket(void) {
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
				robotState.id = robotGoal.id;

				// Reset the variables to help with reception of a RobotGoal
				robotGoalDataPtr = robotGoalData;
				startSeqCount = 0;
				totalBytesRead = 0;

				xTaskNotify(CommandTaskHandle, NOTIFIED_FROM_TASK, eSetBits);// Wake control task
				xTaskNotify(IMUTaskHandle, NOTIFIED_FROM_TASK, eSetBits);// Wake MPU task
				continue;
			}
		} else {
			// This control block is used to verify that the data header is in tact
			if (buffRx[i] == 0xFF) {
				startSeqCount++;
			} else {
				startSeqCount = 0;
			}
		}
	}

}
/******************************** Functions **********************************/
/*****************************************************************************/
/*  Submodule name                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup TODO -- define the submodule (something like "Module_SubmoduleName Submodule name")
 * @brief    TODO -- describe submodule here
 *
 * # TODO -- Page title for submodule #
 *
 * TODO -- detailed description of submodule
 *
 * @ingroup TODO -- name of the parent module
 * @{
 */

// TODO: put functions here
/**
 * @}
 */
/* end TODO -- Module_SubmoduleName */

