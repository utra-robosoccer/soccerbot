/**
 *****************************************************************************
 * @file    tx_helper.c
 * @author  TODO -- your name here
 * @brief   TODO -- brief description of file
 *
 * @defgroup TODO -- module name
 * @brief    TODO -- description of module
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "tx_helper.h"
#include "cmsis_os.h"
#include "UART_Handler.h"
#include "../Drivers/Dynamixel/h/Dynamixel_Types.h"
#include "../Drivers/Communication/robotState.h"
#include "../Drivers/Communication/Communication.h"
#include "../Drivers/Dynamixel/h/Dynamixel_Data.h"
#include "../Drivers/MPU6050/MPU6050.h"
#include "Notification.h"

/********************************** Externs ***********************************/
extern osThreadId PCUARTHandle;
extern osThreadId TXQueueHandle;

/********************************* Constants *********************************/

/********************************** Types ************************************/

/****************************** Public Variables *****************************/

/***************************** Private Variables *****************************/
TXData_t receivedData;
Dynamixel_HandleTypeDef* motorPtr = NULL;
MPU6050_HandleTypeDef* imuPtr = NULL;
char* const pIMUXGyroData = &robotState.msg[ROBOT_STATE_MPU_DATA_OFFSET];

HAL_StatusTypeDef status;
uint32_t notification;
uint32_t dataReadyFlags = 0; // Bits in this are set based on which sensor data is ready

uint32_t NOTIFICATION_MASK = 0x80000000;

/************************ Private Function Prototypes ************************/

/******************************** Functions **********************************/
/*****************************************************************************/
void shiftNotificationMask(void) {
	for (uint8_t i = 1; i <= 12; i++) {
		NOTIFICATION_MASK |= (1 << i);
	}
}

void copySensorDataToSend(void) {
	while ((dataReadyFlags & NOTIFICATION_MASK) != NOTIFICATION_MASK) {
		while (xQueueReceive(TXQueueHandle, &receivedData, portMAX_DELAY)
				!= pdTRUE);

		switch (receivedData.eDataType) {
		case eMotorData:
			motorPtr = (Dynamixel_HandleTypeDef*) receivedData.pData;

			if (motorPtr == NULL) {
				break;
			}

			// Validate data and store it in robotState
			if (motorPtr->_ID <= NUM_MOTORS) {
				// Copy sensor data for this motor into its section of robotState.msg
				memcpy(&robotState.msg[4 * (motorPtr->_ID - 1)],
						&(motorPtr->_lastPosition), sizeof(float));

				// Set flag indicating the motor with this id has reported in with position data
				dataReadyFlags |= (1 << motorPtr->_ID);
			}
			break;
		case eIMUData:
			imuPtr = (MPU6050_HandleTypeDef*) receivedData.pData;

			if (imuPtr == NULL) {
				break;
			}

			// Copy sensor data into the IMU data section of robotState.msg
			memcpy(pIMUXGyroData, (&imuPtr->_X_GYRO), 6 * sizeof(float));

			// Set flag indicating IMU data has reported in
			dataReadyFlags |= 0x80000000;
			break;
		default:
			break;
		}
	}
	dataReadyFlags = 0; // Clear all flags
}

void transmitStatusFromPC(void) {
	do {
		// This do-while loop with the mutex inside of it makes calls to the UART module
		// responsible for PC communication atomic. This attempts to solve the following
		// scenario: the TX thread is in the middle of executing the call to HAL_UART_Transmit
		// when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
		// returns immediately when it detects that the uart module is already locked. Then
		// the RX thread blocks itself and never wakes up since a RX transfer was never
		// initialized.
		xSemaphoreTake(PCUARTHandle, 1);
		status = HAL_UART_Transmit_DMA(&huart5, (uint8_t*) &robotState,
				sizeof(RobotState));
		xSemaphoreGive(PCUARTHandle);
	} while (status != HAL_OK);
}

void waitForNotificationTX(void) {
	// Wait until notified from ISR. Clear no bits on entry in case the notification
	// came while a higher priority task was executing.
	do {
		xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, portMAX_DELAY);
	} while ((notification & NOTIFIED_FROM_TX_ISR) != NOTIFIED_FROM_TX_ISR);
}

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
