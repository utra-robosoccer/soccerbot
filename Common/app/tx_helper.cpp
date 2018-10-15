/**
 *****************************************************************************
 * @file    tx_helper.c
 * @author  Hannah
 * @brief   Helper file for the function StartTXTask() in freertos.cpp
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "uart_handler.h"
#include "tx_helper.h"
#include "cmsis_os.h"
#include "PeripheralInstances.h"
#include "robotState.h"
#include "Communication.h"
#include "MPU6050.h"
#include "Notification.h"

/********************************** Externs **********************************/
extern osThreadId PCUARTHandle;
extern osThreadId TXQueueHandle;

/***************************** Private Variables *****************************/
static TXData_t receivedData;
static MotorData_t* motorDataPtr = nullptr;
static imu::IMUStruct_t* imuPtr = nullptr;

static char* const pIMUXGyroData = &robotState.msg[
    ROBOT_STATE_MPU_DATA_OFFSET
];

static HAL_StatusTypeDef status;
static uint32_t notification;

// Bits in this are set based on which sensor data is ready
static uint32_t dataReadyFlags = 0;

static uint32_t NOTIFICATION_MASK = 0x80000000;

/******************************** Functions **********************************/
/*  StartTxTask Helper Functions                                             */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup TxHelperFunctions StartTxTask Helper Functions
 * @brief    Helper functions for StartTxTask()
 *
 * # StartTxTask Helper Functions #
 *
 * @ingroup Helpers
 * @{
 */

/**
 * @brief   Shifts bits of NOTIFICATION_MASK
 * @param 	None
 * @return  None
 */
void shiftNotificationMask(void) {
    for (uint8_t i = 1; i <= 12; i++) {
        NOTIFICATION_MASK |= (1 << i);
    }
}

/**
 * @brief   Validates and copies sensor data to transmit
 * @details This function receives two types of sensor data(motor and IMU) and
 *          updates to the according section of robotState.msg
 * @param 	None
 * @return  None
 */
void copySensorDataToSend(void) {
    while ((dataReadyFlags & NOTIFICATION_MASK) != NOTIFICATION_MASK) {
        while (xQueueReceive(TXQueueHandle, &receivedData, portMAX_DELAY) != pdTRUE);

        switch (receivedData.eDataType) {
        case eMotorData:
            motorDataPtr = static_cast<MotorData_t*>(receivedData.pData);

            if (motorDataPtr == NULL) {
                break;
            }

            // Validate data and store it in robotState
            if (motorDataPtr->id <= periph::NUM_MOTORS) {
                // Copy sensor data for this motor into its section of
                // robotState.msg
                memcpy(
                    &robotState.msg[4 * (motorDataPtr->id - 1)],
                    &(motorDataPtr->payload),
                    sizeof(float)
                );

                // Set flag indicating the motor with this id has reported in
                // with position data
                dataReadyFlags |= (1 << motorDataPtr->id);
            }
            break;
        case eIMUData:
            imuPtr = (imu::IMUStruct_t*)receivedData.pData;

            if(imuPtr == NULL){ break; }

            // Copy sensor data into the IMU data section of robotState.msg
            memcpy(pIMUXGyroData, (&imuPtr->x_Gyro), sizeof(imu::IMUStruct_t));

            // Set flag indicating IMU data has reported in
            dataReadyFlags |= 0x80000000;
            break;
        default:
            break;
        }
    }
    dataReadyFlags = 0; // Clear all flags
}

/**
 * @brief   Makes calls to the UART module responsible for PC communication atomic.
 * @details This do-while loop with the mutex inside of it makes c This attempts to solve the following
 *			scenario: the TX thread is in the middle of executing the call to HAL_UART_Transmit
 * 			when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
 *			returns immediately when it detects that the uart module is already locked. Then
 *			the RX thread blocks itself and never wakes up since a RX transfer was never
 *			initialized.
 * @param 	None
 * @return  None
 */
void transmitStatusFromPC(void) {
    do {
        xSemaphoreTake(PCUARTHandle, 1);
        status = HAL_UART_Transmit_DMA(&huart5, (uint8_t*) &robotState,
                sizeof(RobotState));
        xSemaphoreGive(PCUARTHandle);
    } while (status != HAL_OK);
}

/**
 * @brief   Waits until notified from ISR
 * @details Clear no bits on entry in case the notification came while a higher priority task
 * 			was executing.
 * @param 	None
 * @return  None
 */
void waitForNotificationTX(void) {
    do {
        xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, portMAX_DELAY);
    } while ((notification & NOTIFIED_FROM_TX_ISR) != NOTIFIED_FROM_TX_ISR);
}

/*****************************************************************************/
/**
 * @}
 */
/* end - TxHelperFunctions */
