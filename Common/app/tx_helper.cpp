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
#include "BufferBase.h"

/********************************** Externs **********************************/
extern osThreadId PCUARTHandle;

/***************************** Private Variables *****************************/
static MotorData_t readMotorData;
static imu::IMUStruct_t readIMUData;

static char* const pIMUXGyroData = &robotState.msg[
    ROBOT_STATE_MPU_DATA_OFFSET
];

static HAL_StatusTypeDef status;
static uint32_t notification;

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
 * @brief   Validates and copies sensor data to transmit
 * @details This function receives two types of sensor data(motor and IMU) and
 *          updates to the according section of robotState.msg
 * @param 	BufferMasterPtr Pointer to the sensor data buffer
 */
void copySensorDataToSend(buffer::BufferMaster* BufferMasterPtr) {
    readIMUData = BufferMasterPtr->IMUBuffer.read();
    memcpy(pIMUXGyroData, (&readIMUData.x_Gyro), sizeof(imu::IMUStruct_t));

    for(int i = 0; i <= periph::MOTOR12; ++i)
    {
        readMotorData = BufferMasterPtr->MotorBufferArray[i].read();
        memcpy(&robotState.msg[4 * (readMotorData.id - 1)],
               &readMotorData.payload,
               sizeof(float)
        );
    }
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
