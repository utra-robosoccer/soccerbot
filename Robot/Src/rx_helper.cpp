/**
 *****************************************************************************
 * @file    rx_helper.c
 * @author  Hannah
 * @brief   Helper file for the function StartRXTask in freertos.cpp
 *
 * @defgroup Helpers
 * @ingroup  Threads
 * @brief Helper functions to help the read-ability of freertos.cpp
 * @{
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

static HAL_StatusTypeDef status;
static uint32_t notification;

/********************************  Functions  ********************************/
/*****************************************************************************/
/*  StartRxTask Helper Functions                                             */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup RxHelperFunctions StartRxTask Helper Functions
 * @brief    Helper functions for StartRxTask()
 *
 * # StartRXTask Helper Functions #
 *
 * This subsection includes helper functions for the StartRXTask() function in
 * freertos.cpp in order to make the the file more readable.
 *
 * @{
 */

/**
 * @brief   Initializes the private variables for StartRxTask
 * @param 	None
 * @return  None
 */
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

/**
 * @brief   Initiates DMA receive transfer using DMA
 * @param 	None
 * @return  None
 */
void initiateDMATransfer(void) {
    HAL_UART_Receive_DMA(&huart5, (uint8_t*) buffRx, sizeof(buffRx));
}

/**
 * @brief   Waits until notified from ISR.
 * @details This function clears no bits on entry in case the notification
 * 			comes before this statement is executed (which is rather unlikely as long as
 * 			this task has the highest priority), but overall this is a better decision in
 * 			case priorities are changed in the future.
 * @param 	None
 * @return  None
 */
void waitForNotificationRX(void) {
    do {
        xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, portMAX_DELAY);
    } while ((notification & NOTIFIED_FROM_RX_ISR) != NOTIFIED_FROM_RX_ISR);
}

/**
 * @brief   Makes calls to the UART module responsible for PC communication atomic
 * @details This attempts to solve the following scenario:
 * 			the TX thread is in the middle of executing the call to HAL_UART_Transmit
 * 			when suddenly the RX thread is unblocked. The RX thread calls HAL_UART_Receive, and
 * 			returns immediately when it detects that the UART module is already locked. Then
 * 			the RX thread blocks itself and never wakes up since a RX transfer was never
 * 			initialized.
 * @param 	None
 * @return  None
 */
void updateStatusToPC(void) {
    do {
        xSemaphoreTake(PCUARTHandle, 1);
        status = HAL_UART_Receive_DMA(&huart5, (uint8_t*) buffRx,
                sizeof(buffRx));
        xSemaphoreGive(PCUARTHandle);
    } while (status != HAL_OK);
}

/**
 * @brief   Reads the received data buffer
 * @details Once verified that the header sequence is valid and the correct number of bytes
 * 			are read, the function copies robotGoalData to robotGoal and updates robotGoal id
 * 			to robotState id. Then, the function wakes up control task and MPU task by notifying.
 * @param 	None
 * @return  None
 */
void receiveDataBuffer(void) {
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

/**
 * @}
 */
/* end - RxHelperFunctions */

/**
 * @}
 */
/* end Helpers */

