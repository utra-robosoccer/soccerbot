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
#include "MPU6050/MPU6050.h"
#include "Notification.h"
#include "BufferBase.h"

/***************************** Private Variables *****************************/
static MotorData_t readMotorData;
static imu::IMUStruct_t readIMUData;

static char* const pIMUXGyroData = &robotState.msg[
    ROBOT_STATE_MPU_DATA_OFFSET
];

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

/*****************************************************************************/
/**
 * @}
 */
/* end - TxHelperFunctions */
