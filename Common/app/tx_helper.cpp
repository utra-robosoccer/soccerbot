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
#include "Communication.h"
#include "MPU6050/MPU6050.h"
#include "Notification.h"
#include "BufferBase.h"

using namespace soccerbot; // TODO(tgamvrel) using namespace

/***************************** Private Variables *****************************/
static MotorData_t read_motor_data;
static imu::ImuStruct_t read_imu_data;


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
 * @param 	p_buffer_master Pointer to the sensor data buffer
 */
void copySensorDataToSend(buffer::BufferMaster* p_buffer_master) {
    read_imu_data = p_buffer_master->m_imu_buffer.read();
    comm::RobotState_t& robot_state = comm::get_robot_state();

    memcpy(
        &reinterpret_cast<uint8_t*>(
            &robot_state
        )[comm::ROBOT_STATE_MPU_DATA_OFFSET],
        (&read_imu_data.x_Gyro),
        sizeof(imu::ImuStruct_t)
    );

    for(int i = 0; i <= periph::MOTOR12; ++i)
    {
        read_motor_data = p_buffer_master->m_motor_buffer_array[i].read();
        memcpy(
            &robot_state.msg[4 * (read_motor_data.id - 1)],
            &read_motor_data.payload,
            sizeof(float)
        );
    }
}

/*****************************************************************************/
/**
 * @}
 */
/* end - TxHelperFunctions */
