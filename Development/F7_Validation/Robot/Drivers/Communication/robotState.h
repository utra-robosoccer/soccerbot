/**
  ******************************************************************************
  * @file    robotState.h
  * @author  Jason
  * @brief   Defines the RobotState data structure used in communication with
  *          the high-level software. RobotState is the data structure sent from
  *          the MCU to the PC; it contains sensor data
  ******************************************************************************
  */

#ifndef ROBOTSTATE_H_
#define ROBOTSTATE_H_

#ifdef __cplusplus
extern "C" {
#endif




/** @brief Data structure sent from the MCU to the PC. Contains sensor data */
typedef struct robot_state {
	uint32_t start_seq; /**< Start sequence to attach to message (for data
	                         integrity purposes)                             */
	uint32_t id;        /**< Message ID */
	char msg[80];       /**< Raw message data as bytes sent over a serial
	                         terminal                                        */
	uint32_t end_seq;   /**< End sequence to attach to message (for data
                             integrity purposes)                             */
} RobotState;

#define ROBOT_STATE_MPU_DATA_OFFSET 48 /**< Buffer offset at which MPU6050 data
                                            is inserted                       */

#ifdef __cplusplus
}
#endif

#endif /* ROBOTSTATE_H_ */
