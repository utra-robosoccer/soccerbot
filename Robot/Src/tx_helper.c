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
#include "../Drivers/Dynamixel/h/Dynamixel_Types.h"

/********************************** Macros ***********************************/

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
