/**
  ******************************************************************************
  * @file    Communication.h
  * @author  Jason
  * @author  Tyler
  * @brief   Header for top-level communication module
  *
  * @defgroup CommunicationHeader Communication (Header)
  * @brief    Header for communication, showing the public content
  * @ingroup  Communication
  * @{
  ******************************************************************************
  */




/******************** Define to prevent recursive inclusion *******************/
#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__




/********************************** Includes **********************************/
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "usart.h"
#include "robotState.h"
#include "robotGoal.h"




/******************************* Public Variables ******************************/
extern RobotGoal robotGoal;
extern RobotState robotState;

/**
 * @}
 */
/* end CommunicationHeader */

#endif /* __COMMUNICATION_H__ */
