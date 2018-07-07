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

#ifndef COMMUNICATION_COMMUNICATION_H_
#define COMMUNICATION_COMMUNICATION_H_

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

#endif /* COMMUNICATION_COMMUNICATION_H_ */
