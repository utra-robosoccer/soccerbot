/**
  ******************************************************************************
  * @file    Communication.h
  * @author  Jason
  * @brief   Header for top-level communication module
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

#endif /* COMMUNICATION_COMMUNICATION_H_ */
