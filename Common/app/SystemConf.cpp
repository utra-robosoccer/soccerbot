/**
  *****************************************************************************
  * @file SystemConf.cpp
  * @author Robert
  *
  * @defgroup system_conf
  * @ingroup System
  * @brief Data for SystemConf.
  * @{
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "SystemConf.h"

UART_HandleTypeDef* UART_HANDLE_UpperLeftLeg = &huart1;
UART_HandleTypeDef* UART_HANDLE_UpperRightLeg = &huart4;
UART_HandleTypeDef* UART_HANDLE_LowerLeftLeg = &huart6;

#if USE_DEBUG_UART == 0
UART_HandleTypeDef* UART_HANDLE_PC = &huart5;
UART_HandleTypeDef* UART_HANDLE_LowerRightLeg = &huart2;
UART_HandleTypeDef* UART_HANDLE_HeadAndArms = &huart3;

#elif USE_DEBUG_UART == 1
#pragma "SystemConf warning: make sure UART configurations have been updated in the usart.c file."

#if defined(STM32F446xx)
UART_HandleTypeDef* UART_HANDLE_PC = &huart2;
UART_HandleTypeDef* UART_HANDLE_LowerRightLeg = &huart5;
UART_HandleTypeDef* UART_HANDLE_HeadAndArms = &huart3;

#elif defined(STM32F767xx)
UART_HandleTypeDef* UART_HANDLE_PC = &huart3;
UART_HandleTypeDef* UART_HANDLE_LowerRightLeg = &huart2;
UART_HandleTypeDef* UART_HANDLE_HeadAndArms = &huart5;

#endif

#else
#error "SystemConf error: USE_DEBUG_UART must be set to 0 or 1, and at least one board defined."
#endif

/**
 * @}
 */
/* end - system_conf */
