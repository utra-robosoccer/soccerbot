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

/* Default UART assignments. */
UART_HandleTypeDef* UART_HANDLE_UpperLeftLeg = &huart1;
UART_HandleTypeDef* UART_HANDLE_UpperRightLeg = &huart4;
UART_HandleTypeDef* UART_HANDLE_LowerLeftLeg = &huart6;

#if defined(USE_DEBUG_UART)
#pragma "SystemConf warning: make sure UART configurations have been updated in the usart.c file."

#if defined(STM32F446xx)
/* Swap UARTs 2 and 5. */
UART_HandleTypeDef* UART_HANDLE_PC = &huart2;
UART_HandleTypeDef* UART_HANDLE_LowerRightLeg = &huart5;
UART_HandleTypeDef* UART_HANDLE_HeadAndArms = &huart3;

#elif defined(STM32F767xx)
/* Swap UARTs 3 and 5. */
UART_HandleTypeDef* UART_HANDLE_PC = &huart3;
UART_HandleTypeDef* UART_HANDLE_LowerRightLeg = &huart2;
UART_HandleTypeDef* UART_HANDLE_HeadAndArms = &huart5;

#endif

#else

/* These UART assignments are defaults across both the F4 and F7 boards for now. */
UART_HandleTypeDef* UART_HANDLE_PC = &huart5;
UART_HandleTypeDef* UART_HANDLE_LowerRightLeg = &huart2;
UART_HandleTypeDef* UART_HANDLE_HeadAndArms = &huart3;

#endif

/**
 * @}
 */
/* end - system_conf */
