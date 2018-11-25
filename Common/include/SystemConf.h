/**
  *****************************************************************************
  * @file    SystemConf.h
  * @author  Tyler
  * @author  Robert
  *
  *
  * @defgroup System
  * @brief Enables support for various STM32 microcontrollers by choosing
  *        the appropriate headers to import depending on the build
  *        variables
  * @{
  *****************************************************************************
  */




#ifndef SYSTEM_CONF_H
#define SYSTEM_CONF_H

#include "usart.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

#if defined(STM32F446xx)
#include <stm32f446xx.h>
#define ARM_MATH_CM4 // Use ARM Cortex M4

#elif defined(STM32F767xx)
#include <stm32f767xx.h>
#define ARM_MATH_CM7 // Use ARM Cortex M7

#else
#error "SystemConf error: invalid build configuration! Must define STM32F446xx or STM32F767xx."
#endif

/**
 * @brief Flag for whether or not to use the non-blocking driver features
 * supported by the OS (FreeRTOS). This is resolved at compile time
 */
#define THREADED

/**
 * @brief USE_DEBUG_UART is a flag to use the debug UART handle at the default
 * pins for the board specified for communication with the PC, instead of the
 * handle used on the custom PCB.
 */
#define USE_DEBUG_UART 0

extern UART_HandleTypeDef* UART_HANDLE_PC;
extern UART_HandleTypeDef* UART_HANDLE_StartUART1Task;
extern UART_HandleTypeDef* UART_HANDLE_StartUART2Task;
extern UART_HandleTypeDef* UART_HANDLE_StartUART3Task;
extern UART_HandleTypeDef* UART_HANDLE_StartUART4Task;
extern UART_HandleTypeDef* UART_HANDLE_StartUART6Task;

/**
 * @}
 */
/* end System */

#endif /* SYSTEM_CONF_H */
