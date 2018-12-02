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

/* In general, the build variables such as STM32F* should not be used
outside of the SystemConf.h and SystemConf.cpp files. A new flag should
be defined here and used throughout the project. */

#include "usart.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* Deal with making sure one and only one board is defined here. */
#if defined(STM32F446xx) == defined(STM32F767xx)
#error "SystemConf error: invalid build configuration! Must define only one of STM32F446xx or STM32F767xx."
#endif

#if defined(STM32F446xx)
#include <stm32f446xx.h>
#define ARM_MATH_CM4 // Use ARM Cortex M4

#elif defined(STM32F767xx)
#include <stm32f767xx.h>
#define ARM_MATH_CM7 // Use ARM Cortex M7

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
 *
 * Double check usart.c that the correct DMA settings are being applied for the
 * swapped UARTs. Also double check in freertos.c that the correct baud rate
 * for the swapped UARTS is being set in StartCmdTask.
 */
#if defined(STM32F446xx)
// #define USE_DEBUG_UART

#elif defined(STM32F767xx)
/* TODO: have USE_DEBUG_UART undefined by default (i.e. commented out here)
once F7 board is ready. */
#define USE_DEBUG_UART

#endif

extern UART_HandleTypeDef* UART_HANDLE_PC;
extern UART_HandleTypeDef* UART_HANDLE_UpperLeftLeg;
extern UART_HandleTypeDef* UART_HANDLE_LowerRightLeg;
extern UART_HandleTypeDef* UART_HANDLE_HeadAndArms;
extern UART_HandleTypeDef* UART_HANDLE_UpperRightLeg;
extern UART_HandleTypeDef* UART_HANDLE_LowerLeftLeg;

#if defined(STM32F767xx)
#define USE_MANUAL_UART_ABORT_DEFINITIONS
#endif

#if defined(STM32F446xx)
#define USE_I2C_SILICON_BUG_FIX
#endif

/**
 * @}
 */
/* end System */

#endif /* SYSTEM_CONF_H */
