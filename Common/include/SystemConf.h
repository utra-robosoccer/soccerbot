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
//#define USE_DEBUG_UART // Uncomment if debug cable connected to F4 dev board

#elif defined(STM32F767xx)
/* TODO(rfairley): have USE_DEBUG_UART undefined by default (i.e. commented
 * out here) once F7 PCB by Electrical team is ready. */
#define USE_DEBUG_UART // Uncomment if debug cable connected to F7 dev board

#endif

/* Default UART assignments. */
#define UART_HANDLE_UpperLeftLeg &huart1
#define UART_HANDLE_UpperRightLeg &huart4
#define UART_HANDLE_LowerLeftLeg &huart6

#if defined(USE_DEBUG_UART)

#if defined(STM32F446xx)
/* Swap UARTs 2 and 5.
 * UART 2 is the debug UART on the F4; UART 5 is the PC
 * communications UART on our PCB */
#define UART_HANDLE_PC &huart2
#define UART_HANDLE_LowerRightLeg &huart5
#define UART_HANDLE_HeadAndArms &huart3

#elif defined(STM32F767xx)
/* Swap UARTs 3 and 5.
 * UART 3 is the debug UART on the F7; UART 5 is the PC
 * communications UART on our PCB */
#define UART_HANDLE_PC &huart3
#define UART_HANDLE_LowerRightLeg &huart2
#define UART_HANDLE_HeadAndArms &huart5

#endif

#else

/* These UART assignments are defaults across both the F4 and F7 boards for now. */
#define UART_HANDLE_PC &huart5
#define UART_HANDLE_LowerRightLeg &huart2
#define UART_HANDLE_HeadAndArms &huart3

#endif

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
