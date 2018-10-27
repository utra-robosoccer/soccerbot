/**
  *****************************************************************************
  * @file    SystemConf.h
  * @author  Tyler
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

#if defined(STM32F446xx)
#include <stm32f446xx.h>
#define ARM_MATH_CM4 // Use ARM Cortex M4

#elif defined(STM32F767xx)
#include <stm32f767xx.h>
#define ARM_MATH_CM7 // Use ARM Cortex M7

#else
#error "invalid build configuration! Must define STM32F446xx or STM32F767xx"
#endif

/**
 * @brief Flag for whether or not to use the non-blocking driver features
 * supported by the OS (FreeRTOS). This is resolved at compile time
 */
#define THREADED

/**
 * @}
 */
/* end System */

#endif /* SYSTEM_CONF_H */
