/**
  *****************************************************************************
  * @file    SystemConf.h
  * @author  Tyler
  * @brief   Enables support for various STM32 microcontrollers by choosing
  *          the appropriate headers to import depending on the build
  *          variables
  *
  * @defgroup System
  * @{
  *****************************************************************************
  */




#ifndef SYSTEM_CONF_H
#define SYSTEM_CONF_H

#if defined(STM32F446xx)
#include <stm32f446xx.h>

#elif defined(STM32F767xx)
#include <stm32f767xx.h>

#else
#error "invalid build configuration! Must define STM32F446xx or STM32F767xx"
#endif

/**
 * @}
 */
/* end System */

#endif /* SYSTEM_CONF_H */
