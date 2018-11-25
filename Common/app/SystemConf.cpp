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

/* TODO: change the names of these constexprs to match the new thread names, once thread names are changed
   rfairley: the convention I'm using currently names the uart handle by the name of the thread using it
   (aside from PC), but this is not set in stone. */
UART_HandleTypeDef* UART_HANDLE_StartUART1Task = &huart1;
UART_HandleTypeDef* UART_HANDLE_StartUART4Task = &huart4;
UART_HandleTypeDef* UART_HANDLE_StartUART6Task = &huart6;

#if USE_DEBUG_UART == 0
UART_HandleTypeDef* UART_HANDLE_PC = &huart5;
UART_HandleTypeDef* UART_HANDLE_StartUART2Task = &huart2;
UART_HandleTypeDef* UART_HANDLE_StartUART3Task = &huart3;

#elif USE_DEBUG_UART == 1
#pragma "SystemConf warning: make sure UART configurations have been updated in the usart.c file."

#if defined(STM32F446xx)
UART_HandleTypeDef* UART_HANDLE_PC = &huart2;
UART_HandleTypeDef* UART_HANDLE_StartUART2Task = &huart5;
UART_HandleTypeDef* UART_HANDLE_StartUART3Task = &huart3;

#elif defined(STM32F767xx)
UART_HandleTypeDef* UART_HANDLE_PC = &huart3;
UART_HandleTypeDef* UART_HANDLE_StartUART2Task = &huart2;
UART_HandleTypeDef* UART_HANDLE_StartUART3Task = &huart5;

#endif

#else
#error "SystemConf error: USE_DEBUG_UART must be set to 0 or 1, and at least one board defined."
#endif

/**
 * @}
 */
/* end - system_conf */
