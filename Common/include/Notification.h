/**
  *****************************************************************************
  * @file
  * @author  Tyler
  * @brief   This file defines resources used throughout various parts of the
  *          system that engage in non-blocking I/O transfers
  *
  * @defgroup Notification Header
  * @brief    Defines resources used throughout various parts of the system
  *           that engage in non-blocking I/O transfers
  * @ingroup  FreeRTOS
  * @{
  *****************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef SHAREDMACROS_H
#define SHAREDMACROS_H




/********************************* Includes **********************************/
#include <stdbool.h>
#include <stdint.h>




/********************************** Macros ***********************************/
// Task notification values
#define NOTIFIED_FROM_TX_ISR 0x80   /**< Notification from a transmitter ISR */
#define NOTIFIED_FROM_RX_ISR 0x20   /**< Notification from a receiver ISR    */
#define NOTIFIED_FROM_TASK 0x40     /**< Notification from another task      */


// Timeouts
/**
 * @brief The maximum timeout, in milliseconds, that a task may remain blocked
 *        on a typical I/O call in this system. Any longer than this would
 *        indicate a failure of some sort that should be caught and handled so
 *        as to not interfere with program execution
 */
#define MAX_DELAY_TIME pdMS_TO_TICKS(2)

/****************************** Public functions *****************************/
inline bool CHECK_NOTIFICATION(uint32_t val, uint32_t mask){
    return (val & mask) == mask;
}


/**
 * @}
 */
/* end sharedMacros */

#endif /* __SHAREDMACROS_H__ */
