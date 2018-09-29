/**
  *****************************************************************************
  * @file    sharedMacros.h
  * @author  Tyler
  * @brief   This file defines macros used throughout various parts of the
  *          system that engage in non-blocking I/O transfers
  *
  * @defgroup sharedMacros Shared macros (header)
  * @brief    Defines macros used throughout various parts of the system that
  *           engage in non-blocking I/O transfers
  * @ingroup  FreeRTOS
  * @{
  *****************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef __SHAREDMACROS_H__
#define __SHAREDMACROS_H__




/********************************** Macros ***********************************/
// Task notification values
#define NOTIFIED_FROM_TX_ISR 0x80   /**< Notification from a transmitter ISR */
#define NOTIFIED_FROM_RX_ISR 0x20   /**< Notification from a receiver ISR    */
#define NOTIFIED_FROM_TASK 0x40     /**< Notification from another task      */

// Timeouts
#define MAX_DELAY_TIME 2 /**< The maximum timeout, in milliseconds, that a task
                              should remain blocked on a typical I/O call in
                              this system. Any longer than this would typically
                              indicate a failure of some sort that should be
                              caught and handled so as to not interfere with
                              program execution */

/**
 * @}
 */
/* end sharedMacros */

#endif /* __SHAREDMACROS_H__ */
