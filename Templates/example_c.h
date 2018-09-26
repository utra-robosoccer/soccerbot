/**
  *****************************************************************************
  * @file    example.h
  * @author  Tyler
  * @brief   This file provides interfaces for setting and getting day
  *          information, and also defines types used to accomplish this task
  *
  * @defgroup DaysHeader Days (header)
  * @brief    Header for Days, showing the public content
  * @ingroup  Days
  * @{
  *****************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef __EXAMPLE_H__
#define __EXAMPLE_H__




/********************************* Includes **********************************/
#include <stdint.h>
#include <stdbool.h>




/********************************* Constants *********************************/
extern const uint8_t numDaysPerWeek; /**< The number of days per week */




/********************************** Types ************************************/
/** Enumerates the names of the week */
typedef enum days_e{
    MONDAY,         /**< The first day of the week     */
    TUESDAY,        /**< The second day of the week    */
    WEDNESDAY,      /**< The third day of the week     */
    THURSDAY,       /**< The fouth day of the week     */
    FRIDAY,         /**< The fifth day of the week     */
    SATURDAY,       /**< The first day of the weekend  */
    SUNDAY          /**< The second day of the weekend */
}days_t;




/****************************** Public Variables *****************************/
extern bool isAWeekend;




/***************************** Function prototypes ***************************/
// Setters
int Days_SetDay(uint8_t password, days_t dayToSet);

// Getters
days_t Days_GetDay();

// Computational-based helpers
uint32_t Days_WeeksToDays(uint32_t weeks);

/**
 * @}
 */
/* end ModuleNameHeader */

#endif /* __EXAMPLE_H__ */
