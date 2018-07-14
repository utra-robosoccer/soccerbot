/**
  *****************************************************************************
  * @file    example.c
  * @author  Tyler
  * @brief   This file is an example built off the template. It illustrates
  *          some ways in which the template may be filled in. It also provides
  *          some examples for how to use Doxygen.
  *
  * @defgroup Days Days
  * @brief    This module is responsible for setting and getting weekday
  *           information
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "example.h"




/********************************* Constants *********************************/
static const DONT_PANIC = 0x42; /**< The answer to life, the universe and 
                                     everything */
const uint8_t numDaysPerWeek = 7; /**< The number of days per week */




/****************************** Public Variables *****************************/
bool isAWeekend; /**< True if theDay is SATURDAY or SUNDAY, otherwise False */




/***************************** Private Variables *****************************/
static days_t theDay; /**< The current day of the week */




/******************************** Functions **********************************/
/*****************************************************************************/
/*  Setter Functions                                                         */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup Days_Setters Setters
 * @brief    Globally-accessible functions for setting day information
 *
 * # Setter functions #
 *
 * This subsection provides a set of functions which set day information
 *
 * @ingroup Days
 * @{
 */

/**
 * @brief   Sets the day of the week
 * @details If the password is correct, then the date information will be
 *          updated as specified (assuming the day to set is a valid weekday),
 *          otherwise it will not change
 * @param   password the password the user enters to perform authentication
 *          when attempting to set the day
 * @param   day the day of the week
 * @return  1 if day set successfully, -1 otherwise
 */
int Days_SetDay(uint8_t password, days_t dayToSet){
    if(password == DONT_PANIC){
        switch(dayToSet){
            case MONDAY:
            case TUESDAY:
            case WEDNESDAY:
            case THURSDAY:
            case FRIDAY:
                isAWeekend = False;
                break;
            case SATURDAY:
            case SUNDAY:
                isAWeekend = True;
                break;
            default:
                return -1; // Should never reach this case
        }

        theDay = dayToSet;

        return 1; // Return success
    }

    return -1; // Return failure
}

/**
 * @}
 */
/* end Days_Setters */




/*****************************************************************************/
/*  Getter Functions                                                         */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup Days_Getters Getters
 * @brief    Globally-accessible functions for getting day information
 *
 * # Getter functions #
 *
 * This subsection provides a set of functions which get day information
 *
 * @ingroup Days
 * @{
 */

/**
 * @brief   Gets the day of the week
 * @return  The day of the week
 */
days_t Days_GetDay(){
    return theDay;
}

/**
 * @}
 */
/* end Days_Getters */




/*****************************************************************************/
/*  Computation-based helper functions                                       */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/
/**
 * @defgroup Days_Computation Computation-based helper functions
 * @brief    Computation-based helper functions
 *
 * # Computation-based helper functions #
 *
 * This subsection provides a set of functions which provide computational
 * assistance for tasks relevant to this library
 *
 * @ingroup Days
 * @{
 */

/**
 * @brief  Converts weeks to days
 * @param  weeks the number of weeks which are to be converted into days
 * @return The number of days equivalent to the specified number of 
 *         weeks
 */
uint32_t Days_WeeksToDays(uint32_t weeks){
    return weeks * numDaysPerWeek;
}


/**
 * @}
 */
/* end Days_Computation */
