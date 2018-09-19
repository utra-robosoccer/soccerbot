/**
  *****************************************************************************
  * @file    helpers.c
  * @author  Tyler
  *
  * @defgroup Helpers
  * @brief    Helpers for computation and OS
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "helpers.h"




/******************************** Functions **********************************/
bool waitUntilNotifiedOrTimeout(
    uint32_t notificationVal,
    TickType_t timeout
)
{
    uint32_t notification;
    BaseType_t status;
    bool retval = true;

    // Wait until notified from ISR. Clear no bits on entry in case the notification
    // came while a higher priority task was executing.
    status = xTaskNotifyWait(0, notificationVal, &notification, pdMS_TO_TICKS(timeout));

    if((status != pdTRUE) || !CHECK_NOTIFICATION(notification, notificationVal)){
        retval = false;
    }

    return retval;
}




/**
 * @}
 */
/* end - Helpers */
