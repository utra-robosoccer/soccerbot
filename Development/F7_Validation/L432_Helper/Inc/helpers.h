/**
  *****************************************************************************
  * @file    helpers.h
  * @author  Tyler
  *
  * @defgroup Header
  * @ingroup Helpers
  * @{
  *****************************************************************************
  */




#ifndef HELPERS_H
#define HELPERS_H




/********************************* Includes **********************************/
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"




/***************************** Function prototypes ***************************/
/**
 * @brief Blocks a thread until it received the specified notification or
 *        the allowed wait period elapses. Uses the FreeRTOS task notification
 *        API
 * @param notificationVal The notification value which can unblock this thread
 * @param timeout The allowed wait period in milliseconds
 */
bool waitUntilNotifiedOrTimeout(
    uint32_t notificationVal,
    TickType_t timeout
);



/****************************** Inline Functions *****************************/
/**
 * @brief   Checks whether val has the bits set related to notificationMask
 * @param   val The value of the notification to be checked
 * @param   notificationMask The mask to which val will be compared
 * @returns true if the bits set in notificationMask are also set in val,
 *          otherwise false
 */
inline bool CHECK_NOTIFICATION(uint32_t val, uint32_t notificationMask){
    return (val & notificationMask) == notificationMask;
}

/**
 * @brief Computed a packet checksum assuming it is to be transmitted to or
 *        received from a device interpreting the bytes as per the Dynamixel
 *        version 1.0 protocol
 * @param arr Pointer to the beginning of the packet buffer
 * @param length The size of the buffer
 */
inline uint8_t Dynamixel_ComputeChecksum(uint8_t *arr, int length){
    uint8_t accumulate = 0;

    /* Loop through the array starting from the 2nd element of the array and
     * finishing before the last since the last is where the checksum will
     * be stored */
    for(uint8_t i = 2; i < length - 1; i++){
        accumulate += arr[i];
    }

    return (~accumulate) & 0xFF; // Lower 8 bits of the logical NOT of the sum
}




/**
 * @}
 */
/* end - Headers */

#endif /* HELPERS_H */
