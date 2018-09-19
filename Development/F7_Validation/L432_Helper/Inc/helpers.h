/**
  *****************************************************************************
  * @file    helpers.h
  * @author  Tyler
  *
  * @defgroup Header
  * @ingroup  Helpers
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
bool waitUntilNotifiedOrTimeout(
    uint32_t notificationVal,
    TickType_t timeout
);



/****************************** Inline Functions *****************************/
inline bool CHECK_NOTIFICATION(uint32_t val, uint32_t notificationMask){
    return (val & notificationMask) == notificationMask;
}

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
