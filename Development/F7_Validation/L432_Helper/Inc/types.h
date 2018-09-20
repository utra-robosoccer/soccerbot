/**
  *****************************************************************************
  * @file    types.h
  * @author  Tyler
  *
  * @defgroup Types
  * @brief Defines types shared throughout the program
  * @{
  *****************************************************************************
  */




#ifndef TYPES_H
#define TYPES_H




/********************************* Includes **********************************/
#include <stdint.h>




/********************************** Types ************************************/
/**
 * @brief Container for passing data between threads
 */
typedef struct{
    uint8_t id;   /**< ID of motor data                             */
    uint16_t pos; /**< Only support sending back position right now */
}Data_t;




/**
 * @}
 */
/* end Types */

#endif /* TYPES_H */
