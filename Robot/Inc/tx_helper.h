/**
  *****************************************************************************
  * @file    tx_helper.h
  * @author  TODO -- your name here
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup Header
  * @ingroup  TODO -- module name defined in template.c
  * @{
  *****************************************************************************
  */




#ifndef __TX_HELPER_H__
#define __TX_HELPER_H__

#ifdef __cplusplus
extern "C" {
#endif

/********************************* Includes **********************************/
#include <stdint.h>




/********************************** Macros ***********************************/




/********************************* Constants *********************************/




/********************************** Types ************************************/




/****************************** Public Variables *****************************/




/***************************** Function prototypes ***************************/
void shiftNotificationMask(void);
void copySensorDataToSend(void);
void transmitStatusFromPC(void);
void waitForNotificationTX(void);



/****************************** Inline Functions *****************************/




/**
 * @}
 */
/* end TODO -- module name defined on line 7 */
#ifdef __cplusplus
}
#endif

#endif /* __TX_HELPER_H__ */
