/**
  *****************************************************************************
  * @file    template.h
  * @author  TODO -- your name here
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup Header
  * @ingroup  TODO -- module name defined in template.c
  * @{
  *****************************************************************************
  */


#ifndef __RX_HELPER_H__
#define __RX_HELPER_H__

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
void parsePacket(void);
void initiateDMATransfer(void);
void updateStatusToPC(void);
void waitForNotificationRX(void);
void initializeVars(void);

/****************************** Inline Functions *****************************/




/**
 * @}
 */
/* end TODO -- module name defined on line 7 */
#ifdef __cplusplus
}
#endif

#endif /* __RX_HELPER_H__ */
