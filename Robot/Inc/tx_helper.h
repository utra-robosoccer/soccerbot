/**
 *****************************************************************************
 * @file    tx_helper.h
 * @author  Hannah
 * @brief   Header for the helper file used to aid StartTxTask() in freertos.cpp
 *****************************************************************************
 */

#ifndef TX_HELPER_H
#define TX_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

/********************************* Includes **********************************/
#include <stdint.h>

/***************************** Function prototypes ***************************/
void shiftNotificationMask(void);
void copySensorDataToSend(void);
void transmitStatusFromPC(void);
void waitForNotificationTX(void);

#ifdef __cplusplus
}
#endif

#endif /* TX_HELPER_H */
