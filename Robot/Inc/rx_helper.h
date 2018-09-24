/**
 *****************************************************************************
 * @file    rx_helper.h
 * @author  Hannah
 * @brief   Header for helper file for function StartRXTask in freertos.cpp
 *****************************************************************************
 */

#ifndef RX_HELPER_H
#define RX_HELPER_H

#ifdef __cplusplus
extern "C" {
#endif

/********************************* Includes **********************************/
#include <stdint.h>

/***************************** Function prototypes ***************************/
void receiveDataBuffer(void);
void initiateDMATransfer(void);
void updateStatusToPC(void);
void waitForNotificationRX(void);
void initializeVars(void);

#ifdef __cplusplus
}
#endif

#endif /* RX_HELPER_H */
