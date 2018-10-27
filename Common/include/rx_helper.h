/**
 *****************************************************************************
 * @file    rx_helper.h
 * @author  Hannah
 * @brief   Header for the helper file used to aid StartRXTask() in freertos.cpp
 *****************************************************************************
 */




#ifndef RX_HELPER_H
#define RX_HELPER_H





/*********************************** Includes ********************************/
#include <cstdint>


/***************************** Function prototypes ***************************/
void receiveDataBuffer(void);
void initiateDMATransfer(void);
void updateStatusToPC(void);
void waitForNotificationRX(void);
void initializeVars(void);
void copyIntoBuffRx(uint8_t *copyFrom);

#endif /* RX_HELPER_H */
