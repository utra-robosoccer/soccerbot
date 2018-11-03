/**
 *****************************************************************************
 * @file    tx_helper.h
 * @author  Hannah
 * @brief   Header for the helper file used to aid StartTxTask() in freertos.cpp
 *****************************************************************************
 */




#ifndef TX_HELPER_H
#define TX_HELPER_H

#include "BufferBase.h"


/***************************** Function prototypes ***************************/
void shiftNotificationMask(void);
void copySensorDataToSend(buffer::BufferMaster*);
void transmitStatusFromPC(void);
void waitForNotificationTX(void);

#endif /* TX_HELPER_H */
