/**
 *****************************************************************************
 * @file
 * @author  Hannah
 * @brief   Header for the helper file used to aid StartTxTask() in freertos.cpp
 *****************************************************************************
 */




#ifndef TX_HELPER_H
#define TX_HELPER_H


/********************************* Includes **********************************/
#include "BufferBase.h"


/***************************** Function prototypes ***************************/
void copySensorDataToSend(buffer::BufferMaster* p_buffer_master);

#endif /* TX_HELPER_H */
