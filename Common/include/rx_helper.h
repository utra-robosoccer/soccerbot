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
#include <stddef.h>


/***************************** Function prototypes ***************************/
void initializeVars(void);
void parseByteSequence(uint8_t *in_buff, size_t in_buff_size, bool& complete);
void copyParsedData(void);

enum class RxParseState {
    CHECKING_HEADER,
    READING_DATA
};

#endif /* RX_HELPER_H */
