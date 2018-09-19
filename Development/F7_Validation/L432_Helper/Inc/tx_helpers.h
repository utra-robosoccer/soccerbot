/**
  *****************************************************************************
  * @file    tx_helpers.h
  * @author  Tyler
  *
  * @defgroup Header
  * @ingroup  TX_Helpers
  * @{
  *****************************************************************************
  */




#ifndef TX_HELPERS_H
#define TX_HELPERS_H




/********************************* Includes **********************************/
#include "types.h"




/***************************** Function prototypes ***************************/
void update_buffer_contents(Data_t* data);
void transmit_buffer_contents(void);




/**
 * @}
 */
/* end Header */

#endif /* TX_HELPERS_H */
