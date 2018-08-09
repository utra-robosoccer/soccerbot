/*
 * @file DynamixelProtocolV1_IO_Exports.h
 * @author Tyler
 *
 * @defgroup DynamixelProtocolV1_IO_Exports_Header Exports
 * @ingroup DynamixelProtocolV1_IO
 * @{
 */




#ifndef DYNAMIXEL_H_DYNAMIXELPROTOCOLV1_IO_EXPORTS_H_
#define DYNAMIXEL_H_DYNAMIXELPROTOCOLV1_IO_EXPORTS_H_




/********************************* Includes **********************************/
#include <stdint.h>
#include <stdbool.h>
#include "Dynamixel_Types.h"




/***************************** Function prototypes ***************************/
// Low-level transmission and reception functions
void Dynamixel_DataWriter(
        Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t* args,
        uint8_t numArgs
);

uint16_t Dynamixel_DataReader(
        Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t readAddr,
        uint8_t readLength
);

/**
 * @}
 */
/* end - DynamixelProtocolV1_IO_Header */

#endif /* DYNAMIXEL_H_DYNAMIXELPROTOCOLV1_IO_EXPORTS_H_ */
