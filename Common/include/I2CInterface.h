/*
 * I2CInterface.h
 *
 *  Created on: Oct 6, 2018
 *      Author: Hannah
 */

/**
 *****************************************************************************
 * @file    I2CInterface.h
 * @author  Hannah L
 * @brief   Defines the I2C Interface class that
 *
 * @defgroup I2CInterface
 * @ingroup  I2C
 * @{
 *****************************************************************************
 */

#ifndef COMMON_INCLUDE_I2CINTERFACE_H_
#define COMMON_INCLUDE_I2CINTERFACE_H_

/********************************* Includes **********************************/
#include <cstdint>
#include "SystemConf.h"
#include "i2c.h"

/******************************* I2C Interface *******************************/
namespace i2c {

// Classes and Structs
class I2CInterface {
public:
	virtual ~I2CInterface();

//	HAL-related members
	virtual HAL_StatusTypeDef memWrite(I2C_HandleTypeDef *i2cHandlePtr,
			uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
			uint8_t *pData, uint16_t Size, uint32_t Timeout) const = 0; //HAL_I2C_Mem_Write
	virtual HAL_StatusTypeDef memRead(I2C_HandleTypeDef *i2cHandlePtr,
			uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
			uint8_t *pData, uint16_t Size, uint32_t Timeout) const = 0; //HAL_I2C_Mem_Read
	virtual HAL_StatusTypeDef memWriteIT(I2C_HandleTypeDef *i2cHandlePtr,
			uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
			uint8_t *pData, uint16_t Size) const = 0; //HAL_I2C_Mem_Write_IT
	virtual HAL_StatusTypeDef memReadIT(I2C_HandleTypeDef *i2cHandlePtr,
			uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
			uint8_t *pData, uint16_t Size) const = 0;	//HAL_I2C_Mem_Read_IT

};
// ----------------------------------------------------------------------------
}// end namespace i2c

#endif /* COMMON_INCLUDE_I2CINTERFACE_H_ */
