/*
 * I2CInterface.h
 *
 *  Created on: Oct 6, 2018
 *      Author: Hannah
 */

/**
 *****************************************************************************
 * @file    template_cpp.h
 * @author  TODO -- your name here
 * @brief   TODO -- briefly describe this file
 *
 * @defgroup Header
 * @ingroup  TODO -- module name defined in template_cpp.cpp
 * @{
 *****************************************************************************
 */

#ifndef COMMON_INCLUDE_I2CINTERFACE_H_
#define COMMON_INCLUDE_I2CINTERFACE_H_

/********************************* Includes **********************************/
#include <cstdint>
#include "SystemConf.h"
#include "i2c.h"

/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------
/************************** insert module name here **************************/
namespace i2c {
// Constanst/ Types & enums
// ----------------------------------------------------------------------------

// Classes and Structs
class I2CInterface {
public:
	virtual I2CInterface();
	virtual ~I2CInterface();
//	virtual void assignHandle(I2C_HandleTypeDef *hi2c) = 0;
//
////	HAL-related members
//	virtual HAL_StatusTypeDef memWrite(uint16_t DevAddress, uint16_t MemAddress,
//			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
//			uint32_t Timeout) = 0; //HAL_I2C_Mem_Write
//	virtual HAL_StatusTypeDef memRead(uint16_t DevAddress, uint16_t MemAddress,
//			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
//			uint32_t Timeout) = 0; //HAL_I2C_Mem_Read
//	virtual HAL_StatusTypeDef memWriteIT(uint16_t DevAddress,
//			uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
//			uint16_t Size) = 0; //HAL_I2C_Mem_Write_IT
//	virtual HAL_StatusTypeDef memReadIT(uint16_t DevAddress,
//			uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData,
//			uint16_t Size) = 0;	//HAL_I2C_Mem_Read_IT

};

// ----------------------------------------------------------------------------

// Functions
// ----------------------------------------------------------------------------

}// end namespace module_name


#endif /* COMMON_INCLUDE_I2CINTERFACE_H_ */
