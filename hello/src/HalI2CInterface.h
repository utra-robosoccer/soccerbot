/*
 * HalI2CInterface.h
 *
 *  Created on: Oct 13, 2018
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

#ifndef COMMON_INCLUDE_HALI2CINTERFACE_H
#define COMMON_INCLUDE_HALI2CINTERFACE_H

/********************************* Includes **********************************/
#include "I2CInterface.h"


/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------
/************************** insert module name here **************************/
namespace i2c{

class HALI2CInterface : public I2CInterface {

public:
	HALI2CInterface();
	~HALI2CInterface();
//
//	void assignHandle(I2C_HandleTypeDef *hi2c) override;	//HAL_I2C_Init
//	HAL_StatusTypeDef memWrite(uint16_t DevAddress, uint16_t MemAddress,
//			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
//			uint32_t Timeout) override; //HAL_I2C_Mem_Write
//	HAL_StatusTypeDef memRead(uint16_t DevAddress, uint16_t MemAddress,
//			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
//			uint32_t Timeout) override; //HAL_I2C_Mem_Read
//	HAL_StatusTypeDef memWriteIT(uint16_t DevAddress, uint16_t MemAddress,
//			uint16_t MemAddSize, uint8_t *pData, uint16_t Size) override; //HAL_I2C_Mem_Write_IT
//	HAL_StatusTypeDef memReadIT(uint16_t DevAddress, uint16_t MemAddress,
//			uint16_t MemAddSize, uint8_t *pData, uint16_t Size) override;//HAL_I2C_Mem_Read_IT
//private:
//	I2C_HandleTypeDef *hi2c = nullptr; //virtual
};

}

#endif /* HALI2CINTERFACE_H*/
