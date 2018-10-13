/**
 *****************************************************************************
 * @file    template_cpp.cpp
 * @author  TODO -- your name here
 * @brief   TODO -- brief description of file
 *
 * @defgroup TODO -- module name
 * @brief    TODO -- description of module
 * @{
 *****************************************************************************
 */

/********************************* Includes **********************************/
#include "HalI2CInterface.h"

/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------
using namespace i2c;
/******************************** File-local *********************************/
// Classes and structs
// ----------------------------------------------------------------------------
// Variables
// ----------------------------------------------------------------------------
// Functions
HALI2CInterface::HALI2CInterface() {;}
HALI2CInterface::~HALI2CInterface() {;}

void HALI2CInterface::assignHandle(I2C_HandleTypeDef *i2cHandlePtr) const {
	this->_I2C_Handle = i2cHandlePtr;

HAL_StatusTypeDef HALI2CInterface::memWrite(uint16_t DevAddress, uint16_t MemAddress,
		uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) const { //HAL_I2C_Mem_Write

	return HAL_I2C_Mem_Write(i2cHandlePtr, DevAddress, MemAddress,
			MemAddSize, pData, Size, Timeout);
}

HAL_StatusTypeDef HALI2CInterface::memRead(uint16_t DevAddress, uint16_t MemAddress,
		uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) const { //HAL_I2C_Mem_Read
	return HAL_I2C_Mem_Read(i2cHandlePtr, DevAddress, MemAddress, MemAddSize, pData, Size,
			Timeout);
}

HAL_StatusTypeDef HALI2CInterface::memWriteIT(uint16_t DevAddress, uint16_t MemAddress,
		uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const { //HAL_I2C_Mem_Write_IT
	return HAL_I2C_Mem_Write_IT(i2cHandlePtr, DevAddress, MemAddress, MemAddSize, pData, Size);
}

HAL_StatusTypeDef HALI2CInterface::memReadIT(uint16_t DevAddress, uint16_t MemAddress,
		uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const { //HAL_I2C_Mem_Read_IT
	return HAL_I2C_Mem_Read_IT(i2cHandlePtr, DevAddress, MemAddress, MemAddSize, pData, Size);
}
// ----------------------------------------------------------------------------

/************************** insert class name here ***************************/
// Public
// ----------------------------------------------------------------------------

// Protected
// ----------------------------------------------------------------------------

// Private
// ----------------------------------------------------------------------------

/**
 * @}
 */
/* end - module name */

