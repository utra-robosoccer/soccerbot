/*
 * HalI2CInterface.h
 *
 *  Created on: Oct 13, 2018
 *      Author: Hannah
 */

#ifndef COMMON_HALI2CINTERFACE_H_
#define COMMON_HALI2CINTERFACE_H_

#include "I2CInterface.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6050.h"

using namespace i2c;

class HALI2CInterface : public I2CInterface {
public:
	HALI2CInterface();
	~HALI2CInterface();
	void assignHandle(I2C_HandleTypeDef *i2cHandlePtr) const override;	//HAL_I2C_Init
	HAL_StatusTypeDef memWrite(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
			uint32_t Timeout) const override; //HAL_I2C_Mem_Write
	HAL_StatusTypeDef memRead(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
			uint32_t Timeout) const override; //HAL_I2C_Mem_Read
	HAL_StatusTypeDef memWriteIT(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const override; //HAL_I2C_Mem_Write_IT
	HAL_StatusTypeDef memReadIT(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const override;//HAL_I2C_Mem_Read_IT
private:
	I2C_HandleTypeDef *i2cHandlePtr = nullptr; //virtual
};



#endif /* COMMON_HALI2CINTERFACE_H_ */
