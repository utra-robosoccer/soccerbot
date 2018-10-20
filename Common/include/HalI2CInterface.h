/**
 *****************************************************************************
 * @file    HalI2CInterface.h
 * @author  Hannah L
 * @brief   Defines the HALI2CInterface class, which calls HAL functions related to I2C
 *
 * @defgroup HalI2CInterface
 * @ingroup  I2C
 * @{
 *****************************************************************************
 */

#ifndef COMMON_HALI2CINTERFACE_H_
#define COMMON_HALI2CINTERFACE_H_

/********************************* Includes **********************************/

#include "I2CInterface.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_def.h"
#include "MPU6050.h"

/****************************** HAL I2C Interface ****************************/
using namespace i2c;

class HALI2CInterface: public I2CInterface {
public:
	HALI2CInterface();
	~HALI2CInterface();
	void assignHandle(I2C_HandleTypeDef *i2cHandlePtr) const override;//HAL_I2C_Init
	HAL_StatusTypeDef memWrite(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
			uint32_t Timeout) const override; //HAL_I2C_Mem_Write
	HAL_StatusTypeDef memRead(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
			uint32_t Timeout) const override; //HAL_I2C_Mem_Read
	HAL_StatusTypeDef memWriteIT(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const override; //HAL_I2C_Mem_Write_IT
	HAL_StatusTypeDef memReadIT(uint16_t DevAddress, uint16_t MemAddress,
			uint16_t MemAddSize, uint8_t *pData, uint16_t Size) const override; //HAL_I2C_Mem_Read_IT
private:
	I2C_HandleTypeDef *i2cHandlePtr = nullptr; //virtual
};

#endif /* COMMON_HALI2CINTERFACE_H_ */
