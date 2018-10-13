/*
 * MockI2CInterface.h
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
#ifndef COMMON_INCLUDE_MOCKI2CINTERFACE_H_
#define COMMON_INCLUDE_MOCKI2CINTERFACE_H_

/********************************* Includes **********************************/
#include "I2CInterface.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>

/********************************** Globals **********************************/
// Constants
// ----------------------------------------------------------------------------
/************************** insert module name here **************************/
using namespace i2c;
namespace mocks{

class MockI2CInterface: public I2CInterface{
public:
	MOCK_CONST_METHOD1(assignHandle, void(I2C_HandleTypeDef *hi2c));
	MOCK_CONST_METHOD6(memWrite , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
				uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
				uint32_t Timeout));
	MOCK_CONST_METHOD6(memRead , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
					uint16_t MemAddSize, uint8_t *pData, uint16_t Size,
					uint32_t Timeout));
	MOCK_CONST_METHOD5(memWrite_IT , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
					uint16_t MemAddSize, uint8_t *pData, uint16_t Size));
	MOCK_CONST_METHOD5(memRead_IT , HAL_StatusTypeDef(uint16_t DevAddress, uint16_t MemAddress,
						uint16_t MemAddSize, uint8_t *pData, uint16_t Size));
}
// Constanst/ Types & enums
// ----------------------------------------------------------------------------

}// end namespace module_name


#endif /* COMMON_INCLUDE_MOCKI2CINTERFACE_H_ */
